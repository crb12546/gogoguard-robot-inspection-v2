from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from gogoguard_contracts import is_safe_external_id, utc_now

from .checkpoint_alignment import CheckpointViewAlignment


@dataclass(frozen=True)
class RuntimeCheckpoint:
    checkpoint_id: str
    route_progress_index: int
    settle_before_s: float
    body_yaw_rad: float
    camera_pan_deg: float = 0.0
    camera_tilt_deg: float = 0.0
    camera_roll_deg: float = 0.0
    spin: bool = True
    dwell_s: float = 3.0
    legacy_spin_only: bool = False


@dataclass(frozen=True)
class NavigationMission:
    mission_id: str
    map_version: str
    route_id: str
    mission_hash: str
    checkpoints: tuple[RuntimeCheckpoint, ...]
    decision_mode: str = "platform"
    verdict_timeout_s: int = 15
    max_retake_attempts: int = 2


def load_navigation_mission(
    path: Path,
    *,
    expected_map_version: str,
    expected_route_id: str,
    route_point_count: int,
) -> NavigationMission:
    path = Path(path)
    if not path.is_file():
        return NavigationMission("", expected_map_version, expected_route_id, "", ())
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("schema") != "gogoguard.navigation_mission.v1":
        raise ValueError("navigation mission schema is invalid")
    mission_hash = str(value.get("missionHash") or "")
    canonical = {key: item for key, item in value.items() if key != "missionHash"}
    actual_hash = hashlib.sha256(
        json.dumps(
            canonical, ensure_ascii=False, sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
    ).hexdigest()
    if mission_hash != actual_hash:
        raise ValueError("navigation mission hash is invalid")
    mission_id = str(value.get("missionId") or "")
    if not is_safe_external_id(mission_id):
        raise ValueError("navigation mission id is invalid")
    if value.get("mapVersion") != expected_map_version or value.get("routeId") != expected_route_id:
        raise ValueError("navigation mission map or route binding is invalid")
    raw_checkpoints = value.get("checkpoints")
    if not isinstance(raw_checkpoints, list) or len(raw_checkpoints) > 256:
        raise ValueError("navigation mission checkpoints are invalid")
    checkpoints: list[RuntimeCheckpoint] = []
    previous = -1
    seen: set[str] = set()
    for raw in raw_checkpoints:
        if not isinstance(raw, dict) or raw.get("action") not in {"body_spin_360", "platform_checkpoint"}:
            raise ValueError("navigation checkpoint action is invalid")
        checkpoint_id = str(raw.get("checkpointId") or "")
        index = raw.get("routeProgressIndex")
        settle_s = raw.get("settleBeforeS")
        legacy = raw.get("action") == "body_spin_360"
        target_yaw = raw.get("targetYawRad") if legacy else raw.get("bodyYawRad")
        if not is_safe_external_id(checkpoint_id) or checkpoint_id in seen:
            raise ValueError("navigation checkpoint id is invalid")
        if (
            isinstance(index, bool)
            or not isinstance(index, int)
            or index < previous
            or index < 0
            or index >= route_point_count
        ):
            raise ValueError("navigation checkpoint route index is invalid")
        if (
            isinstance(settle_s, bool)
            or not isinstance(settle_s, (int, float))
            or not math.isfinite(settle_s)
            or not 0.5 <= float(settle_s) <= 5.0
        ):
            raise ValueError("navigation checkpoint settle time is invalid")
        if (
            isinstance(target_yaw, bool)
            or not isinstance(target_yaw, (int, float))
            or not math.isfinite(target_yaw)
            or (legacy and not math.isclose(float(target_yaw), 2.0 * math.pi, abs_tol=1.0e-3))
        ):
            raise ValueError("navigation checkpoint yaw is invalid")
        camera = raw.get("camera") if isinstance(raw.get("camera"), dict) else {}
        camera_angles = tuple(
            float(camera.get(name) or 0.0) for name in ("pan", "tilt", "roll")
        )
        if any(not math.isfinite(angle) for angle in camera_angles):
            raise ValueError("navigation checkpoint camera angle is invalid")
        spin = raw.get("spin", True)
        if not isinstance(spin, bool):
            raise ValueError("navigation checkpoint spin is invalid")
        dwell_s = raw.get("dwellSec", 3.0)
        if (
            isinstance(dwell_s, bool)
            or not isinstance(dwell_s, (int, float))
            or not 0.0 <= float(dwell_s) <= 30.0
        ):
            raise ValueError("navigation checkpoint dwell is invalid")
        seen.add(checkpoint_id)
        previous = index
        checkpoints.append(
            RuntimeCheckpoint(
                checkpoint_id,
                index,
                float(settle_s),
                float(target_yaw),
                *camera_angles,
                spin,
                float(dwell_s),
                legacy,
            )
        )
    decision_mode = str(value.get("decisionMode") or "platform")
    if decision_mode not in {"platform", "local_operator"}:
        raise ValueError("navigation mission decision mode is invalid")
    verdict_timeout = value.get("verdictTimeoutSec", 15)
    max_retakes = value.get("maxRetakeAttempts", 2)
    if isinstance(verdict_timeout, bool) or not isinstance(verdict_timeout, int) or not 5 <= verdict_timeout <= 120:
        raise ValueError("navigation mission verdict timeout is invalid")
    if isinstance(max_retakes, bool) or not isinstance(max_retakes, int) or not 0 <= max_retakes <= 5:
        raise ValueError("navigation mission max retakes is invalid")
    return NavigationMission(
        mission_id,
        expected_map_version,
        expected_route_id,
        mission_hash,
        tuple(checkpoints),
        decision_mode,
        verdict_timeout,
        max_retakes,
    )


class CheckpointExecutor:
    """Pure true-stop -> pose -> platform verdict -> suffix-resume flow."""

    def __init__(self, mission: NavigationMission) -> None:
        self.mission = mission
        self.cursor = 0
        self.phase = "TRAVELING"
        self.active_checkpoint: RuntimeCheckpoint | None = None
        self.stable_since: float | None = None
        self.last_stop_receipt: dict[str, Any] | None = None
        self.last_completed_checkpoint_id: str | None = None
        self.failure_reason: str | None = None
        self.attempt = 1
        self.last_control_id: str | None = None
        self.view_alignment: CheckpointViewAlignment | None = None

    def reset(self) -> None:
        self.__init__(self.mission)

    @property
    def active(self) -> bool:
        return self.phase not in {"TRAVELING", "FAILED"}

    def observe_progress(self, route_progress_index: int) -> bool:
        if self.phase != "TRAVELING" or self.cursor >= len(self.mission.checkpoints):
            return False
        checkpoint = self.mission.checkpoints[self.cursor]
        if int(route_progress_index) < checkpoint.route_progress_index:
            return False
        self.active_checkpoint = checkpoint
        self.view_alignment = None
        self.phase = "PAUSING"
        return True

    def route_goal_cancelled(self) -> None:
        if self.phase != "PAUSING":
            raise RuntimeError("checkpoint is not waiting for route cancellation")
        self.phase = "SETTLING"
        self.stable_since = None

    def observe_stop(
        self,
        now: float,
        *,
        motion_authorized: bool,
        command_linear_mps: float,
        command_angular_rps: float,
        robot_linear_mps: float,
        robot_angular_rps: float,
        route_progress_index: int,
    ) -> bool:
        if self.phase != "SETTLING" or self.active_checkpoint is None:
            return False
        stopped = (
            motion_authorized is False
            and abs(command_linear_mps) <= 0.02
            and abs(command_angular_rps) <= 0.03
            and abs(robot_linear_mps) <= 0.02
            and abs(robot_angular_rps) <= 0.03
        )
        if not stopped:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = float(now)
            return False
        stable_for = max(0.0, float(now) - self.stable_since)
        if stable_for < self.active_checkpoint.settle_before_s:
            return False
        self.last_stop_receipt = {
            "schema": "gogoguard.navigation_stop_receipt.v1",
            "missionId": self.mission.mission_id,
            "checkpointId": self.active_checkpoint.checkpoint_id,
            "pauseRequestId": (
                f"{self.mission.mission_id}:{self.active_checkpoint.checkpoint_id}:"
                f"{self.cursor}"
            ),
            "mapVersion": self.mission.map_version,
            "routeId": self.mission.route_id,
            "routeProgressIndex": int(route_progress_index),
            "stopped": True,
            "motionAuthorized": False,
            "linearSpeedMps": float(robot_linear_mps),
            "angularSpeedRps": float(robot_angular_rps),
            "stableForS": stable_for,
            "observedAt": utc_now(),
        }
        self.phase = (
            "SPIN_REQUESTED"
            if self.active_checkpoint.legacy_spin_only
            else "POSE_REQUESTED"
        )
        return True

    def pose_started(self) -> None:
        if self.phase != "POSE_REQUESTED":
            raise RuntimeError("checkpoint pose was not requested")
        self.phase = "POSING"

    def pose_completed(self) -> None:
        if self.phase not in {"POSE_REQUESTED", "POSING"}:
            raise RuntimeError("checkpoint pose is not active")
        self.phase = "WAITING_PLATFORM"

    def set_view_alignment(self, alignment: CheckpointViewAlignment) -> None:
        if self.active_checkpoint is None:
            raise RuntimeError("checkpoint is not active")
        self.view_alignment = alignment

    def request_spin(self) -> None:
        if self.phase != "WAITING_PLATFORM":
            raise RuntimeError("checkpoint is not ready for platform capture")
        self.phase = "SPIN_REQUESTED" if self.active_checkpoint and self.active_checkpoint.spin else "WAITING_VERDICT"

    def spin_started(self) -> None:
        if self.phase != "SPIN_REQUESTED":
            raise RuntimeError("checkpoint spin was not requested")
        self.phase = "SPINNING"

    def spin_completed(self) -> None:
        if self.phase != "SPINNING" or self.active_checkpoint is None:
            raise RuntimeError("checkpoint spin is not active")
        if self.active_checkpoint.legacy_spin_only:
            self.complete_checkpoint()
        else:
            self.phase = "WAITING_VERDICT"

    def apply_platform_control(self, control: dict[str, Any]) -> str:
        if not isinstance(control, dict) or self.active_checkpoint is None:
            return "ignored"
        control_id = str(control.get("controlId") or "")
        if not control_id or control_id == self.last_control_id:
            return "duplicate"
        try:
            control_attempt = int(control.get("attempt") or 0)
        except (TypeError, ValueError):
            return "invalid"
        if (
            control.get("missionId") != self.mission.mission_id
            or control.get("checkpointId") != self.active_checkpoint.checkpoint_id
            or control_attempt != self.attempt
        ):
            return "stale"
        action = control.get("action")
        if action == "capture" and self.phase == "WAITING_PLATFORM":
            self.request_spin()
        elif action == "retake" and self.phase == "WAITING_VERDICT":
            if self.attempt > self.mission.max_retake_attempts:
                return "retake_limit"
            self.attempt += 1
            self.phase = "WAITING_PLATFORM"
        elif action in {"continue", "skip", "timeout"} and self.phase == "WAITING_VERDICT":
            self.complete_checkpoint()
        else:
            return "out_of_phase"
        self.last_control_id = control_id
        return str(action)

    def complete_checkpoint(self) -> None:
        if self.active_checkpoint is None:
            raise RuntimeError("checkpoint is not active")
        self.last_completed_checkpoint_id = self.active_checkpoint.checkpoint_id
        self.cursor += 1
        self.active_checkpoint = None
        self.view_alignment = None
        self.stable_since = None
        self.attempt = 1
        self.phase = "TRAVELING"

    def fail(self, reason: str) -> None:
        self.failure_reason = str(reason)
        self.phase = "FAILED"

    def status(self) -> dict[str, Any]:
        return {
            "missionId": self.mission.mission_id or None,
            "missionHash": self.mission.mission_hash or None,
            "decisionMode": self.mission.decision_mode,
            "phase": self.phase,
            "checkpointCount": len(self.mission.checkpoints),
            "completedCheckpointCount": self.cursor,
            "activeCheckpointId": (
                self.active_checkpoint.checkpoint_id if self.active_checkpoint else None
            ),
            "activeRouteProgressIndex": (
                self.active_checkpoint.route_progress_index if self.active_checkpoint else None
            ),
            "lastCompletedCheckpointId": self.last_completed_checkpoint_id,
            "attempt": self.attempt if self.active_checkpoint else None,
            "verdictTimeoutSec": self.mission.verdict_timeout_s,
            "maxRetakeAttempts": self.mission.max_retake_attempts,
            "bodyYawRad": (
                self.active_checkpoint.body_yaw_rad if self.active_checkpoint else None
            ),
            "camera": (
                {
                    "pan": (
                        self.view_alignment.camera_pan_deg
                        if self.view_alignment is not None
                        else self.active_checkpoint.camera_pan_deg
                    ),
                    "tilt": self.active_checkpoint.camera_tilt_deg,
                    "roll": self.active_checkpoint.camera_roll_deg,
                }
                if self.active_checkpoint else None
            ),
            "observationAlignment": (
                self.view_alignment.as_status()
                if self.view_alignment is not None
                else None
            ),
            "spin": self.active_checkpoint.spin if self.active_checkpoint else None,
            "dwellSec": (
                self.active_checkpoint.dwell_s if self.active_checkpoint else None
            ),
            "lastStopReceipt": self.last_stop_receipt,
            "failureReason": self.failure_reason,
        }
