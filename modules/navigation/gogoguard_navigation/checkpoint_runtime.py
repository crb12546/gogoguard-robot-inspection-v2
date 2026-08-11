from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from gogoguard_contracts import utc_now


SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")


@dataclass(frozen=True)
class RuntimeCheckpoint:
    checkpoint_id: str
    route_progress_index: int
    settle_before_s: float
    target_yaw_rad: float


@dataclass(frozen=True)
class NavigationMission:
    mission_id: str
    map_version: str
    route_id: str
    mission_hash: str
    checkpoints: tuple[RuntimeCheckpoint, ...]


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
    if not SAFE_ID.fullmatch(mission_id):
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
        if not isinstance(raw, dict) or raw.get("action") != "body_spin_360":
            raise ValueError("navigation checkpoint action is invalid")
        checkpoint_id = str(raw.get("checkpointId") or "")
        index = raw.get("routeProgressIndex")
        settle_s = raw.get("settleBeforeS")
        target_yaw = raw.get("targetYawRad")
        if not SAFE_ID.fullmatch(checkpoint_id) or checkpoint_id in seen:
            raise ValueError("navigation checkpoint id is invalid")
        if (
            isinstance(index, bool)
            or not isinstance(index, int)
            or index <= previous
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
            or not math.isclose(float(target_yaw), 2.0 * math.pi, abs_tol=1.0e-3)
        ):
            raise ValueError("navigation checkpoint must request one 360 degree spin")
        seen.add(checkpoint_id)
        previous = index
        checkpoints.append(
            RuntimeCheckpoint(checkpoint_id, index, float(settle_s), float(target_yaw))
        )
    return NavigationMission(
        mission_id,
        expected_map_version,
        expected_route_id,
        mission_hash,
        tuple(checkpoints),
    )


class CheckpointExecutor:
    """Pure true-stop -> Nav2 Spin -> suffix-resume checkpoint flow."""

    def __init__(self, mission: NavigationMission) -> None:
        self.mission = mission
        self.cursor = 0
        self.phase = "TRAVELING"
        self.active_checkpoint: RuntimeCheckpoint | None = None
        self.stable_since: float | None = None
        self.last_stop_receipt: dict[str, Any] | None = None
        self.last_completed_checkpoint_id: str | None = None
        self.failure_reason: str | None = None

    def reset(self) -> None:
        self.__init__(self.mission)

    @property
    def active(self) -> bool:
        return self.phase in {"PAUSING", "SETTLING", "SPIN_REQUESTED", "SPINNING"}

    def observe_progress(self, route_progress_index: int) -> bool:
        if self.phase != "TRAVELING" or self.cursor >= len(self.mission.checkpoints):
            return False
        checkpoint = self.mission.checkpoints[self.cursor]
        if int(route_progress_index) < checkpoint.route_progress_index:
            return False
        self.active_checkpoint = checkpoint
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
        self.phase = "SPIN_REQUESTED"
        return True

    def spin_started(self) -> None:
        if self.phase != "SPIN_REQUESTED":
            raise RuntimeError("checkpoint spin was not requested")
        self.phase = "SPINNING"

    def spin_completed(self) -> None:
        if self.phase != "SPINNING" or self.active_checkpoint is None:
            raise RuntimeError("checkpoint spin is not active")
        self.last_completed_checkpoint_id = self.active_checkpoint.checkpoint_id
        self.cursor += 1
        self.active_checkpoint = None
        self.stable_since = None
        self.phase = "TRAVELING"

    def fail(self, reason: str) -> None:
        self.failure_reason = str(reason)
        self.phase = "FAILED"

    def status(self) -> dict[str, Any]:
        return {
            "missionId": self.mission.mission_id or None,
            "missionHash": self.mission.mission_hash or None,
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
            "lastStopReceipt": self.last_stop_receipt,
            "failureReason": self.failure_reason,
        }
