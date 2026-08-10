from __future__ import annotations

import math
import re
from dataclasses import dataclass, replace

from gogoguard_contracts import (
    CheckpointPlan,
    MissionPlan,
    MissionState,
    MissionStatus,
    NavigationStopReceipt,
    utc_now,
)


SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")


@dataclass(frozen=True)
class MissionTransition:
    action: str
    status: MissionStatus
    checkpoint: CheckpointPlan | None = None


def validate_mission_plan(plan: MissionPlan) -> MissionPlan:
    for name, value in (
        ("mission_id", plan.mission_id),
        ("map_version", plan.map_version),
        ("route_id", plan.route_id),
    ):
        if not SAFE_ID.fullmatch(value):
            raise ValueError(f"{name} is invalid")
    if not isinstance(plan.offline_continue_after_evidence, bool):
        raise ValueError("offline continue policy must be boolean")
    last_index = -1
    checkpoint_ids: set[str] = set()
    for checkpoint in plan.checkpoints:
        if not SAFE_ID.fullmatch(checkpoint.checkpoint_id):
            raise ValueError("checkpoint_id is invalid")
        if checkpoint.checkpoint_id in checkpoint_ids:
            raise ValueError("checkpoint_id must be unique")
        checkpoint_ids.add(checkpoint.checkpoint_id)
        if (
            isinstance(checkpoint.route_progress_index, bool)
            or not isinstance(checkpoint.route_progress_index, int)
            or checkpoint.route_progress_index < 0
        ):
            raise ValueError("checkpoint route index must be a non-negative integer")
        if checkpoint.route_progress_index <= last_index:
            raise ValueError("checkpoints must have strictly increasing route indexes")
        last_index = checkpoint.route_progress_index
        view_ids: set[str] = set()
        for view in checkpoint.views:
            if not SAFE_ID.fullmatch(view.view_id) or view.view_id in view_ids:
                raise ValueError("view_id must be valid and unique within a checkpoint")
            view_ids.add(view.view_id)
            for angle in (
                view.pan_body_deg,
                view.tilt_euler_deg,
                view.roll_euler_deg,
                view.settle_s,
            ):
                if (
                    isinstance(angle, bool)
                    or not isinstance(angle, (int, float))
                    or not math.isfinite(angle)
                ):
                    raise ValueError("inspection view values must be finite")
            if view.settle_s < 0.0:
                raise ValueError("inspection view settle time must not be negative")
    return plan


class MissionCoordinator:
    """Pure checkpoint state machine; hardware and navigation stay behind ports."""

    def __init__(self, plan: MissionPlan) -> None:
        self.plan = validate_mission_plan(plan)
        self._status = MissionStatus(
            mission_id=plan.mission_id,
            map_version=plan.map_version,
            route_id=plan.route_id,
        )
        self._interrupted_from: MissionState | None = None

    def status(self) -> MissionStatus:
        return replace(self._status)

    def start(self) -> MissionTransition:
        self._require_state(MissionState.IDLE)
        return self._change(MissionState.TRAVELING, "MISSION_STARTED", "navigate")

    def observe_route_progress(self, route_progress_index: int) -> MissionTransition:
        self._require_state(MissionState.TRAVELING)
        if (
            isinstance(route_progress_index, bool)
            or not isinstance(route_progress_index, int)
            or route_progress_index < 0
        ):
            raise ValueError("route progress index must be a non-negative integer")
        self._status.route_progress_index = max(
            self._status.route_progress_index, int(route_progress_index)
        )
        checkpoint = self._active_checkpoint()
        if checkpoint is None or self._status.route_progress_index < checkpoint.route_progress_index:
            return self._transition("none")
        self._status.active_checkpoint_id = checkpoint.checkpoint_id
        self._status.pause_request_id = (
            f"{self.plan.mission_id}:{checkpoint.checkpoint_id}:"
            f"{self._status.checkpoint_index}"
        )
        self._status.resume_route_progress_index = self._status.route_progress_index
        return self._change(
            MissionState.PAUSING,
            "CHECKPOINT_REACHED_PAUSE_REQUIRED",
            "pause_navigation",
            checkpoint,
        )

    def confirm_stopped(self, receipt: NavigationStopReceipt) -> MissionTransition:
        self._require_state(MissionState.PAUSING)
        checkpoint = self._active_checkpoint()
        if (
            receipt.mission_id != self.plan.mission_id
            or checkpoint is None
            or receipt.checkpoint_id != checkpoint.checkpoint_id
            or receipt.pause_request_id != self._status.pause_request_id
        ):
            raise ValueError("stop receipt does not match the active pause request")
        if receipt.map_version != self.plan.map_version or receipt.route_id != self.plan.route_id:
            raise ValueError("stop receipt does not match mission map and route")
        if receipt.route_progress_index < checkpoint.route_progress_index:
            raise ValueError("stop receipt precedes the active checkpoint")
        if receipt.stopped is not True or receipt.motion_authorized is not False:
            raise ValueError("stop receipt does not prove motion is disabled")
        values = (
            receipt.linear_speed_mps,
            receipt.angular_speed_rps,
            receipt.stable_for_s,
        )
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in values
        ):
            raise ValueError("stop receipt contains an invalid motion measurement")
        if (
            abs(receipt.linear_speed_mps) > 0.02
            or abs(receipt.angular_speed_rps) > 0.03
            or receipt.stable_for_s < 0.5
        ):
            raise ValueError("stop receipt does not prove the robot is stable")
        self._status.route_progress_index = max(
            self._status.route_progress_index, receipt.route_progress_index
        )
        self._status.resume_route_progress_index = self._status.route_progress_index
        return self._change(MissionState.PAUSED, "CHECKPOINT_STOP_CONFIRMED", "stopped", checkpoint)

    def begin_inspection(self) -> MissionTransition:
        self._require_state(MissionState.PAUSED)
        return self._change(
            MissionState.INSPECTING,
            "CHECKPOINT_INSPECTION_STARTED",
            "inspect",
            self._active_checkpoint(),
        )

    def mark_inspection_complete(self) -> MissionTransition:
        self._require_state(MissionState.INSPECTING)
        return self._change(
            MissionState.WAITING_CONTINUE,
            "CHECKPOINT_EVIDENCE_READY",
            "wait_platform_continue",
            self._active_checkpoint(),
        )

    def request_continue(self) -> MissionTransition:
        self._require_state(MissionState.WAITING_CONTINUE)
        return self._change(
            MissionState.RESUMING,
            "PLATFORM_CONTINUE_REQUESTED",
            "resume_navigation",
            self._active_checkpoint(),
        )

    def request_offline_continue(self, *, evidence_durable: bool) -> MissionTransition:
        """Use only when the downloaded mission explicitly preauthorizes offline progress."""

        self._require_state(MissionState.WAITING_CONTINUE)
        if not self.plan.offline_continue_after_evidence:
            raise RuntimeError("mission does not authorize offline continuation")
        if evidence_durable is not True:
            raise RuntimeError("offline continuation requires durably buffered evidence")
        return self._change(
            MissionState.RESUMING,
            "OFFLINE_EVIDENCE_BUFFERED_CONTINUE",
            "resume_navigation",
            self._active_checkpoint(),
        )

    def confirm_resumed(self, route_progress_index: int) -> MissionTransition:
        self._require_state(MissionState.RESUMING)
        if isinstance(route_progress_index, bool) or not isinstance(route_progress_index, int):
            raise ValueError("route progress index must be a non-negative integer")
        if route_progress_index < self._status.resume_route_progress_index:
            raise ValueError("navigation resumed behind the retained route suffix")
        self._status.route_progress_index = int(route_progress_index)
        self._status.checkpoint_index += 1
        self._status.active_checkpoint_id = None
        self._status.pause_request_id = None
        return self._change(MissionState.TRAVELING, "ROUTE_SUFFIX_RESUMED", "navigate")

    def route_completed(self) -> MissionTransition:
        self._require_state(MissionState.TRAVELING)
        if self._active_checkpoint() is not None:
            raise ValueError("route completed before every checkpoint was inspected")
        return self._change(MissionState.COMPLETED, "MISSION_COMPLETE", "complete")

    def interrupt(self, reason: str, *, resumable: bool) -> MissionTransition:
        if self._status.state in {MissionState.IDLE, MissionState.COMPLETED, MissionState.FAILED}:
            raise RuntimeError("inactive or terminal mission cannot be interrupted")
        self._interrupted_from = self._status.state
        self._status.resumable = bool(resumable)
        state = MissionState.INTERRUPTED if resumable else MissionState.FAILED
        return self._change(state, reason.strip() or "MISSION_INTERRUPTED", "interrupt")

    def resume_interrupted(self) -> MissionTransition:
        self._require_state(MissionState.INTERRUPTED)
        if not self._status.resumable:
            raise RuntimeError("mission interruption is not resumable")
        self._status.resumable = False
        previous = self._interrupted_from
        self._interrupted_from = None
        if previous == MissionState.TRAVELING:
            return self._change(MissionState.TRAVELING, "MISSION_RESUMED", "navigate")
        if previous in {MissionState.PAUSING, MissionState.PAUSED, MissionState.INSPECTING}:
            return self._change(
                MissionState.PAUSING,
                "MISSION_RESUMED_RECONFIRM_STOP",
                "pause_navigation",
                self._active_checkpoint(),
            )
        if previous == MissionState.WAITING_CONTINUE:
            return self._change(
                MissionState.WAITING_CONTINUE,
                "MISSION_RESUMED_WAITING_CONTINUE",
                "wait_platform_continue",
                self._active_checkpoint(),
            )
        if previous == MissionState.RESUMING:
            return self._change(
                MissionState.RESUMING,
                "MISSION_RESUMED_ROUTE_SUFFIX",
                "resume_navigation",
                self._active_checkpoint(),
            )
        raise RuntimeError("interrupted mission has no resumable source state")

    def _active_checkpoint(self) -> CheckpointPlan | None:
        if self._status.checkpoint_index >= len(self.plan.checkpoints):
            return None
        return self.plan.checkpoints[self._status.checkpoint_index]

    def _require_state(self, expected: MissionState) -> None:
        if self._status.state != expected:
            raise RuntimeError(
                f"mission state must be {expected.value}, got {self._status.state.value}"
            )

    def _transition(
        self, action: str, checkpoint: CheckpointPlan | None = None
    ) -> MissionTransition:
        self._status.updated_at = utc_now()
        return MissionTransition(action, self.status(), checkpoint)

    def _change(
        self,
        state: MissionState,
        reason: str,
        action: str,
        checkpoint: CheckpointPlan | None = None,
    ) -> MissionTransition:
        self._status.state = state
        self._status.reason = reason
        return self._transition(action, checkpoint)
