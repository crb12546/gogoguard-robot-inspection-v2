"""Pure timing contract for one ``startPatrol`` attempt.

The runtime node owns the mutable object, while offline commissioning tools
consume the JSON snapshot.  Monotonic timestamps are used for durations so an
NTP correction cannot make a fast-start result look faster or slower.  Epoch
timestamps are retained only to correlate the robot trace with the SaaS trace.
"""

from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass
from typing import Dict, Optional


def _now(epoch: Optional[float], monotonic: Optional[float]):
    wall = time.time() if epoch is None else float(epoch)
    steady = time.monotonic() if monotonic is None else float(monotonic)
    if not math.isfinite(wall) or not math.isfinite(steady):
        raise ValueError("start timing timestamps must be finite")
    return wall, steady


def _duration_ms(start: Optional[float], end: Optional[float]):
    if start is None or end is None:
        return None
    return round(max(0.0, float(end) - float(start)) * 1000.0, 1)


@dataclass
class StartAttemptTimeline:
    """Mutable state for one service request and its resulting Nav2 goal."""

    attempt_id: str
    requested_map_version: str
    requested_route_id: str
    received_at_epoch: float
    received_at_monotonic: float
    stage: str = "SERVICE_RECEIVED"
    outcome: str = "PENDING"
    reason: str = ""
    service_responded_at_epoch: Optional[float] = None
    service_responded_at_monotonic: Optional[float] = None
    goal_requested_at_epoch: Optional[float] = None
    goal_requested_at_monotonic: Optional[float] = None
    goal_decided_at_epoch: Optional[float] = None
    goal_decided_at_monotonic: Optional[float] = None
    nav2_goal_accepted: Optional[bool] = None
    motion_authorized_at_epoch: Optional[float] = None
    motion_authorized_at_monotonic: Optional[float] = None
    first_final_nonzero_at_epoch: Optional[float] = None
    first_final_nonzero_at_monotonic: Optional[float] = None
    finished_at_epoch: Optional[float] = None
    finished_at_monotonic: Optional[float] = None

    @classmethod
    def create(
        cls,
        *,
        requested_map_version: str = "",
        requested_route_id: str = "",
        attempt_id: str = "",
        epoch: Optional[float] = None,
        monotonic: Optional[float] = None,
    ) -> "StartAttemptTimeline":
        wall, steady = _now(epoch, monotonic)
        return cls(
            attempt_id=str(attempt_id).strip() or uuid.uuid4().hex,
            requested_map_version=str(requested_map_version or ""),
            requested_route_id=str(requested_route_id or ""),
            received_at_epoch=wall,
            received_at_monotonic=steady,
        )

    def mark_goal_requested(
        self,
        *,
        epoch: Optional[float] = None,
        monotonic: Optional[float] = None,
    ) -> None:
        if self.goal_requested_at_monotonic is not None:
            return
        wall, steady = _now(epoch, monotonic)
        self.goal_requested_at_epoch = wall
        self.goal_requested_at_monotonic = steady
        self.stage = "GOAL_REQUESTED"
        self.outcome = "PENDING"

    def mark_service_response(
        self,
        *,
        success: bool,
        reason: str,
        epoch: Optional[float] = None,
        monotonic: Optional[float] = None,
    ) -> None:
        if self.service_responded_at_monotonic is None:
            wall, steady = _now(epoch, monotonic)
            self.service_responded_at_epoch = wall
            self.service_responded_at_monotonic = steady
        self.reason = str(reason or "")
        if not success:
            if self.goal_requested_at_monotonic is None:
                self.stage = "REJECTED"
                self.outcome = "REJECTED"
            else:
                self.stage = "FAILED"
                self.outcome = "FAILED"
            self.finished_at_epoch = self.service_responded_at_epoch
            self.finished_at_monotonic = self.service_responded_at_monotonic

    def mark_goal_decision(
        self,
        accepted: bool,
        *,
        reason: str = "",
        epoch: Optional[float] = None,
        monotonic: Optional[float] = None,
    ) -> None:
        if self.goal_decided_at_monotonic is not None:
            return
        wall, steady = _now(epoch, monotonic)
        self.goal_decided_at_epoch = wall
        self.goal_decided_at_monotonic = steady
        self.nav2_goal_accepted = bool(accepted)
        self.reason = str(reason or ("OK" if accepted else "NAV2_GOAL_REJECTED"))
        if accepted:
            self.stage = "GOAL_ACCEPTED"
            self.outcome = "RUNNING"
        else:
            self.stage = "GOAL_REJECTED"
            self.outcome = "FAILED"
            self.finished_at_epoch = wall
            self.finished_at_monotonic = steady

    def mark_motion_authorized(
        self,
        *,
        epoch: Optional[float] = None,
        monotonic: Optional[float] = None,
    ) -> None:
        if self.motion_authorized_at_monotonic is not None:
            return
        if self.nav2_goal_accepted is not True:
            raise RuntimeError("motion cannot be authorized before Nav2 accepts the goal")
        wall, steady = _now(epoch, monotonic)
        self.motion_authorized_at_epoch = wall
        self.motion_authorized_at_monotonic = steady
        self.stage = "MOTION_AUTHORIZED"
        self.outcome = "RUNNING"

    def mark_first_final_nonzero(
        self,
        *,
        epoch: Optional[float] = None,
        monotonic: Optional[float] = None,
    ) -> None:
        if self.first_final_nonzero_at_monotonic is not None:
            return
        if self.motion_authorized_at_monotonic is None:
            raise RuntimeError("final nonzero command cannot precede authorization")
        wall, steady = _now(epoch, monotonic)
        self.first_final_nonzero_at_epoch = wall
        self.first_final_nonzero_at_monotonic = steady
        self.stage = "FIRST_FINAL_NONZERO"
        self.outcome = "RUNNING"

    def finish(
        self,
        *,
        outcome: str,
        reason: str,
        epoch: Optional[float] = None,
        monotonic: Optional[float] = None,
    ) -> None:
        if self.finished_at_monotonic is None:
            wall, steady = _now(epoch, monotonic)
            self.finished_at_epoch = wall
            self.finished_at_monotonic = steady
        self.outcome = str(outcome)
        self.reason = str(reason or "")
        self.stage = "FINISHED"

    def snapshot(self) -> Dict[str, object]:
        """Return the stable audit payload; never publish monotonic values."""
        durations = {
            "serviceResponseMs": _duration_ms(
                self.received_at_monotonic,
                self.service_responded_at_monotonic,
            ),
            "receiveToGoalRequestMs": _duration_ms(
                self.received_at_monotonic,
                self.goal_requested_at_monotonic,
            ),
            "goalDecisionMs": _duration_ms(
                self.goal_requested_at_monotonic,
                self.goal_decided_at_monotonic,
            ),
            "serviceResponseToGoalDecisionMs": _duration_ms(
                self.service_responded_at_monotonic,
                self.goal_decided_at_monotonic,
            ),
            "receiveToGoalAcceptedMs": (
                _duration_ms(
                    self.received_at_monotonic,
                    self.goal_decided_at_monotonic,
                )
                if self.nav2_goal_accepted is True
                else None
            ),
            "receiveToMotionAuthorizedMs": _duration_ms(
                self.received_at_monotonic,
                self.motion_authorized_at_monotonic,
            ),
            "authorizationToFirstFinalNonzeroMs": _duration_ms(
                self.motion_authorized_at_monotonic,
                self.first_final_nonzero_at_monotonic,
            ),
            "receiveToFirstFinalNonzeroMs": _duration_ms(
                self.received_at_monotonic,
                self.first_final_nonzero_at_monotonic,
            ),
        }
        return {
            "schema": "go2.start_timing.v1",
            "attemptId": self.attempt_id,
            "requestedMapVersion": self.requested_map_version,
            "requestedRouteId": self.requested_route_id,
            "stage": self.stage,
            "outcome": self.outcome,
            "reason": self.reason,
            "receivedAtEpoch": self.received_at_epoch,
            "serviceRespondedAtEpoch": self.service_responded_at_epoch,
            "goalRequestedAtEpoch": self.goal_requested_at_epoch,
            "goalDecidedAtEpoch": self.goal_decided_at_epoch,
            "nav2GoalAccepted": self.nav2_goal_accepted,
            "motionAuthorizedAtEpoch": self.motion_authorized_at_epoch,
            "firstFinalNonzeroAtEpoch": self.first_final_nonzero_at_epoch,
            "finishedAtEpoch": self.finished_at_epoch,
            "durationsMs": durations,
        }
