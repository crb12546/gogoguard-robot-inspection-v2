from __future__ import annotations

import math
from dataclasses import dataclass, replace


def wrap_radians(value: float) -> float:
    """Return the shortest signed ROS yaw angle in [-pi, pi]."""

    return math.atan2(math.sin(float(value)), math.cos(float(value)))


@dataclass(frozen=True)
class CheckpointViewAlignment:
    """One body/camera allocation for a recorded inspection view.

    Z1Pro positive pan is camera-right while ROS positive yaw is counter-
    clockwise, so the optical yaw is ``body_yaw - camera_pan``.
    """

    desired_view_yaw_rad: float
    target_body_yaw_rad: float
    body_turn_rad: float
    camera_pan_deg: float
    mode: str
    pan_limit_deg: float

    def with_mode(self, mode: str) -> "CheckpointViewAlignment":
        return replace(self, mode=str(mode))

    def as_status(self) -> dict[str, float | str]:
        return {
            "desiredViewYawRad": self.desired_view_yaw_rad,
            "targetBodyYawRad": self.target_body_yaw_rad,
            "bodyTurnRad": self.body_turn_rad,
            "cameraPanDeg": self.camera_pan_deg,
            "mode": self.mode,
            "panLimitDeg": self.pan_limit_deg,
        }


def plan_checkpoint_view_alignment(
    *,
    recorded_body_yaw_rad: float,
    recorded_camera_pan_deg: float,
    current_body_yaw_rad: float,
    pan_limit_deg: float,
    body_tolerance_rad: float = 0.03,
) -> CheckpointViewAlignment:
    """Preserve the recorded optical direction with minimum body rotation.

    The camera consumes as much of the yaw difference as the selected pan
    range allows.  The body receives only the remainder.
    """

    values = (
        recorded_body_yaw_rad,
        recorded_camera_pan_deg,
        current_body_yaw_rad,
        pan_limit_deg,
        body_tolerance_rad,
    )
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in values):
        raise ValueError("checkpoint alignment inputs must be numeric")
    if any(not math.isfinite(float(value)) for value in values):
        raise ValueError("checkpoint alignment inputs must be finite")
    limit = float(pan_limit_deg)
    tolerance = float(body_tolerance_rad)
    if not 0.0 < limit < 180.0:
        raise ValueError("checkpoint camera pan limit must be between 0 and 180 degrees")
    if not 0.0 <= tolerance <= 0.25:
        raise ValueError("checkpoint body tolerance is invalid")

    desired_view = wrap_radians(
        float(recorded_body_yaw_rad) - math.radians(float(recorded_camera_pan_deg))
    )
    camera_only_pan = math.degrees(
        wrap_radians(float(current_body_yaw_rad) - desired_view)
    )
    if abs(camera_only_pan) <= limit:
        target_body = wrap_radians(float(current_body_yaw_rad))
        body_turn = 0.0
        camera_pan = camera_only_pan
        mode = "camera_only"
    else:
        camera_pan = math.copysign(limit, camera_only_pan)
        target_body = wrap_radians(desired_view + math.radians(camera_pan))
        body_turn = wrap_radians(target_body - float(current_body_yaw_rad))
        mode = "body_plus_camera"
        if abs(body_turn) <= tolerance:
            body_turn = 0.0
            target_body = wrap_radians(float(current_body_yaw_rad))

    return CheckpointViewAlignment(
        desired_view_yaw_rad=desired_view,
        target_body_yaw_rad=target_body,
        body_turn_rad=body_turn,
        camera_pan_deg=camera_pan,
        mode=mode,
        pan_limit_deg=limit,
    )
