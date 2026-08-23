"""Explicit GLIM sensor-pose to robot-body-pose conversion at the route boundary."""

from __future__ import annotations

import math
from typing import Any, Iterable, Sequence

from gogoguard_calibration import MountCalibration, RigidTransform


BASE_POSE_FRAME = "base_link"
LIDAR_POSE_FRAME = "lidar_link"
POSE_FRAMES = {BASE_POSE_FRAME, LIDAR_POSE_FRAME}


def artifact_pose_frame(value: dict[str, Any], *, legacy: str) -> str:
    frame = value.get("trajectoryPoseFrame", value.get("poseFrame", legacy))
    if frame not in POSE_FRAMES:
        raise ValueError("trajectory pose frame must be base_link or lidar_link")
    return str(frame)


def _planar_yaws(points: Sequence[tuple[float, float]]) -> list[float]:
    if len(points) < 2:
        raise ValueError("trajectory needs at least two points")
    yaws: list[float] = []
    for index, point in enumerate(points):
        neighbor = None
        for candidate in points[index + 1 :]:
            if math.hypot(candidate[0] - point[0], candidate[1] - point[1]) >= 0.02:
                neighbor = candidate
                break
        if neighbor is not None:
            yaws.append(math.atan2(neighbor[1] - point[1], neighbor[0] - point[0]))
            continue
        for candidate in reversed(points[:index]):
            if math.hypot(point[0] - candidate[0], point[1] - candidate[1]) >= 0.02:
                yaws.append(math.atan2(point[1] - candidate[1], point[0] - candidate[0]))
                break
        else:
            yaws.append(yaws[-1] if yaws else 0.0)
    return yaws


def planar_trajectory_to_base(
    trajectory: Iterable[Sequence[Any]],
    *,
    pose_frame: str,
    calibration: MountCalibration,
) -> list[list[float]]:
    """Return map-frame XYZ positions of ``base_link`` for a planar route."""

    values = [list(item) for item in trajectory]
    if len(values) < 2 or any(len(item) < 3 for item in values):
        raise ValueError("trajectory needs at least two XYZ poses")
    if pose_frame == BASE_POSE_FRAME:
        return [[float(item[0]), float(item[1]), float(item[2])] for item in values]
    if pose_frame != LIDAR_POSE_FRAME:
        raise ValueError("unsupported trajectory pose frame")
    tx, ty, _tz = calibration.transform.translation_xyz_m
    xy = [(float(item[0]), float(item[1])) for item in values]
    result: list[list[float]] = []
    for item, yaw in zip(values, _planar_yaws(xy)):
        # T_map_base = T_map_lidar * inverse(T_base_lidar).  The compact map
        # trajectory omits orientation, so its planar tangent supplies yaw.
        result.append(
            [
                float(item[0]) - math.cos(yaw) * tx + math.sin(yaw) * ty,
                float(item[1]) - math.sin(yaw) * tx - math.cos(yaw) * ty,
                float(item[2]),
            ]
        )
    return result


def optimized_pose_to_base(
    pose: dict[str, Any],
    *,
    pose_frame: str,
    calibration: MountCalibration,
) -> RigidTransform:
    """Return ``T_map_base`` for a quaternion-bearing optimized GLIM pose."""

    transform = RigidTransform(
        parent_frame="map",
        child_frame=pose_frame,
        translation_xyz_m=(float(pose["x"]), float(pose["y"]), float(pose["z"])),
        rotation_xyzw=(
            float(pose["qx"]),
            float(pose["qy"]),
            float(pose["qz"]),
            float(pose["qw"]),
        ),
    )
    if pose_frame == BASE_POSE_FRAME:
        return transform
    if pose_frame != LIDAR_POSE_FRAME:
        raise ValueError("unsupported optimized trajectory pose frame")
    return transform.compose(calibration.transform.inverse())
