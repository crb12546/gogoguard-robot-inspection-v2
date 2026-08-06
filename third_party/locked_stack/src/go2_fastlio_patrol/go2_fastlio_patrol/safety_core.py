#!/usr/bin/env python3
"""Dependency-free command limits and stream-freshness safety helpers."""

import math


def pointcloud_xyz_layout(
    fields,
    point_step,
    row_step,
    width,
    height,
    data_length,
    float32_datatype=7,
):
    """
    Validate a PointCloud2 XYZ memory layout without importing ROS.

    ``fields`` is a mapping from field name to ``(offset, datatype)``.  The
    returned offsets are safe to use with ``struct.unpack_from`` for every
    declared point, including organized clouds with row padding.
    """
    try:
        point_step = int(point_step)
        row_step = int(row_step)
        width = int(width)
        height = int(height)
        data_length = int(data_length)
        float32_datatype = int(float32_datatype)
    except (TypeError, ValueError, OverflowError):
        return None, "cloud_layout_not_integer"
    if point_step <= 0:
        return None, "cloud_point_step_invalid"
    if width <= 0 or height <= 0:
        return None, "cloud_empty"
    minimum_row_step = width * point_step
    if row_step < minimum_row_step:
        return None, "cloud_row_step_invalid"
    required_length = (height - 1) * row_step + minimum_row_step
    if data_length < required_length:
        return None, "cloud_data_truncated"
    if not isinstance(fields, dict):
        return None, "cloud_fields_invalid"

    offsets = {}
    for name in ("x", "y", "z"):
        spec = fields.get(name)
        if not isinstance(spec, (tuple, list)) or len(spec) != 2:
            return None, "cloud_xyz_missing"
        try:
            offset = int(spec[0])
            datatype = int(spec[1])
        except (TypeError, ValueError, OverflowError):
            return None, "cloud_xyz_field_invalid"
        if datatype != float32_datatype:
            return None, "cloud_xyz_not_float32"
        if offset < 0 or offset + 4 > point_step:
            return None, "cloud_xyz_offset_invalid"
        offsets[name] = offset
    return offsets, "cloud_layout_valid"


def localization_gate(status, expected_map_version, receive_age, timeout):
    """Validate the version-bound continuous-localization motion gate."""
    expected = str(expected_map_version).strip()
    if not expected:
        return False, "expected_map_version_missing"
    age = float(receive_age)
    if not math.isfinite(age) or age > max(0.0, float(timeout)):
        return False, "localization_status_timeout"
    if not isinstance(status, dict) or status.get("schemaVersion") != 1:
        return False, "localization_status_schema"
    if status.get("mapVersion") != expected:
        return False, "localization_map_version_mismatch"
    if status.get("state") != "TRACKING":
        return False, "localization_not_tracking"
    if status.get("usable") is not True:
        return False, "localization_not_usable"
    return True, "localization_ready"


def runtime_authorization_gate(authorized, receive_age, timeout):
    """Require a fresh explicit authorization from the patrol state machine."""
    age = float(receive_age)
    if not math.isfinite(age) or age > max(0.0, float(timeout)):
        return False, "runtime_authorization_timeout"
    if authorized is not True:
        return False, "runtime_motion_not_authorized"
    return True, "runtime_motion_authorized"


def stream_receive_age(now, last_receive_time):
    """Measure stream freshness from local receipt, not message stamps."""
    receive_time = float(last_receive_time)
    if not math.isfinite(receive_time) or receive_time <= 0.0:
        return float("inf")
    return max(0.0, float(now) - receive_time)


def safety_status_payload(
    timestamp,
    expected_map_version,
    localization_status,
    localization_ready,
    authorization_ready,
    command_age,
    command_timeout,
    cloud_age,
    cloud_timeout,
    obstacle_stop,
    obstacle_distance,
    stop_reason,
    cloud_validity_reason="cloud_unknown",
    cloud_valid_point_count=0,
    localization_age=float("inf"),
    localization_timeout=0.0,
    authorization_age=float("inf"),
    authorization_timeout=0.0,
    obstacle_stop_point_count=0,
    obstacle_roi_point_count=0,
    lateral_stop_point_count=0,
    lateral_obstacle_distance=float("inf"),
    left_corridor_point_count=0,
    right_corridor_point_count=0,
    left_corridor_nearest_x=float("inf"),
    right_corridor_nearest_x=float("inf"),
):
    """
    Build a read-only account of the inputs that authorized final motion.

    This status never feeds the command gate.  It exists so validation can
    prove which real gate stopped the robot instead of inferring from logs.
    """
    expected = str(expected_map_version).strip()
    exact_map = bool(
        expected
        and isinstance(localization_status, dict)
        and localization_status.get('mapVersion') == expected
    )
    distance = float(obstacle_distance)
    lateral_distance = float(lateral_obstacle_distance)
    left_distance = float(left_corridor_nearest_x)
    right_distance = float(right_corridor_nearest_x)

    def finite_age(value):
        numeric = float(value)
        return numeric if math.isfinite(numeric) else None

    return {
        'schema': 'go2.safety_status.v1',
        'timestamp': float(timestamp),
        'expectedMapVersion': expected,
        'localizationUsable': localization_ready is True,
        'exactMap': exact_map,
        'authorized': authorization_ready is True,
        'commandFresh': math.isfinite(float(command_age))
        and float(command_age) <= max(0.0, float(command_timeout)),
        'commandAgeS': finite_age(command_age),
        'commandTimeoutS': max(0.0, float(command_timeout)),
        'cloudFresh': math.isfinite(float(cloud_age))
        and float(cloud_age) <= max(0.0, float(cloud_timeout)),
        'cloudAgeS': finite_age(cloud_age),
        'cloudTimeoutS': max(0.0, float(cloud_timeout)),
        'localizationAgeS': finite_age(localization_age),
        'localizationTimeoutS': max(0.0, float(localization_timeout)),
        'authorizationAgeS': finite_age(authorization_age),
        'authorizationTimeoutS': max(0.0, float(authorization_timeout)),
        'cloudValidityReason': str(cloud_validity_reason),
        'cloudValidPointCount': max(0, int(cloud_valid_point_count)),
        'obstacleDetected': obstacle_stop is True,
        'obstacleDistanceM': distance if math.isfinite(distance) else None,
        'obstacleStopPointCount': max(0, int(obstacle_stop_point_count)),
        'obstacleRoiPointCount': max(0, int(obstacle_roi_point_count)),
        'lateralStopPointCount': max(0, int(lateral_stop_point_count)),
        'lateralObstacleDistanceM': (
            lateral_distance if math.isfinite(lateral_distance) else None
        ),
        'leftCorridorPointCount': max(0, int(left_corridor_point_count)),
        'rightCorridorPointCount': max(0, int(right_corridor_point_count)),
        'leftCorridorNearestXM': (
            left_distance if math.isfinite(left_distance) else None
        ),
        'rightCorridorNearestXM': (
            right_distance if math.isfinite(right_distance) else None
        ),
        'stopReason': str(stop_reason),
    }


def limit_planar_command(
    vx,
    vy,
    yaw_rate,
    max_vx,
    max_vy,
    max_yaw_rate,
    enabled=True,
):
    """Clamp a command, or return a complete zero command for a safety stop."""
    if not enabled:
        return 0.0, 0.0, 0.0

    try:
        values = tuple(
            float(value)
            for value in (vx, vy, yaw_rate, max_vx, max_vy, max_yaw_rate)
        )
    except (TypeError, ValueError):
        return 0.0, 0.0, 0.0
    if not all(math.isfinite(value) for value in values):
        return 0.0, 0.0, 0.0
    vx, vy, yaw_rate, max_vx, max_vy, max_yaw_rate = values
    if max_vx < 0.0 or max_vy < 0.0 or max_yaw_rate < 0.0:
        return 0.0, 0.0, 0.0

    def clamp(value, limit):
        return max(-limit, min(limit, value))

    return (
        clamp(vx, max_vx),
        clamp(vy, max_vy),
        clamp(yaw_rate, max_yaw_rate),
    )


def point_in_lateral_motion_roi(
    x,
    y,
    z,
    vy,
    cmd_deadband=0.02,
    x_min=0.10,
    x_max=1.00,
    inner_y=0.30,
    outer_y=0.65,
    z_min=0.25,
    z_max=0.90,
):
    """Return whether a point blocks the side selected by body-frame ``vy``."""
    vy = float(vy)
    if abs(vy) <= max(0.0, float(cmd_deadband)):
        return False
    if not (
        float(x_min) <= float(x) <= float(x_max)
        and float(z_min) <= float(z) <= float(z_max)
    ):
        return False

    inner_y = abs(float(inner_y))
    outer_y = max(inner_y, abs(float(outer_y)))
    if vy > 0.0:
        return inner_y <= float(y) <= outer_y
    return -outer_y <= float(y) <= -inner_y
