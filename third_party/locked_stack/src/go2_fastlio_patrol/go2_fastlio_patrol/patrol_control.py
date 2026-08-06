#!/usr/bin/env python3
"""
Legacy line/corner controller helpers kept for offline experiments.

The deployed nearest/lookahead follower does not use the line-following
functions in this module.  Safety helpers are re-exported for compatibility,
but their production home is :mod:`go2_fastlio_patrol.safety_core`.
"""

import bisect
import math

from go2_fastlio_patrol.safety_core import (
    limit_planar_command,
    point_in_lateral_motion_roi,
    stream_receive_age,
)

__all__ = (
    'limit_planar_command',
    'point_in_lateral_motion_roi',
    'stream_receive_age',
)


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def displacement_course_heading(
    start_x,
    start_y,
    end_x,
    end_y,
    minimum_distance=0.10,
):
    """Return a stable travel heading once displacement is measurable."""
    dx = float(end_x) - float(start_x)
    dy = float(end_y) - float(start_y)
    distance = math.hypot(dx, dy)
    if distance < max(0.0, float(minimum_distance)):
        return None
    return math.atan2(dy, dx)


def segment_metrics(x, y, start_x, start_y, end_x, end_y):
    """Describe a pose relative to one explicitly selected route segment."""
    dx = float(end_x) - float(start_x)
    dy = float(end_y) - float(start_y)
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        raise ValueError('route segment must have non-zero length')

    ux = dx / length
    uy = dy / length
    pose_dx = float(x) - float(start_x)
    pose_dy = float(y) - float(start_y)
    along = pose_dx * ux + pose_dy * uy
    lateral = ux * pose_dy - uy * pose_dx
    clamped_along = max(0.0, min(length, along))
    projected_x = float(start_x) + ux * clamped_along
    projected_y = float(start_y) + uy * clamped_along

    return {
        'length': length,
        'yaw': math.atan2(dy, dx),
        'along': along,
        'remaining': max(0.0, length - along),
        'lateral_error': lateral,
        'projected_x': projected_x,
        'projected_y': projected_y,
        'target_distance': math.hypot(
            float(x) - float(end_x),
            float(y) - float(end_y),
        ),
    }


def waypoint_reached(metrics, reach_distance):
    """Advance an ordered CSV gate once its endpoint has been passed."""
    if float(metrics['target_distance']) <= max(0.0, float(reach_distance)):
        return True
    return float(metrics['along']) >= float(metrics['length'])


def corner_turn_angle(start, corner, end):
    """Return the signed heading change at one ordered CSV vertex."""
    incoming = math.atan2(
        float(corner[1]) - float(start[1]),
        float(corner[0]) - float(start[0]),
    )
    outgoing = math.atan2(
        float(end[1]) - float(corner[1]),
        float(end[0]) - float(corner[0]),
    )
    return normalize_angle(outgoing - incoming)


def ordered_upcoming_corner(
    route,
    anchor_index,
    target_index,
    direction,
    current_segment_remaining,
    corner_angle,
    search_distance,
):
    """Return the next sharp ordered CSV vertex inside a distance horizon."""
    direction = 1 if int(direction) >= 0 else -1
    previous_index = int(anchor_index)
    corner_index = int(target_index)
    distance = max(0.0, float(current_segment_remaining))
    distance_limit = max(0.0, float(search_distance))

    def next_distinct(index):
        anchor = route[index]
        candidate = index + direction
        while 0 <= candidate < len(route):
            point = route[candidate]
            if math.hypot(
                float(point['x']) - float(anchor['x']),
                float(point['y']) - float(anchor['y']),
            ) > 1e-4:
                return candidate
            candidate += direction
        return None

    for _ in range(256):
        next_index = next_distinct(corner_index)
        if next_index is None:
            return None

        previous = route[previous_index]
        corner = route[corner_index]
        following = route[next_index]
        turn_angle = corner_turn_angle(
            (previous['x'], previous['y']),
            (corner['x'], corner['y']),
            (following['x'], following['y']),
        )
        if abs(turn_angle) >= max(0.0, float(corner_angle)):
            return {
                'index': corner_index,
                'next_index': next_index,
                'turn_angle': turn_angle,
                'distance': distance,
                'x': float(corner['x']),
                'y': float(corner['y']),
            }

        if distance > distance_limit:
            return None

        distance += math.hypot(
            float(following['x']) - float(corner['x']),
            float(following['y']) - float(corner['y']),
        )
        previous_index = corner_index
        corner_index = next_index

    raise RuntimeError('too many CSV points while finding next corner')


def point_target_heading(
    x,
    y,
    target_x,
    target_y,
    fallback_yaw,
):
    """Aim at one ordered CSV point without searching another segment."""
    dx = float(target_x) - float(x)
    dy = float(target_y) - float(y)
    if math.hypot(dx, dy) <= 1e-6:
        return float(fallback_yaw)
    return math.atan2(dy, dx)


def ordered_route_heading(
    route,
    arc_s,
    progress_s,
    direction,
    lookahead_distance,
    fallback_yaw,
):
    """Return the ordered CSV chord heading a short distance ahead."""
    if len(route) < 2 or len(arc_s) != len(route):
        raise ValueError('route geometry is inconsistent')

    total_length = float(arc_s[-1])
    if total_length <= 1e-9:
        return float(fallback_yaw)

    def interpolate(distance):
        distance = max(0.0, min(total_length, float(distance)))
        if distance <= 0.0:
            return float(route[0]['x']), float(route[0]['y'])
        if distance >= total_length:
            return float(route[-1]['x']), float(route[-1]['y'])

        index = max(
            0,
            min(
                len(route) - 2,
                bisect.bisect_right(arc_s, distance) - 1,
            ),
        )
        while (
            index < len(route) - 2
            and float(arc_s[index + 1])
            <= float(arc_s[index]) + 1e-9
        ):
            index += 1
        segment_length = (
            float(arc_s[index + 1]) - float(arc_s[index])
        )
        if segment_length <= 1e-9:
            return (
                float(route[index + 1]['x']),
                float(route[index + 1]['y']),
            )
        ratio = (distance - float(arc_s[index])) / segment_length
        return (
            float(route[index]['x'])
            + ratio
            * (float(route[index + 1]['x']) - float(route[index]['x'])),
            float(route[index]['y'])
            + ratio
            * (float(route[index + 1]['y']) - float(route[index]['y'])),
        )

    current_s = max(0.0, min(total_length, float(progress_s)))
    travel_direction = 1.0 if int(direction) >= 0 else -1.0
    target_s = max(
        0.0,
        min(
            total_length,
            current_s
            + travel_direction
            * max(0.05, float(lookahead_distance)),
        ),
    )
    current_x, current_y = interpolate(current_s)
    target_x, target_y = interpolate(target_s)
    return point_target_heading(
        current_x,
        current_y,
        target_x,
        target_y,
        fallback_yaw,
    )


def heading_drive_command(
    desired_yaw,
    current_yaw,
    forward_speed,
    heading_gain=1.20,
    yaw_deadband=0.03,
    min_yaw_rate=0.20,
    max_yaw_rate=0.50,
    max_vx=0.50,
):
    """Drive forward with the body pointing along the desired path."""
    heading_error = normalize_angle(
        float(desired_yaw) - float(current_yaw)
    )
    deadband = max(0.0, float(yaw_deadband))
    if abs(heading_error) <= deadband:
        yaw_rate = 0.0
    else:
        yaw_rate = max(0.0, float(heading_gain)) * heading_error
        minimum = max(0.0, float(min_yaw_rate))
        if abs(yaw_rate) < minimum:
            yaw_rate = math.copysign(minimum, heading_error)

    yaw_limit = max(0.0, float(max_yaw_rate))
    yaw_rate = max(-yaw_limit, min(yaw_limit, yaw_rate))
    vx = max(
        0.0,
        min(max(0.0, float(max_vx)), float(forward_speed)),
    )
    return {
        'vx': vx,
        'vy': 0.0,
        'yaw_rate': yaw_rate,
        'target_yaw': float(desired_yaw),
        'heading_error': heading_error,
    }


def predict_lateral_error(
    lateral_error,
    lateral_velocity,
    prediction_time,
):
    """Project cross-track error over one short control horizon."""
    return (
        float(lateral_error)
        + max(0.0, float(prediction_time))
        * float(lateral_velocity)
    )


def feedback_motion_scale(
    feedback_age,
    normal_age=0.60,
    slow_age=1.20,
    hold_age=2.00,
    slow_scale=0.50,
):
    """Reduce motion to one usable floor before a true feedback hold."""
    age = float(feedback_age)
    if not math.isfinite(age):
        return 0.0
    age = max(0.0, age)
    normal_limit = max(0.0, float(normal_age))
    slow_limit = max(normal_limit + 1e-6, float(slow_age))
    hold_limit = max(slow_limit + 1e-6, float(hold_age))
    middle_scale = max(0.0, min(1.0, float(slow_scale)))

    if age <= normal_limit:
        return 1.0
    if age <= slow_limit:
        ratio = (
            (age - normal_limit)
            / (slow_limit - normal_limit)
        )
        return 1.0 + (middle_scale - 1.0) * ratio
    if age < hold_limit:
        return middle_scale
    return 0.0


def effective_forward_speed(
    requested_speed,
    target_speed,
    motion_scale,
    minimum_moving_speed,
):
    """Avoid sustained forward commands below the robot's usable range."""
    requested = max(0.0, float(requested_speed))
    scale = max(0.0, min(1.0, float(motion_scale)))
    if requested <= 0.0 or scale <= 0.0:
        return 0.0
    usable_floor = min(
        max(0.0, float(target_speed)),
        max(0.0, float(minimum_moving_speed)),
    )
    return max(requested * scale, usable_floor)


def line_follow_command(
    lateral_error,
    lateral_velocity,
    route_yaw,
    current_yaw,
    forward_speed,
    motion_yaw=None,
    line_deadband=0.030,
    correction_lookahead_distance=0.28,
    correction_prediction_time=0.50,
    max_correction_angle=math.radians(12.0),
    heading_gain=1.20,
    yaw_deadband=0.03,
    min_yaw_rate=0.20,
    max_track_yaw_rate=0.50,
    max_vx=0.50,
):
    """
    Track one CSV line with continuous travel-direction feedback.

    ``vy`` is always zero: the dog turns its head and body toward the line,
    then walks forward onto it.  While measurable motion exists, ``motion_yaw``
    closes the loop on the path the dog actually travels instead of assuming
    that body yaw and travel direction are identical.
    """
    error = float(lateral_error)
    deadband = max(0.0, float(line_deadband))
    predicted_error = predict_lateral_error(
        error,
        lateral_velocity,
        correction_prediction_time,
    )
    if abs(predicted_error) <= deadband:
        correction_angle = 0.0
    else:
        lookahead = max(
            0.05,
            float(correction_lookahead_distance),
        )
        correction_angle = math.atan2(-predicted_error, lookahead)
        angle_limit = max(0.0, float(max_correction_angle))
        correction_angle = max(
            -angle_limit,
            min(angle_limit, correction_angle),
        )

    target_yaw = normalize_angle(
        float(route_yaw) + correction_angle
    )
    feedback_yaw = (
        float(current_yaw)
        if motion_yaw is None
        else float(motion_yaw)
    )
    command = heading_drive_command(
        desired_yaw=target_yaw,
        current_yaw=feedback_yaw,
        forward_speed=forward_speed,
        heading_gain=heading_gain,
        yaw_deadband=yaw_deadband,
        min_yaw_rate=min_yaw_rate,
        max_yaw_rate=max_track_yaw_rate,
        max_vx=max_vx,
    )
    command['correction_angle'] = correction_angle
    command['predicted_lateral_error'] = predicted_error
    command['heading_feedback_yaw'] = feedback_yaw
    command['heading_feedback_source'] = (
        'body' if motion_yaw is None else 'course'
    )
    command['body_heading_error'] = normalize_angle(
        target_yaw - float(current_yaw)
    )
    return command
