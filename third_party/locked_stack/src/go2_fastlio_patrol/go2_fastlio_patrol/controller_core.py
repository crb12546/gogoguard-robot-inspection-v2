#!/usr/bin/env python3
"""
Dependency-free waypoint follower mathematics.

This module deliberately owns no ROS objects, clocks, files, or loggers.  The
ROS node keeps lifecycle and state-transition responsibilities while calling
these functions for deterministic geometry and command calculation.
"""

import math


def normalize_angle(angle):
    """Normalize an angle with the deployed follower's exact boundary rules."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def nearest_global(route, current_x, current_y):
    """Return the index of the closest route point."""
    best_index = 0
    best_distance = 1e9
    for index, point in enumerate(route):
        distance = math.hypot(
            point["x"] - current_x,
            point["y"] - current_y,
        )
        if distance < best_distance:
            best_distance = distance
            best_index = index
    return best_index


def nearest_window(
    route,
    current_x,
    current_y,
    nearest_index,
    search_window,
):
    """Return the closest point inside the deployed local search window."""
    count = len(route)
    start = max(0, nearest_index - search_window)
    end = min(count - 1, nearest_index + search_window)
    best_index = nearest_index
    best_distance = 1e9
    for index in range(start, end + 1):
        point = route[index]
        distance = math.hypot(
            point["x"] - current_x,
            point["y"] - current_y,
        )
        if distance < best_distance:
            best_distance = distance
            best_index = index
    return best_index, best_distance


def lookahead_index(route, nearest_index, direction, lookahead_distance):
    """Walk route arc length until the configured lookahead is reached."""
    count = len(route)
    index = nearest_index
    accumulated = 0.0
    while True:
        next_index = index + direction
        if next_index < 0 or next_index >= count:
            return index
        start = route[index]
        end = route[next_index]
        accumulated += math.hypot(
            end["x"] - start["x"],
            end["y"] - start["y"],
        )
        index = next_index
        if accumulated >= lookahead_distance:
            return index


def target_command(
    target,
    current_x,
    current_y,
    current_yaw,
    v_base,
    max_vx,
    k_yaw,
    max_yaw_rate,
    turn_in_place_angle,
    slow_down_angle,
):
    """Calculate the exact deployed nearest/lookahead drive command."""
    delta_x = target["x"] - current_x
    delta_y = target["y"] - current_y
    distance = math.hypot(delta_x, delta_y)
    target_angle = math.atan2(delta_y, delta_x)
    alpha = normalize_angle(target_angle - current_yaw)
    yaw_rate = k_yaw * alpha
    yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))
    if abs(alpha) > turn_in_place_angle:
        velocity_x = 0.0
    elif abs(alpha) > slow_down_angle:
        velocity_x = min(v_base * 0.5, max_vx)
    else:
        velocity_x = min(v_base, max_vx)
    return {
        "distance": distance,
        "target_angle": target_angle,
        "alpha": alpha,
        "vx": velocity_x,
        "yaw_rate": yaw_rate,
    }
