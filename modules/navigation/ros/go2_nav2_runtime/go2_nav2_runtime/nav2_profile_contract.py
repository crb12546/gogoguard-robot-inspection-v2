"""Fail-closed checks for the ROS 2 Humble Go2 local-navigation profile."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence, Tuple

try:
    import yaml
except ImportError:  # Development hosts may not have the robot's apt set.
    yaml = None


class Nav2ProfileError(ValueError):
    pass


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise Nav2ProfileError("%s must be a mapping" % label)
    return value


def _finite(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise Nav2ProfileError("%s must be numeric" % label) from exc
    if not math.isfinite(result):
        raise Nav2ProfileError("%s must be finite" % label)
    return result


def _positive(value: Any, label: str) -> float:
    result = _finite(value, label)
    if result <= 0.0:
        raise Nav2ProfileError("%s must be positive" % label)
    return result


def _positive_int(value: Any, label: str) -> int:
    result = _positive(value, label)
    integer = int(result)
    if result != integer:
        raise Nav2ProfileError("%s must be an integer" % label)
    return integer


def _points(value: Any, label: str) -> Tuple[Tuple[float, float], ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise Nav2ProfileError("%s must be a flat coordinate list" % label)
    if len(value) < 6 or len(value) % 2:
        raise Nav2ProfileError("%s must contain at least three x/y pairs" % label)
    numbers = tuple(_finite(item, label) for item in value)
    return tuple(zip(numbers[0::2], numbers[1::2]))


def _bounds(points: Sequence[Tuple[float, float]]) -> Tuple[float, float, float, float]:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), max(xs), min(ys), max(ys)


def _footprint(value: Any) -> Tuple[Tuple[float, float], ...]:
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except (TypeError, ValueError) as exc:
            raise Nav2ProfileError("local_costmap footprint is invalid JSON") from exc
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise Nav2ProfileError("local_costmap footprint must be a point list")
    points = []
    for item in value:
        if not isinstance(item, Sequence) or len(item) != 2:
            raise Nav2ProfileError("local_costmap footprint points must be [x, y]")
        points.append(
            (
                _finite(item[0], "footprint.x"),
                _finite(item[1], "footprint.y"),
            )
        )
    if len(points) < 3:
        raise Nav2ProfileError("local_costmap footprint needs at least three points")
    return tuple(points)


def load_nav2_profile(path: Path) -> Mapping[str, Any]:
    if yaml is None:
        raise Nav2ProfileError(
            "PyYAML is required to validate the Nav2 profile; install python3-yaml"
        )
    try:
        payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise Nav2ProfileError("cannot load Nav2 profile: %s" % exc) from exc
    return _mapping(payload, "Nav2 profile")


def validate_nav2_profile(path: Path) -> Mapping[str, float]:
    """Validate parameters that must remain coherent on Nav2 Humble.

    This is intentionally stricter than ROS parameter parsing. Unknown or
    mutually inconsistent values must stop the complete runtime generation
    before a hardware-output process is launched.
    """
    profile = load_nav2_profile(path)
    controller = _mapping(profile.get("controller_server"), "controller_server")
    controller = _mapping(controller.get("ros__parameters"), "controller parameters")
    follow = _mapping(controller.get("FollowPath"), "FollowPath")
    plugins = controller.get("controller_plugins")
    if not isinstance(plugins, Sequence) or set(plugins) != {
        "FollowPath",
        "DetourPath",
    }:
        raise Nav2ProfileError(
            "controller ownership requires FollowPath and DetourPath only"
        )
    if follow.get("plugin") != "nav2_mppi_controller::MPPIController":
        raise Nav2ProfileError("FollowPath must use the Humble MPPI controller")
    if follow.get("motion_model") != "Omni":
        raise Nav2ProfileError("FollowPath motion_model must be Omni")
    progress = _mapping(controller.get("progress_checker"), "progress_checker")
    if progress.get("plugin") != "nav2_controller::PoseProgressChecker":
        raise Nav2ProfileError(
            "progress checker must count RPP heading rotation as progress"
        )
    required_movement_angle = _positive(
        progress.get("required_movement_angle"),
        "progress_checker.required_movement_angle",
    )
    if required_movement_angle > 0.35:
        raise Nav2ProfileError("progress rotation threshold is too coarse")

    frequency = _positive(controller.get("controller_frequency"), "controller_frequency")
    model_dt = _positive(follow.get("model_dt"), "FollowPath.model_dt")
    if not math.isclose(frequency * model_dt, 1.0, rel_tol=0.0, abs_tol=1.0e-6):
        raise Nav2ProfileError(
            "Humble MPPI requires controller period to equal model_dt"
        )
    time_steps = _positive_int(follow.get("time_steps"), "FollowPath.time_steps")
    batch_size = _positive_int(follow.get("batch_size"), "FollowPath.batch_size")
    iteration_count = _positive_int(
        follow.get("iteration_count"),
        "FollowPath.iteration_count",
    )
    if time_steps > 80 or batch_size > 2000 or frequency > 30.0:
        raise Nav2ProfileError("MPPI profile exceeds the sealed low-compute budget")
    rollout_states_per_second = (
        time_steps * batch_size * iteration_count * frequency
    )
    if iteration_count != 1 or rollout_states_per_second > 1200000:
        raise Nav2ProfileError(
            "MPPI rollout work exceeds the sealed low-compute budget"
        )
    vx_max = _positive(follow.get("vx_max"), "FollowPath.vx_max")
    vx_min = _finite(follow.get("vx_min"), "FollowPath.vx_min")
    if vx_min < 0.0:
        raise Nav2ProfileError("fixed patrol routes must not authorize reverse driving")
    if vx_min > vx_max:
        raise Nav2ProfileError("FollowPath.vx_min cannot exceed vx_max")
    vy_max = _positive(follow.get("vy_max"), "FollowPath.vy_max")
    if vy_max < 0.10:
        raise Nav2ProfileError("Omni bypass needs at least 0.10 m/s lateral authority")

    critics = follow.get("critics")
    required_critics = {
        "ConstraintCritic",
        "CostCritic",
        "PathAlignCritic",
        "PathFollowCritic",
        "PathAngleCritic",
    }
    if not isinstance(critics, Sequence) or not required_critics.issubset(set(critics)):
        raise Nav2ProfileError("MPPI profile is missing collision/path critics")
    cost_critic = _mapping(follow.get("CostCritic"), "FollowPath.CostCritic")
    if cost_critic.get("enabled") is not True or cost_critic.get("consider_footprint") is not True:
        raise Nav2ProfileError("CostCritic must collision-check the real footprint")
    if cost_critic.get("inflation_layer_name") != "inflation_layer":
        raise Nav2ProfileError("CostCritic must bind the configured inflation layer")
    path_align = _mapping(follow.get("PathAlignCritic"), "FollowPath.PathAlignCritic")
    if path_align.get("enabled") is not True:
        raise Nav2ProfileError("PathAlignCritic must remain enabled for fixed routes")
    if path_align.get("use_path_orientations") is not True:
        raise Nav2ProfileError("fixed route yaw must be consumed by PathAlignCritic")
    path_angle = _mapping(follow.get("PathAngleCritic"), "FollowPath.PathAngleCritic")
    if path_angle.get("enabled") is not True:
        raise Nav2ProfileError("PathAngleCritic must remain enabled for fixed routes")
    if path_angle.get("forward_preference") is not True:
        raise Nav2ProfileError("PathAngleCritic must prefer forward route progress")
    detour = _mapping(controller.get("DetourPath"), "DetourPath")
    if (
        detour.get("plugin")
        != "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    ):
        raise Nav2ProfileError("DetourPath must use Regulated Pure Pursuit")
    detour_speed = _positive(
        detour.get("desired_linear_vel"), "DetourPath.desired_linear_vel"
    )
    if detour_speed < 0.24:
        raise Nav2ProfileError("DetourPath falls below the commissioned Go2 gait")
    if detour.get("use_rotate_to_heading") is not True:
        raise Nav2ProfileError("DetourPath must rotate before forward recovery")
    if detour.get("allow_reversing") is not False:
        raise Nav2ProfileError("DetourPath must not authorize reverse driving")

    local = _mapping(profile.get("local_costmap"), "local_costmap")
    local = _mapping(local.get("local_costmap"), "local_costmap.local_costmap")
    local = _mapping(local.get("ros__parameters"), "local costmap parameters")
    if local.get("global_frame") != "map" or local.get("robot_base_frame") != "base_link":
        raise Nav2ProfileError("local costmap frames must be map -> base_link")
    if local.get("always_send_full_costmap") is not True:
        raise Nav2ProfileError(
            "local costmap must publish deterministic full-grid health frames"
        )
    footprint = _footprint(local.get("footprint"))
    circumscribed_radius = max(math.hypot(x, y) for x, y in footprint)
    footprint_padding = _finite(local.get("footprint_padding"), "footprint_padding")
    if footprint_padding < 0.0:
        raise Nav2ProfileError("footprint_padding must be non-negative")
    padded_footprint = tuple(
        (
            x + math.copysign(footprint_padding, x) if x != 0.0 else x,
            y + math.copysign(footprint_padding, y) if y != 0.0 else y,
        )
        for x, y in footprint
    )
    effective_footprint_radius = max(
        math.hypot(x, y) for x, y in padded_footprint
    )
    inflation = _mapping(local.get("inflation_layer"), "inflation_layer")
    inflation_radius = _positive(inflation.get("inflation_radius"), "inflation_radius")
    clearance_envelope = inflation_radius - effective_footprint_radius
    if clearance_envelope + 1.0e-9 < 0.15:
        raise Nav2ProfileError(
            "inflation radius must cover the padded robot footprint plus 0.15m preference margin"
        )
    inflation_cost_scaling = _positive(
        inflation.get("cost_scaling_factor"), "cost_scaling_factor"
    )
    if inflation_cost_scaling > 4.0:
        raise Nav2ProfileError(
            "inflation cost decays too sharply for a smooth 0.35m bypass"
        )
    width = _positive(local.get("width"), "local_costmap.width")
    height = _positive(local.get("height"), "local_costmap.height")
    if min(width, height) < 4.0:
        raise Nav2ProfileError("local costmap is too small for a short bypass")
    plugins = local.get("plugins")
    if plugins != ["obstacle_layer", "inflation_layer"]:
        raise Nav2ProfileError(
            "local costmap must use the planar obstacle layer before inflation"
        )
    obstacle = _mapping(local.get("obstacle_layer"), "obstacle_layer")
    if obstacle.get("plugin") != "nav2_costmap_2d::ObstacleLayer":
        raise Nav2ProfileError("local obstacle owner must be the 2D ObstacleLayer")
    source = _mapping(obstacle.get("mid360_body"), "obstacle_layer.mid360_body")
    if source.get("topic") != "/navigation/cloud_body" or source.get("data_type") != "PointCloud2":
        raise Nav2ProfileError("local costmap must use the calibrated body cloud")
    if source.get("clearing") is not True or source.get("marking") is not True:
        raise Nav2ProfileError("body cloud must both clear and mark the rolling costmap")

    collision = _mapping(profile.get("collision_monitor"), "collision_monitor")
    collision = _mapping(collision.get("ros__parameters"), "collision monitor parameters")
    if collision.get("cmd_vel_in_topic") != "/nav2/smoothed_cmd_vel":
        raise Nav2ProfileError("collision monitor input topic breaks the motion chain")
    if collision.get("cmd_vel_out_topic") != "/patrol_cmd":
        raise Nav2ProfileError("collision monitor output topic breaks the motion chain")
    if collision.get("base_frame_id") != "base_link" or collision.get("odom_frame_id") != "odom":
        raise Nav2ProfileError("collision monitor frames must be odom -> base_link")
    collision_source = _mapping(collision.get("mid360_body"), "collision mid360_body")
    if collision_source.get("type") != "pointcloud" or collision_source.get("topic") != "/navigation/cloud_body":
        raise Nav2ProfileError("collision monitor must use the calibrated body cloud")

    stop_zone = _mapping(collision.get("StopZone"), "StopZone")
    slow_zone = _mapping(collision.get("SlowZone"), "SlowZone")
    approach_zone = _mapping(collision.get("FootprintApproach"), "FootprintApproach")
    for name, zone in (("StopZone", stop_zone), ("SlowZone", slow_zone)):
        if "min_points" in zone:
            raise Nav2ProfileError("%s uses non-Humble min_points" % name)
        max_points = zone.get("max_points")
        if not isinstance(max_points, int) or isinstance(max_points, bool) or max_points < 0:
            raise Nav2ProfileError("%s.max_points must be a non-negative integer" % name)
        if not 3 <= max_points <= 12:
            raise Nav2ProfileError(
                "%s must require between four and thirteen coherent points" % name
            )
    approach_max_points = approach_zone.get("max_points")
    if (
        not isinstance(approach_max_points, int)
        or isinstance(approach_max_points, bool)
        or not 3 <= approach_max_points <= 12
    ):
        raise Nav2ProfileError(
            "FootprintApproach must require between four and thirteen coherent points"
        )
    collision_min_height = _finite(
        collision_source.get("min_height"), "collision mid360_body.min_height"
    )
    if collision_min_height < 0.03:
        raise Nav2ProfileError(
            "collision source must reject the calibrated floor-return band"
        )
    if stop_zone.get("action_type") != "stop" or slow_zone.get("action_type") != "slowdown":
        raise Nav2ProfileError("collision zones have incorrect actions")
    stop_bounds = _bounds(_points(stop_zone.get("points"), "StopZone.points"))
    slow_bounds = _bounds(_points(slow_zone.get("points"), "SlowZone.points"))
    if not (
        slow_bounds[0] <= stop_bounds[0]
        and slow_bounds[1] >= stop_bounds[1]
        and slow_bounds[2] <= stop_bounds[2]
        and slow_bounds[3] >= stop_bounds[3]
    ):
        raise Nav2ProfileError("SlowZone must fully contain StopZone")
    if slow_bounds[1] < 1.0:
        raise Nav2ProfileError("SlowZone must begin intervention before one metre")
    slow_lateral_half_width = max(abs(slow_bounds[2]), abs(slow_bounds[3]))
    if slow_lateral_half_width > 0.55 + 1.0e-9:
        raise Nav2ProfileError(
            "SlowZone lateral envelope must not latch on passable side walls"
        )
    slowdown_ratio = _positive(
        slow_zone.get("slowdown_ratio"), "SlowZone.slowdown_ratio"
    )
    if slowdown_ratio < 0.65 or slowdown_ratio > 0.90:
        raise Nav2ProfileError(
            "SlowZone slowdown ratio must stay inside the commissioned gait envelope"
        )
    prediction_distance = time_steps * model_dt * vx_max
    if prediction_distance + 1.0e-9 < slow_bounds[1]:
        raise Nav2ProfileError("MPPI prediction horizon does not reach the slowdown zone")

    smoother = _mapping(profile.get("velocity_smoother"), "velocity_smoother")
    smoother = _mapping(smoother.get("ros__parameters"), "velocity smoother parameters")
    max_velocity = smoother.get("max_velocity")
    if not isinstance(max_velocity, Sequence) or len(max_velocity) != 3:
        raise Nav2ProfileError("velocity_smoother.max_velocity must have three axes")
    if _finite(max_velocity[0], "max_velocity.x") + 1.0e-9 < vx_max:
        raise Nav2ProfileError("velocity smoother clips MPPI forward authority")
    if _finite(max_velocity[1], "max_velocity.y") + 1.0e-9 < vy_max:
        raise Nav2ProfileError("velocity smoother clips MPPI lateral authority")
    min_velocity = smoother.get("min_velocity")
    if not isinstance(min_velocity, Sequence) or len(min_velocity) != 3:
        raise Nav2ProfileError("velocity_smoother.min_velocity must have three axes")
    if _finite(min_velocity[0], "min_velocity.x") < 0.0:
        raise Nav2ProfileError("velocity smoother must not authorize reverse driving")

    return {
        "controllerFrequencyHz": frequency,
        "modelDtS": model_dt,
        "predictionTimeS": time_steps * model_dt,
        "predictionDistanceM": prediction_distance,
        "lateralAuthorityMps": vy_max,
        "detourSpeedMps": detour_speed,
        "requiredMovementAngleRad": required_movement_angle,
        "circumscribedRadiusM": circumscribed_radius,
        "effectiveFootprintRadiusM": effective_footprint_radius,
        "inflationRadiusM": inflation_radius,
        "inflationClearanceEnvelopeM": clearance_envelope,
        "inflationCostScalingFactor": inflation_cost_scaling,
        "slowZoneFrontM": slow_bounds[1],
        "slowZoneLateralHalfWidthM": slow_lateral_half_width,
        "slowdownRatio": slowdown_ratio,
        "stopZoneFrontM": stop_bounds[1],
        "batchSize": batch_size,
        "iterationCount": iteration_count,
        "rolloutStatesPerSecond": rollout_states_per_second,
    }
