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


def _footprint(value: Any, label: str) -> Tuple[Tuple[float, float], ...]:
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except (TypeError, ValueError) as exc:
            raise Nav2ProfileError("%s is invalid JSON" % label) from exc
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise Nav2ProfileError("%s must be a point list" % label)
    points = []
    for item in value:
        if not isinstance(item, Sequence) or len(item) != 2:
            raise Nav2ProfileError("%s points must be [x, y]" % label)
        points.append((_finite(item[0], label), _finite(item[1], label)))
    if len(points) < 3:
        raise Nav2ProfileError("%s needs at least three points" % label)
    return tuple(points)


def _padded_bounds(
    footprint: Sequence[Tuple[float, float]], padding: float
) -> Tuple[float, float, float, float]:
    minimum_x, maximum_x, minimum_y, maximum_y = _bounds(footprint)
    return (
        minimum_x - padding,
        maximum_x + padding,
        minimum_y - padding,
        maximum_y + padding,
    )


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
    if not isinstance(plugins, Sequence) or list(plugins) != ["FollowPath"]:
        raise Nav2ProfileError("controller ownership requires one MPPI FollowPath")
    if follow.get("plugin") != "nav2_mppi_controller::MPPIController":
        raise Nav2ProfileError("FollowPath must use the Humble MPPI controller")
    if follow.get("motion_model") != "Omni":
        raise Nav2ProfileError("FollowPath motion_model must be Omni")
    progress = _mapping(controller.get("progress_checker"), "progress_checker")
    if progress.get("plugin") != "nav2_controller::PoseProgressChecker":
        raise Nav2ProfileError("progress checker must count heading rotation as progress")
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
    if cost_critic.get("enabled") is not True:
        raise Nav2ProfileError("CostCritic must remain enabled")
    if cost_critic.get("consider_footprint") is not True:
        raise Nav2ProfileError(
            "CostCritic must collision-check the shared rectangular footprint"
        )
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
    local = _mapping(profile.get("local_costmap"), "local_costmap")
    local = _mapping(local.get("local_costmap"), "local_costmap.local_costmap")
    local = _mapping(local.get("ros__parameters"), "local costmap parameters")
    if local.get("global_frame") != "map" or local.get("robot_base_frame") != "base_link":
        raise Nav2ProfileError("local costmap frames must be map -> base_link")
    if local.get("always_send_full_costmap") is not True:
        raise Nav2ProfileError(
            "local costmap must publish deterministic full-grid health frames"
        )
    if "robot_radius" in local:
        raise Nav2ProfileError(
            "local costmap must not define a circular radius beside the footprint"
        )
    footprint = _footprint(local.get("footprint"), "local_costmap.footprint")
    footprint_padding = _finite(
        local.get("footprint_padding"), "local_costmap.footprint_padding"
    )
    if not math.isclose(footprint_padding, 0.10, rel_tol=0.0, abs_tol=1.0e-9):
        raise Nav2ProfileError("planning footprint must keep 0.10m shoulder padding")
    physical_bounds = _bounds(footprint)
    expected_physical_bounds = (-0.33, 0.40, -0.20, 0.20)
    if any(
        not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-9)
        for actual, expected in zip(physical_bounds, expected_physical_bounds)
    ):
        raise Nav2ProfileError("planning footprint does not match the measured Go2 body")
    padded_bounds = _padded_bounds(footprint, footprint_padding)
    expected_padded_bounds = (-0.43, 0.50, -0.30, 0.30)
    if any(
        not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-9)
        for actual, expected in zip(padded_bounds, expected_padded_bounds)
    ):
        raise Nav2ProfileError("padded planning envelope is inconsistent")
    lateral_half_width = max(abs(padded_bounds[2]), abs(padded_bounds[3]))
    inflation = _mapping(local.get("inflation_layer"), "inflation_layer")
    inflation_radius = _positive(inflation.get("inflation_radius"), "inflation_radius")
    clearance_envelope = inflation_radius - lateral_half_width
    if clearance_envelope + 1.0e-9 < 0.15:
        raise Nav2ProfileError(
            "inflation radius must retain a 0.15m soft shoulder preference"
        )
    if clearance_envelope > 0.20 + 1.0e-9:
        raise Nav2ProfileError("soft inflation must not recreate the oversized hard circle")
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
    if plugins != ["static_layer", "obstacle_layer", "inflation_layer"]:
        raise Nav2ProfileError(
            "local costmap must use static, live obstacle and inflation layers"
        )
    local_static = _mapping(local.get("static_layer"), "local static_layer")
    if (
        local_static.get("plugin") != "nav2_costmap_2d::StaticLayer"
        or local_static.get("map_topic") != "/navigation_static_map"
    ):
        raise Nav2ProfileError("local MPPI must consume the reviewed static map")
    if local.get("filters") != ["keepout_filter"]:
        raise Nav2ProfileError("local MPPI must consume the allowed-area mask")
    local_keepout = _mapping(local.get("keepout_filter"), "local keepout_filter")
    if local_keepout.get("plugin") != "nav2_costmap_2d::KeepoutFilter":
        raise Nav2ProfileError("local allowed area must use Nav2 KeepoutFilter")
    obstacle = _mapping(local.get("obstacle_layer"), "obstacle_layer")
    if obstacle.get("plugin") != "nav2_costmap_2d::ObstacleLayer":
        raise Nav2ProfileError("local obstacle owner must be the 2D ObstacleLayer")
    source = _mapping(obstacle.get("mid360_body"), "obstacle_layer.mid360_body")
    if source.get("topic") != "/navigation/cloud_obstacles" or source.get("data_type") != "PointCloud2":
        raise Nav2ProfileError("local costmap must use the self-filtered obstacle cloud")
    if source.get("clearing") is not True or source.get("marking") is not True:
        raise Nav2ProfileError("body cloud must both clear and mark the rolling costmap")

    planner = _mapping(profile.get("planner_server"), "planner_server")
    planner = _mapping(planner.get("ros__parameters"), "planner parameters")
    if planner.get("planner_plugins") != ["GridBased"]:
        raise Nav2ProfileError("one GridBased global planner is required")
    grid_planner = _mapping(planner.get("GridBased"), "GridBased planner")
    if grid_planner.get("plugin") != "nav2_smac_planner/SmacPlanner2D":
        raise Nav2ProfileError("large bypasses must use Nav2 SmacPlanner2D")
    if grid_planner.get("allow_unknown") is not False:
        raise Nav2ProfileError("global planning must remain inside observed free space")

    global_costmap = _mapping(profile.get("global_costmap"), "global_costmap")
    global_costmap = _mapping(
        global_costmap.get("global_costmap"), "global_costmap.global_costmap"
    )
    global_costmap = _mapping(
        global_costmap.get("ros__parameters"), "global costmap parameters"
    )
    if "robot_radius" in global_costmap:
        raise Nav2ProfileError(
            "global costmap must not define a circular radius beside the footprint"
        )
    global_footprint = _footprint(
        global_costmap.get("footprint"), "global_costmap.footprint"
    )
    global_padding = _finite(
        global_costmap.get("footprint_padding"),
        "global_costmap.footprint_padding",
    )
    if global_footprint != footprint or not math.isclose(
        global_padding, footprint_padding, rel_tol=0.0, abs_tol=1.0e-9
    ):
        raise Nav2ProfileError("local and global costmaps must share one footprint")
    if global_costmap.get("rolling_window") is not False:
        raise Nav2ProfileError(
            "global planning must use the full reviewed static navigation map"
        )
    if global_costmap.get("track_unknown_space") is not True:
        raise Nav2ProfileError("global planning must not treat unknown space as free")
    if global_costmap.get("plugins") != [
        "static_layer", "obstacle_layer", "inflation_layer"
    ]:
        raise Nav2ProfileError(
            "global planning requires static, live obstacle and inflation layers"
        )
    static_layer = _mapping(global_costmap.get("static_layer"), "static_layer")
    if (
        static_layer.get("plugin") != "nav2_costmap_2d::StaticLayer"
        or static_layer.get("map_topic") != "/navigation_static_map"
    ):
        raise Nav2ProfileError("global static layer must consume the reviewed map")
    if global_costmap.get("filters") != ["keepout_filter"]:
        raise Nav2ProfileError("global planning must consume the allowed-area mask")
    keepout = _mapping(global_costmap.get("keepout_filter"), "keepout_filter")
    if keepout.get("plugin") != "nav2_costmap_2d::KeepoutFilter":
        raise Nav2ProfileError("allowed area must use the standard Nav2 KeepoutFilter")

    collision = _mapping(profile.get("collision_monitor"), "collision_monitor")
    collision = _mapping(collision.get("ros__parameters"), "collision monitor parameters")
    if collision.get("cmd_vel_in_topic") != "/nav2/smoothed_cmd_vel":
        raise Nav2ProfileError("collision monitor input topic breaks the motion chain")
    if collision.get("cmd_vel_out_topic") != "/patrol_cmd":
        raise Nav2ProfileError("collision monitor output topic breaks the motion chain")
    if collision.get("base_frame_id") != "base_link" or collision.get("odom_frame_id") != "odom":
        raise Nav2ProfileError("collision monitor frames must be odom -> base_link")
    collision_source = _mapping(collision.get("mid360_body"), "collision mid360_body")
    if collision_source.get("type") != "pointcloud" or collision_source.get("topic") != "/navigation/cloud_obstacles":
        raise Nav2ProfileError("collision monitor must use the self-filtered obstacle cloud")

    if collision.get("polygons") != ["SafetyEnvelope"]:
        raise Nav2ProfileError("collision monitor must expose one safety envelope")
    safety_envelope = _mapping(
        collision.get("SafetyEnvelope"), "SafetyEnvelope"
    )
    if safety_envelope.get("type") != "polygon":
        raise Nav2ProfileError("SafetyEnvelope must use the rectangular body")
    safety_points = _points(safety_envelope.get("points"), "SafetyEnvelope.points")
    safety_bounds = _bounds(safety_points)
    if any(
        not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-9)
        for actual, expected in zip(safety_bounds, padded_bounds)
    ):
        raise Nav2ProfileError(
            "collision monitor and costmaps must share one padded envelope"
        )
    max_points = safety_envelope.get("max_points")
    if not isinstance(max_points, int) or isinstance(max_points, bool) or not 3 <= max_points <= 12:
        raise Nav2ProfileError(
            "SafetyEnvelope must require between four and thirteen coherent points"
        )
    collision_min_height = _finite(
        collision_source.get("min_height"), "collision mid360_body.min_height"
    )
    if collision_min_height < 0.03:
        raise Nav2ProfileError(
            "collision source must reject the calibrated floor-return band"
        )
    if safety_envelope.get("action_type") != "stop":
        raise Nav2ProfileError("SafetyEnvelope must be an emergency stop boundary")
    prediction_distance = time_steps * model_dt * vx_max

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

    behavior = _mapping(profile.get("behavior_server"), "behavior_server")
    behavior = _mapping(behavior.get("ros__parameters"), "behavior parameters")
    if behavior.get("behavior_plugins") != ["spin"]:
        raise Nav2ProfileError("checkpoint inspection requires only the Nav2 Spin behavior")
    spin = _mapping(behavior.get("spin"), "behavior spin")
    if spin.get("plugin") != "nav2_behaviors/Spin":
        raise Nav2ProfileError("checkpoint inspection must use the Humble Nav2 Spin plugin")
    if behavior.get("global_frame") != "map" or behavior.get("robot_base_frame") != "base_link":
        raise Nav2ProfileError("Nav2 Spin frames must be map -> base_link")
    if behavior.get("costmap_topic") != "local_costmap/costmap_raw":
        raise Nav2ProfileError("Nav2 Spin must collision-check the local costmap")
    spin_max = _positive(behavior.get("max_rotational_vel"), "max_rotational_vel")
    spin_min = _positive(behavior.get("min_rotational_vel"), "min_rotational_vel")
    if spin_min > spin_max or spin_max > _finite(max_velocity[2], "max_velocity.yaw"):
        raise Nav2ProfileError("Nav2 Spin speed exceeds the commissioned yaw chain")

    return {
        "controllerFrequencyHz": frequency,
        "modelDtS": model_dt,
        "predictionTimeS": time_steps * model_dt,
        "predictionDistanceM": prediction_distance,
        "lateralAuthorityMps": vy_max,
        "requiredMovementAngleRad": required_movement_angle,
        "physicalFootprintBoundsM": physical_bounds,
        "paddedFootprintBoundsM": padded_bounds,
        "shoulderClearanceM": footprint_padding,
        "lateralHalfWidthM": lateral_half_width,
        "inflationRadiusM": inflation_radius,
        "inflationClearanceEnvelopeM": clearance_envelope,
        "inflationCostScalingFactor": inflation_cost_scaling,
        "globalCostmapSource": "/navigation_static_map",
        "safetyEnvelopeBoundsM": safety_bounds,
        "batchSize": batch_size,
        "iterationCount": iteration_count,
        "rolloutStatesPerSecond": rollout_states_per_second,
        "spinMaxRotationalVelocityRps": spin_max,
    }
