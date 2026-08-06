"""Bring up localization and navigation from one immutable active map."""

import hashlib
import math
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from go2_nav2_runtime.runtime_core import (
    load_candidate_runtime_bundle,
    load_runtime_bundle,
)
from go2_nav2_runtime.nav2_profile_contract import validate_nav2_profile


def _sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _runtime_nodes(context):
    map_store_root = LaunchConfiguration("map_store_root").perform(context).strip()
    site_id = LaunchConfiguration("site_id").perform(context).strip()
    expected_version = LaunchConfiguration("expected_map_version").perform(context).strip()
    runtime_source = LaunchConfiguration("runtime_source").perform(context).strip()
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    safety_stream_timeout_s = ParameterValue(
        LaunchConfiguration("safety_stream_timeout_s"), value_type=float
    )
    runtime_binding_check_period_s = ParameterValue(
        LaunchConfiguration("runtime_binding_check_period_s"), value_type=float
    )
    robot_state_timeout_s = ParameterValue(
        LaunchConfiguration("robot_state_timeout_s"), value_type=float
    )
    straight_speed = float(LaunchConfiguration("straight_speed_mps").perform(context))
    turn_speed = float(LaunchConfiguration("turn_speed_radps").perform(context))
    lateral_speed = float(LaunchConfiguration("lateral_speed_mps").perform(context))
    acceleration = float(LaunchConfiguration("acceleration_mps2").perform(context))
    stop_front = float(LaunchConfiguration("stop_zone_front_m").perform(context))
    stop_rear = float(LaunchConfiguration("stop_zone_rear_m").perform(context))
    stop_half_width = float(LaunchConfiguration("stop_zone_half_width_m").perform(context))
    slow_front = float(LaunchConfiguration("slow_zone_front_m").perform(context))
    slow_rear = float(LaunchConfiguration("slow_zone_rear_m").perform(context))
    slow_half_width = float(LaunchConfiguration("slow_zone_half_width_m").perform(context))
    slowdown_ratio = float(LaunchConfiguration("slowdown_ratio").perform(context))
    blocked_decision_s = float(
        LaunchConfiguration("blocked_decision_s").perform(context)
    )
    controller_frequency = float(LaunchConfiguration("controller_frequency_hz").perform(context))
    mppi_time_steps = int(LaunchConfiguration("mppi_time_steps").perform(context))
    mppi_batch_size = int(LaunchConfiguration("mppi_batch_size").perform(context))
    mppi_iterations = int(LaunchConfiguration("mppi_iteration_count").perform(context))
    hardware_output = LaunchConfiguration("hardware_output_enabled").perform(context)
    hardware_output = hardware_output.strip().lower()
    if hardware_output not in {"true", "false"}:
        raise RuntimeError("hardware_output_enabled must be true or false")
    sdk_receiver_enabled = LaunchConfiguration("sdk_receiver_enabled").perform(context)
    sdk_receiver_enabled = sdk_receiver_enabled.strip().lower()
    if sdk_receiver_enabled not in {"true", "false"}:
        raise RuntimeError("sdk_receiver_enabled must be true or false")
    candidate_parameters = {}
    if runtime_source == "active":
        bundle = load_runtime_bundle(map_store_root, site_id, expected_version)
    elif runtime_source == "candidate":
        candidate_parameters = {
            "candidate.localization_map": LaunchConfiguration(
                "candidate_localization_map"
            ).perform(context).strip(),
            "candidate.route": LaunchConfiguration("candidate_route").perform(context).strip(),
            "candidate.runtime_profile": LaunchConfiguration(
                "candidate_runtime_profile"
            ).perform(context).strip(),
            "candidate.localization_map_hash": LaunchConfiguration(
                "localization_map_hash"
            ).perform(context).strip(),
            "candidate.route_hash": LaunchConfiguration("route_hash").perform(context).strip(),
            "candidate.runtime_profile_hash": LaunchConfiguration(
                "runtime_profile_hash"
            ).perform(context).strip(),
        }
        bundle = load_candidate_runtime_bundle(
            site_id=site_id,
            version_id=expected_version,
            localization_map_path=candidate_parameters["candidate.localization_map"],
            route_path=candidate_parameters["candidate.route"],
            runtime_profile_path=candidate_parameters["candidate.runtime_profile"],
            localization_map_hash=candidate_parameters["candidate.localization_map_hash"],
            route_hash=candidate_parameters["candidate.route_hash"],
            runtime_profile_hash=candidate_parameters["candidate.runtime_profile_hash"],
        )
    else:
        raise RuntimeError("runtime_source must be active or candidate")

    nav2_share = Path(get_package_share_directory("go2_nav2_runtime"))
    map_manager_share = Path(get_package_share_directory("go2_map_manager"))
    nav2_config = str(nav2_share / "config" / "go2_nav2_patrol.yaml")
    validate_nav2_profile(Path(nav2_config))
    localization_config = str(
        map_manager_share / "config" / "continuous_map_localizer.yaml"
    )
    # The route owns geometry and start/end behavior. The active, versioned
    # robot profile owns commissioning speed and avoidance behavior.
    speed = straight_speed
    robot_id = LaunchConfiguration("robot_id").perform(context).strip()
    sensor_id = LaunchConfiguration("sensor_id").perform(context).strip()
    if runtime_source == "active":
        if not robot_id or not sensor_id:
            raise RuntimeError("robot_id and sensor_id are required for mount calibration")
        if bundle.calibration_bundle_path is None:
            raise RuntimeError("active release has no sealed calibration bundle")
        if robot_id != bundle.robot_id or sensor_id != bundle.sensor_id:
            raise RuntimeError(
                "active release calibration belongs to different robot/sensor hardware"
            )
    sdk_interface = LaunchConfiguration("sdk_interface").perform(context).strip()
    sdk_receiver = ""
    if hardware_output == "true" and sdk_receiver_enabled == "true":
        if not sdk_interface or any(
            character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-"
            for character in sdk_interface
        ):
            raise RuntimeError("sdk_interface contains unsafe characters")
        bridge_prefix = Path(get_package_prefix("go2_cmd_vel_bridge"))
        sdk_receiver = str(
            bridge_prefix
            / "lib"
            / "go2_cmd_vel_bridge"
            / "go2_sdk2_udp_receiver"
        )
    initial = bundle.initialization_center
    common_tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    runtime_nodes = []
    if runtime_source == "active":
        runtime_nodes.extend(
            [
                Node(
                    package="go2_site_ops",
                    executable="mount_tf_publisher",
                    name="mount_tf_publisher",
                    output="screen",
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[
                        {
                            "release_calibration_bundle_path": str(
                                bundle.calibration_bundle_path
                            ),
                            "robot_id": robot_id,
                            "sensor_id": sensor_id,
                            "use_sim_time": use_sim_time,
                        }
                    ],
                ),
                Node(
                    package="go2_map_manager",
                    executable="calibrated_cloud_transformer",
                    name="calibrated_cloud_transformer",
                    output="screen",
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[
                        {
                            "input_topic": "/navigation/cloud_lidar",
                            "output_topic": "/navigation/cloud_body",
                            "source_frame": "lidar_link",
                            "target_frame": "base_link",
                            "use_sim_time": use_sim_time,
                        }
                    ],
                    remappings=common_tf_remaps,
                ),
            ]
        )
    manager_node = Node(
        package="go2_nav2_runtime",
        executable="patrol_runtime_manager",
        name="patrol_runtime_manager",
        output="screen",
        parameters=[
            {
                "map_store_root": map_store_root,
                "site_id": site_id,
                "expected_map_version": bundle.version_id,
                "runtime_source": runtime_source,
                "nav2_profile_hash": _sha256_file(nav2_config),
                "runtime_trace_status_timeout_s": ParameterValue(
                    LaunchConfiguration("runtime_trace_status_timeout_s"),
                    value_type=float,
                ),
                "use_sim_time": use_sim_time,
                "localization_timeout_s": ParameterValue(
                    LaunchConfiguration("localization_status_timeout_s"), value_type=float
                ),
                "localization_dropout_grace_s": ParameterValue(
                    LaunchConfiguration("localization_dropout_grace_s"), value_type=float
                ),
                "localization_recovery_stable_s": ParameterValue(
                    LaunchConfiguration("localization_recovery_stable_s"), value_type=float
                ),
                "blocked_decision_s": ParameterValue(
                    LaunchConfiguration("blocked_decision_s"), value_type=float
                ),
                "rejoin_lookahead_m": ParameterValue(
                    LaunchConfiguration("rejoin_lookahead_m"), value_type=float
                ),
                "active_map_check_period_s": runtime_binding_check_period_s,
                "robot_state_timeout_s": robot_state_timeout_s,
                "fastlio_health.settle_window_s": ParameterValue(
                    LaunchConfiguration("fastlio_settle_window_s"),
                    value_type=float,
                ),
                "fastlio_health.max_stationary_translation_m": ParameterValue(
                    LaunchConfiguration("fastlio_max_stationary_translation_m"),
                    value_type=float,
                ),
                "fastlio_health.max_stationary_rotation_deg": ParameterValue(
                    LaunchConfiguration("fastlio_max_stationary_rotation_deg"),
                    value_type=float,
                ),
                "fastlio_health.odom_timeout_s": ParameterValue(
                    LaunchConfiguration("fastlio_odom_timeout_s"),
                    value_type=float,
                ),
                "fastlio_health.max_frame_gap_s": ParameterValue(
                    LaunchConfiguration("fastlio_max_frame_gap_s"),
                    value_type=float,
                ),
                "fastlio_health.minimum_samples": ParameterValue(
                    LaunchConfiguration("fastlio_minimum_settle_samples"),
                    value_type=int,
                ),
                **candidate_parameters,
            }
        ],
    )
    runtime_generation_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=manager_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason="runtime generation owner exited")
                )
            ],
        )
    )
    # Register the generation owner before any process starts so even an
    # immediate startup rejection tears down the entire coherent generation.
    runtime_nodes.insert(0, runtime_generation_shutdown)
    runtime_nodes.extend([
        ExecuteProcess(
            cmd=[
                "python3",
                "/home/unitree/go2_fastlio_ws/scripts/patrol_performance_monitor.py",
                "--interval",
                "1.0",
                "--filesystem-path",
                LaunchConfiguration("runtime_log_dir"),
                "--output",
                PathJoinSubstitution(
                    [LaunchConfiguration("runtime_log_dir"), "performance.jsonl"]
                ),
                "--rotate-bytes",
                LaunchConfiguration("performance_log_rotate_bytes"),
                "--rotate-files",
                LaunchConfiguration("performance_log_rotate_files"),
            ],
            name="patrol_performance_monitor",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="go2_nav2_runtime",
            executable="runtime_trace_recorder",
            name="runtime_trace_recorder",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                {
                    "output_dir": LaunchConfiguration("runtime_log_dir"),
                    "site_id": bundle.site_id,
                    "map_version": bundle.version_id,
                    "manifest_hash": bundle.manifest_hash,
                    "route_hash": bundle.route.source_hash,
                    "runtime_profile_hash": _sha256_file(
                        bundle.runtime_profile_path
                    ),
                    "nav2_profile_hash": _sha256_file(nav2_config),
                    "calibration_hash": bundle.calibration_hash,
                    "robot_id": robot_id,
                    "sensor_id": sensor_id,
                    "localization_quality_profile_id": (
                        bundle.localization_quality.profile_id
                    ),
                    "runtime_source": runtime_source,
                    "max_file_bytes": ParameterValue(
                        LaunchConfiguration("runtime_trace_max_file_bytes"),
                        value_type=int,
                    ),
                    "retained_files": ParameterValue(
                        LaunchConfiguration("runtime_trace_retained_files"),
                        value_type=int,
                    ),
                    "minimum_free_bytes": ParameterValue(
                        LaunchConfiguration("runtime_log_minimum_free_bytes"),
                        value_type=int,
                    ),
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
        Node(
            package="go2_map_manager",
            executable="continuous_map_localizer",
            name="continuous_map_localizer",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                localization_config,
                {
                    "map_file": str(bundle.localization_map_path),
                    "map_version": bundle.version_id,
                    **bundle.localization_quality.ros_parameters(),
                    "initialization.center_x": initial[0],
                    "initialization.center_y": initial[1],
                    "initialization.center_z": initial[2],
                    "initialization.xy_radius": bundle.initialization_radius_m,
                    "initialization.xy_step": min(
                        1.0, bundle.initialization_radius_m
                    ),
                    "initialization.yaw_center_deg": math.degrees(
                        bundle.initialization_yaw_rad
                    ),
                    "initialization.yaw_radius_deg": (
                        bundle.initialization_yaw_tolerance_deg
                    ),
                    "initialization.yaw_step_deg": min(
                        15.0, bundle.initialization_yaw_tolerance_deg
                    ),
                    "publish_odom_base_tf": runtime_source == "candidate",
                    "use_sim_time": use_sim_time,
                },
            ],
            remappings=common_tf_remaps,
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                nav2_config,
                {
                    "controller_frequency": controller_frequency,
                    "FollowPath.vx_max": speed,
                    "FollowPath.model_dt": 1.0 / controller_frequency,
                    "FollowPath.vy_max": lateral_speed,
                    "FollowPath.wz_max": turn_speed,
                    "FollowPath.time_steps": mppi_time_steps,
                    "FollowPath.batch_size": mppi_batch_size,
                    "FollowPath.iteration_count": mppi_iterations,
                    "progress_checker.movement_time_allowance": blocked_decision_s,
                    "use_sim_time": use_sim_time,
                },
            ],
            remappings=common_tf_remaps
            + [("cmd_vel", "/nav2/raw_cmd_vel"), ("odom", "/Odometry")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                nav2_config,
                {
                    "max_velocity": [speed, lateral_speed, turn_speed],
                    # The immutable route is directional. Keep lateral and yaw
                    # authority for an omni bypass, but never re-authorize
                    # reverse x after the sealed profile rejected it.
                    "min_velocity": [0.0, -lateral_speed, -turn_speed],
                    "max_accel": [acceleration, max(0.35, acceleration * 0.5), 0.80],
                    "use_sim_time": use_sim_time,
                },
            ],
            remappings=common_tf_remaps
            + [
                ("cmd_vel", "/nav2/raw_cmd_vel"),
                ("cmd_vel_smoothed", "/nav2/smoothed_cmd_vel"),
            ],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                nav2_config,
                {
                    "StopZone.points": [
                        stop_front, stop_half_width, stop_front, -stop_half_width,
                        -stop_rear, -stop_half_width, -stop_rear, stop_half_width,
                    ],
                    "SlowZone.points": [
                        slow_front, slow_half_width, slow_front, -slow_half_width,
                        -slow_rear, -slow_half_width, -slow_rear, slow_half_width,
                    ],
                    "SlowZone.slowdown_ratio": slowdown_ratio,
                    "use_sim_time": use_sim_time,
                },
            ],
            remappings=common_tf_remaps,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_go2_patrol",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "bond_timeout": 4.0,
                    "attempt_respawn_reconnection": True,
                    "use_sim_time": use_sim_time,
                    "node_names": [
                        "controller_server",
                        "velocity_smoother",
                        "collision_monitor",
                    ],
                }
            ],
        ),
        manager_node,
        Node(
            package="go2_fastlio_patrol",
            executable="unitree_safe_cmd_node",
            name="unitree_safe_cmd_node",
            output="screen",
            respawn=True,
            respawn_delay=1.0,
            parameters=[
                {
                    "cmd_topic": "/patrol_cmd",
                    "pointcloud_topic": "/navigation/cloud_body",
                    "expected_cloud_frame": "base_link",
                    "expected_map_version": bundle.version_id,
                    # Collision Monitor is the sole point-cloud obstacle owner;
                    # the final bridge retains only authorization, watchdog,
                    # finite/range validation and absolute hardware caps.
                    "require_localization": False,
                    "require_obstacle_gate": False,
                    "require_runtime_authorization": True,
                    "cmd_timeout": safety_stream_timeout_s,
                    "cloud_timeout": safety_stream_timeout_s,
                    "localization_timeout": safety_stream_timeout_s,
                    "runtime_authorization_timeout": safety_stream_timeout_s,
                    "max_vx": speed,
                    "max_vy": lateral_speed,
                    "max_yaw_rate": turn_speed,
                    "output_cmd_topic": "/cmd_vel",
                    "roi_z_min": -0.10,
                    "roi_z_max": 1.45,
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
    ])
    if hardware_output == "true":
        runtime_nodes.append(
            Node(
                package="go2_cmd_vel_bridge",
                executable="cmd_vel_udp_sender",
                name="cmd_vel_udp_sender",
                output="screen",
                respawn=True,
                respawn_delay=1.0,
                parameters=[
                    {
                        "target_ip": "127.0.0.1",
                        "target_port": 5005,
                        "max_vx": speed,
                        "max_vy": lateral_speed,
                        "max_vyaw": turn_speed,
                        "unitree_vy_sign": 1.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        )
    if hardware_output == "true" and sdk_receiver_enabled == "true":
        runtime_nodes.append(
            ExecuteProcess(
                cmd=[sdk_receiver, sdk_interface, "5005"],
                name="go2_sdk2_udp_receiver",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                additional_env={
                    "GO2_SDK_EVENT_LOG": PathJoinSubstitution(
                        [LaunchConfiguration("runtime_log_dir"), "sdk-events.jsonl"]
                    )
                },
            )
        )
    return runtime_nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "runtime_log_dir",
                default_value=EnvironmentVariable(
                    "GO2_RUNTIME_LOG_DIR",
                    default_value="/data/go2/runtime_logs",
                ),
            ),
            DeclareLaunchArgument(
                "runtime_trace_max_file_bytes",
                default_value=EnvironmentVariable(
                    "GO2_RUNTIME_TRACE_MAX_FILE_BYTES",
                    default_value="67108864",
                ),
            ),
            DeclareLaunchArgument(
                "runtime_trace_retained_files",
                default_value=EnvironmentVariable(
                    "GO2_RUNTIME_TRACE_RETAINED_FILES",
                    default_value="24",
                ),
            ),
            DeclareLaunchArgument(
                "runtime_log_minimum_free_bytes",
                default_value=EnvironmentVariable(
                    "GO2_RUNTIME_LOG_MINIMUM_FREE_BYTES",
                    default_value="536870912",
                ),
            ),
            DeclareLaunchArgument(
                "runtime_trace_status_timeout_s",
                default_value=EnvironmentVariable(
                    "GO2_RUNTIME_TRACE_STATUS_TIMEOUT_S",
                    default_value="0.50",
                ),
            ),
            DeclareLaunchArgument(
                "performance_log_rotate_bytes",
                default_value=EnvironmentVariable(
                    "GO2_PERFORMANCE_LOG_ROTATE_BYTES",
                    default_value="67108864",
                ),
            ),
            DeclareLaunchArgument(
                "performance_log_rotate_files",
                default_value=EnvironmentVariable(
                    "GO2_PERFORMANCE_LOG_ROTATE_FILES",
                    default_value="8",
                ),
            ),
            DeclareLaunchArgument(
                "map_store_root",
                default_value=EnvironmentVariable(
                    "GO2_MAP_STORE_ROOT", default_value="/data/go2/maps"
                ),
            ),
            DeclareLaunchArgument(
                "site_id",
                default_value=EnvironmentVariable("GO2_SITE_ID", default_value=""),
            ),
            DeclareLaunchArgument("expected_map_version", default_value=""),
            DeclareLaunchArgument("runtime_source", default_value="active"),
            DeclareLaunchArgument("candidate_localization_map", default_value=""),
            DeclareLaunchArgument("candidate_route", default_value=""),
            DeclareLaunchArgument("candidate_runtime_profile", default_value=""),
            DeclareLaunchArgument("localization_map_hash", default_value=""),
            DeclareLaunchArgument("route_hash", default_value=""),
            DeclareLaunchArgument("runtime_profile_hash", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("hardware_output_enabled", default_value="true"),
            DeclareLaunchArgument("sdk_receiver_enabled", default_value="true"),
            DeclareLaunchArgument(
                "safety_stream_timeout_s",
                default_value=EnvironmentVariable(
                    "GO2_SAFETY_STREAM_TIMEOUT_S", default_value="0.20"
                ),
            ),
            DeclareLaunchArgument("localization_status_timeout_s", default_value="0.60"),
            DeclareLaunchArgument("straight_speed_mps", default_value="0.60"),
            DeclareLaunchArgument("turn_speed_radps", default_value="0.40"),
            DeclareLaunchArgument("lateral_speed_mps", default_value="0.20"),
            DeclareLaunchArgument("acceleration_mps2", default_value="0.90"),
            DeclareLaunchArgument("stop_zone_front_m", default_value="0.55"),
            DeclareLaunchArgument("stop_zone_rear_m", default_value="0.38"),
            DeclareLaunchArgument("stop_zone_half_width_m", default_value="0.27"),
            DeclareLaunchArgument("slow_zone_front_m", default_value="1.00"),
            DeclareLaunchArgument("slow_zone_rear_m", default_value="0.55"),
            DeclareLaunchArgument("slow_zone_half_width_m", default_value="0.30"),
            DeclareLaunchArgument("slowdown_ratio", default_value="0.85"),
            DeclareLaunchArgument("blocked_decision_s", default_value="2.5"),
            DeclareLaunchArgument("rejoin_lookahead_m", default_value="2.0"),
            DeclareLaunchArgument("localization_dropout_grace_s", default_value="1.0"),
            DeclareLaunchArgument("localization_recovery_stable_s", default_value="0.5"),
            DeclareLaunchArgument("controller_frequency_hz", default_value="15.0"),
            DeclareLaunchArgument("mppi_time_steps", default_value="56"),
            DeclareLaunchArgument("mppi_batch_size", default_value="700"),
            DeclareLaunchArgument("mppi_iteration_count", default_value="1"),
            DeclareLaunchArgument(
                "runtime_binding_check_period_s",
                default_value=EnvironmentVariable(
                    "GO2_RUNTIME_BINDING_CHECK_PERIOD_S", default_value="0.10"
                ),
            ),
            DeclareLaunchArgument(
                "robot_state_timeout_s",
                default_value=EnvironmentVariable(
                    "GO2_ROBOT_STATE_TIMEOUT_S", default_value="0.20"
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_settle_window_s",
                default_value=EnvironmentVariable(
                    "GO2_FASTLIO_SETTLE_WINDOW_S", default_value="5.0"
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_max_stationary_translation_m",
                default_value=EnvironmentVariable(
                    "GO2_FASTLIO_MAX_STATIONARY_TRANSLATION_M",
                    default_value="0.05",
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_max_stationary_rotation_deg",
                default_value=EnvironmentVariable(
                    "GO2_FASTLIO_MAX_STATIONARY_ROTATION_DEG",
                    default_value="1.0",
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_odom_timeout_s",
                default_value=EnvironmentVariable(
                    "GO2_FASTLIO_ODOM_TIMEOUT_S", default_value="0.5"
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_max_frame_gap_s",
                default_value=EnvironmentVariable(
                    "GO2_FASTLIO_MAX_FRAME_GAP_S", default_value="0.5"
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_minimum_settle_samples",
                default_value=EnvironmentVariable(
                    "GO2_FASTLIO_MINIMUM_SETTLE_SAMPLES", default_value="30"
                ),
            ),
            DeclareLaunchArgument(
                "robot_id",
                default_value=EnvironmentVariable("GO2_ROBOT_ID", default_value=""),
            ),
            DeclareLaunchArgument(
                "sensor_id",
                default_value=EnvironmentVariable("GO2_LIDAR_SENSOR_ID", default_value=""),
            ),
            DeclareLaunchArgument(
                "sdk_interface",
                default_value=EnvironmentVariable("GO2_SDK_IF", default_value="eth0"),
            ),
            OpaqueFunction(function=_runtime_nodes),
        ]
    )
