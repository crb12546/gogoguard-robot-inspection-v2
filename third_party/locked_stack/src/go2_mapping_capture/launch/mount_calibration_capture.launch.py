"""Passive, no-command capture for solving T_base_lidar.

This launch intentionally does not include mount_tf_publisher, the production
FAST-LIO launch, Nav2, a route follower, or any Unitree command publisher.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device_config = LaunchConfiguration("device_config")
    fastlio_config = LaunchConfiguration("fastlio_config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("livox_ros_driver2"),
                        "config",
                        "MID360s_config.json",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "fastlio_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("go2_mapping_capture"),
                        "config",
                        "go2_mid360_mount_calibration_fastlio.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "robot_id",
                default_value=EnvironmentVariable("GO2_ROBOT_ID", default_value=""),
            ),
            DeclareLaunchArgument(
                "sensor_id",
                default_value=EnvironmentVariable(
                    "GO2_LIDAR_SENSOR_ID", default_value=""
                ),
            ),
            DeclareLaunchArgument("capture_id", default_value=""),
            DeclareLaunchArgument(
                "recording_root",
                default_value=EnvironmentVariable(
                    "GO2_CALIBRATION_CAPTURE_ROOT",
                    default_value="/data/go2/mount-calibration",
                ),
            ),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_mount_calibration_source",
                output="screen",
                parameters=[
                    {
                        "xfer_format": 0,
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": 10.0,
                        "output_data_type": 0,
                        "frame_id": "lidar_link",
                        "imu_frame_id": "lidar_imu",
                        "user_config_path": device_config,
                    }
                ],
                remappings=[
                    ("/livox/lidar", "/calibration/livox/lidar"),
                    ("/livox/imu", "/calibration/livox/imu"),
                ],
            ),
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="mount_calibration_lidar_odometry",
                output="screen",
                parameters=[
                    fastlio_config,
                    {
                        "output.frame_mode": "lidar_calibration",
                        "output.require_base_frame_calibration": False,
                    },
                ],
            ),
            Node(
                package="go2_mapping_capture",
                executable="mount_calibration_recorder",
                name="mount_calibration_recorder",
                output="screen",
                parameters=[
                    {
                        "robot_id": LaunchConfiguration("robot_id"),
                        "sensor_id": LaunchConfiguration("sensor_id"),
                        "capture_id": LaunchConfiguration("capture_id"),
                        "recording_root": LaunchConfiguration("recording_root"),
                    }
                ],
            ),
        ]
    )
