"""Publish a dedicated timestamped PointCloud2 stream for GLIM capture.

This launch is exclusive with the normal CustomMsg/FAST-LIO Livox driver.
The process supervisor must stop patrol motion and the normal LiDAR process
before launching this mapping source.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
                description="Livox network/device JSON for this robot",
            ),
            DeclareLaunchArgument(
                "fastlio_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("go2_mapping_capture"),
                        "config",
                        "go2_mid360_capture_fastlio.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "mount_calibration",
                default_value=EnvironmentVariable(
                    "GO2_MOUNT_CALIBRATION", default_value=""
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
            DeclareLaunchArgument("console_enabled", default_value="true"),
            DeclareLaunchArgument(
                "console_base_url",
                default_value=EnvironmentVariable(
                    "GO2_SITE_CONSOLE_URL", default_value="http://127.0.0.1:8765"
                ),
            ),
            DeclareLaunchArgument(
                "console_token_file",
                default_value=EnvironmentVariable(
                    "GO2_SITE_CONSOLE_TOKEN_FILE", default_value=""
                ),
            ),
            DeclareLaunchArgument("capture_id", default_value=""),
            DeclareLaunchArgument(
                "recording_root",
                default_value=EnvironmentVariable(
                    "GO2_CAPTURE_ROOT", default_value="/data/go2/captures"
                ),
            ),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_mapping_publisher",
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
                    ("/livox/lidar", "/mapping/livox/lidar"),
                    ("/livox/imu", "/mapping/livox/imu"),
                ],
            ),
            Node(
                package="go2_site_ops",
                executable="mount_tf_publisher",
                name="mapping_mount_tf_publisher",
                output="screen",
                parameters=[
                    {
                        "calibration_path": LaunchConfiguration("mount_calibration"),
                        "robot_id": LaunchConfiguration("robot_id"),
                        "sensor_id": LaunchConfiguration("sensor_id"),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("fast_lio"), "launch", "mapping.launch.py"]
                    )
                ),
                launch_arguments={
                    "config": fastlio_config,
                    "rviz": "false",
                    "mount_calibration": LaunchConfiguration("mount_calibration"),
                    "robot_id": LaunchConfiguration("robot_id"),
                    "sensor_id": LaunchConfiguration("sensor_id"),
                }.items(),
            ),
            Node(
                package="go2_site_ops",
                executable="site_console_capture_adapter",
                name="site_console_capture_adapter",
                output="screen",
                condition=IfCondition(LaunchConfiguration("console_enabled")),
                parameters=[
                    {
                        "console_base_url": LaunchConfiguration("console_base_url"),
                        "console_token_file": LaunchConfiguration("console_token_file"),
                        "capture_id": LaunchConfiguration("capture_id"),
                        "recording_root": LaunchConfiguration("recording_root"),
                    }
                ],
            ),
        ]
    )
