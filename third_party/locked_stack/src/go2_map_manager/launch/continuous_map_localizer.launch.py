# Copyright 2026 GoGoGuard

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("go2_map_manager"))
    default_config = str(package_share / "config" / "continuous_map_localizer.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("map_file"),
            DeclareLaunchArgument("map_version"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            Node(
                package="go2_map_manager",
                executable="continuous_map_localizer",
                name="continuous_map_localizer",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "map_file": LaunchConfiguration("map_file"),
                        "map_version": LaunchConfiguration("map_version"),
                        "initialization.center_x": ParameterValue(
                            LaunchConfiguration("initial_x"), value_type=float
                        ),
                        "initialization.center_y": ParameterValue(
                            LaunchConfiguration("initial_y"), value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
