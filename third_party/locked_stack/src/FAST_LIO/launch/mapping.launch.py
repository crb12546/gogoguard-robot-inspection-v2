import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from go2_site_ops.calibration import MountCalibration
from go2_site_ops.coordinate_contract import CoordinateContract
from go2_site_ops.internal_calibration import SensorInternalCalibration


def _build_runtime(context):
    contract = CoordinateContract.load(
        Path(LaunchConfiguration("coordinate_contract").perform(context)).resolve()
    )
    mount = MountCalibration.load(
        Path(LaunchConfiguration("mount_calibration").perform(context)).resolve(),
        require_validated=True,
    )
    robot_id = LaunchConfiguration("robot_id").perform(context).strip()
    sensor_id = LaunchConfiguration("sensor_id").perform(context).strip()
    mount.assert_compatible(
        contract,
        robot_id=robot_id,
        sensor_id=sensor_id,
        require_validated=True,
    )
    internal = SensorInternalCalibration.load(
        Path(LaunchConfiguration("internal_calibration").perform(context)).resolve(),
        require_validated=True,
    )

    # T_base_imu = T_base_lidar * T_lidar_imu. FAST-LIO estimates T_odom_imu;
    # its publisher applies the inverse of this fixed transform so /Odometry
    # truthfully represents T_odom_base instead of relabeling the tilted IMU.
    base_to_imu = mount.transform.compose(internal.transform)
    config_override = LaunchConfiguration("config").perform(context).strip()
    if config_override:
        config_file = os.path.abspath(config_override)
    else:
        config_file = os.path.join(
            LaunchConfiguration("config_path").perform(context),
            LaunchConfiguration("config_file").perform(context),
        )
    if not os.path.isfile(config_file):
        raise RuntimeError("FAST-LIO config is not a regular file: %s" % config_file)

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        parameters=[
            config_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "output.frame_mode": "base_navigation",
                "output.require_base_frame_calibration": True,
                "output.base_to_imu_translation": list(
                    base_to_imu.translation_xyz_m
                ),
                "output.base_to_imu_rotation_xyzw": list(
                    base_to_imu.rotation_xyzw
                ),
                "output.mount_calibration_sha256": mount.digest,
                "output.internal_calibration_sha256": internal.digest,
            },
        ],
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_cfg")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return [fast_lio_node, rviz_node]


def generate_launch_description():
    package_path = get_package_share_directory("fast_lio")
    site_ops_path = get_package_share_directory("go2_site_ops")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "config_path",
                default_value=os.path.join(package_path, "config"),
            ),
            DeclareLaunchArgument("config_file", default_value="mid360.yaml"),
            DeclareLaunchArgument(
                "config",
                default_value="",
                description="Optional absolute YAML path overriding config_path/config_file",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_cfg",
                default_value=os.path.join(package_path, "rviz", "fastlio.rviz"),
            ),
            DeclareLaunchArgument(
                "coordinate_contract",
                default_value=os.path.join(
                    site_ops_path, "config", "coordinate_contract.json"
                ),
            ),
            DeclareLaunchArgument(
                "mount_calibration",
                default_value=EnvironmentVariable(
                    "GO2_MOUNT_CALIBRATION", default_value=""
                ),
            ),
            DeclareLaunchArgument(
                "internal_calibration",
                default_value=os.path.join(
                    site_ops_path, "config", "mid360_internal_calibration.json"
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
            OpaqueFunction(function=_build_runtime),
        ]
    )
