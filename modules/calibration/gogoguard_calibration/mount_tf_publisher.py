"""Publish a production-gated base_link -> lidar_link static transform."""

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .calibration import MountCalibration
from .coordinate_contract import CoordinateContract, ContractError
from .internal_calibration import SensorInternalCalibration
from .release_calibration import ReleaseCalibrationBundle


class MountTfPublisher(Node):
    def __init__(self):
        super().__init__("mount_tf_publisher")
        config_root = Path(__file__).resolve().parent / "config"
        default_contract = str(config_root / "coordinate_contract.json")
        default_internal = str(config_root / "mid360_internal_calibration.json")
        self.declare_parameter("contract_path", default_contract)
        self.declare_parameter("release_calibration_bundle_path", "")
        self.declare_parameter("calibration_path", "")
        self.declare_parameter("internal_calibration_path", default_internal)
        self.declare_parameter("robot_id", "")
        self.declare_parameter("sensor_id", "")

        contract_path = Path(str(self.get_parameter("contract_path").value))
        bundle_value = str(
            self.get_parameter("release_calibration_bundle_path").value
        )
        calibration_value = str(self.get_parameter("calibration_path").value)
        robot_id = str(self.get_parameter("robot_id").value)
        sensor_id = str(self.get_parameter("sensor_id").value)
        if not robot_id or not sensor_id:
            raise ContractError("robot_id and sensor_id are required")
        if bundle_value:
            try:
                payload = json.loads(Path(bundle_value).read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise ContractError("cannot read release calibration bundle: %s" % exc)
            bundle = ReleaseCalibrationBundle.from_dict(payload)
            if bundle.robot_id != robot_id:
                raise ContractError("active map calibration belongs to another robot")
            if bundle.sensor_id != sensor_id:
                raise ContractError("active map calibration belongs to another sensor")
            self.contract = bundle.coordinate_contract
            self.calibration = bundle.mount_calibration
            self.internal_calibration = bundle.sensor_internal_calibration
            self.release_calibration_hash = bundle.digest
        else:
            if not calibration_value:
                raise ContractError(
                    "release_calibration_bundle_path is required for production; "
                    "calibration_path is only an engineering fallback"
                )
            self.contract = CoordinateContract.load(contract_path)
            self.calibration = MountCalibration.load(
                Path(calibration_value),
                require_validated=True,
            )
            self.calibration.assert_compatible(
                self.contract,
                robot_id=robot_id,
                sensor_id=sensor_id,
                require_validated=True,
            )
            self.internal_calibration = SensorInternalCalibration.load(
                Path(str(self.get_parameter("internal_calibration_path").value)),
                require_validated=True,
            )
            self.release_calibration_hash = None

        def transform_message(transform):
            message = TransformStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = transform.parent_frame
            message.child_frame_id = transform.child_frame
            message.transform.translation.x = transform.translation_xyz_m[0]
            message.transform.translation.y = transform.translation_xyz_m[1]
            message.transform.translation.z = transform.translation_xyz_m[2]
            message.transform.rotation.x = transform.rotation_xyzw[0]
            message.transform.rotation.y = transform.rotation_xyzw[1]
            message.transform.rotation.z = transform.rotation_xyzw[2]
            message.transform.rotation.w = transform.rotation_xyzw[3]
            return message

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(
            [
                transform_message(self.calibration.transform),
                transform_message(self.internal_calibration.transform),
            ]
        )
        self.status_publisher = self.create_publisher(
            String,
            "/navigation/mount_calibration_status",
            1,
        )
        self.status_message = String()
        self.status_message.data = json.dumps(
            {
                "ready": True,
                "contractId": self.contract.contract_id,
                "contractSha256": self.contract.digest,
                "calibration": self.calibration.summary(),
                "internalCalibration": {
                    "calibrationId": self.internal_calibration.calibration_id,
                    "sha256": self.internal_calibration.digest,
                    "transform": self.internal_calibration.transform.to_dict(),
                },
                "releaseCalibrationBundleSha256": self.release_calibration_hash,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.timer = self.create_timer(1.0, self.publish_status)
        self.publish_status()
        transform = self.calibration.transform
        self.get_logger().info(
            "validated coordinate chain active: %s (%s -> %s -> lidar_imu)"
            % (
                self.calibration.calibration_id,
                transform.parent_frame,
                transform.child_frame,
            )
        )

    def publish_status(self):
        self.status_publisher.publish(self.status_message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MountTfPublisher()
        rclpy.spin(node)
    except ContractError as exc:
        if node is not None:
            node.get_logger().fatal("mount calibration rejected: %s" % exc)
        else:
            print("mount calibration rejected: %s" % exc)
        raise SystemExit(2)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
