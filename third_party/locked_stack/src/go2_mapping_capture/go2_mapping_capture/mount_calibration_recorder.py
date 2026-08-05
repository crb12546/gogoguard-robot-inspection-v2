"""Record independent Go2-body and LiDAR trajectories without commanding motion."""

from __future__ import annotations

import hashlib
import json
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import SportModeState


SCHEMA = "go2.mount_calibration_capture.v1"


def _finite_list(values: Iterable[float], count: int, label: str) -> list[float]:
    parsed = [float(value) for value in list(values)[:count]]
    if len(parsed) != count or not all(math.isfinite(value) for value in parsed):
        raise ValueError("%s must contain %d finite values" % (label, count))
    return parsed


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class MountCalibrationRecorder(Node):
    """A passive recorder; this node has subscriptions and no publishers."""

    def __init__(self) -> None:
        super().__init__("mount_calibration_recorder")
        self.declare_parameter("robot_id", "")
        self.declare_parameter("sensor_id", "")
        self.declare_parameter("capture_id", "")
        self.declare_parameter("recording_root", "/data/go2/mount-calibration")
        self.declare_parameter("recording_enabled", False)

        self.robot_id = str(self.get_parameter("robot_id").value).strip()
        self.sensor_id = str(self.get_parameter("sensor_id").value).strip()
        capture_id = str(self.get_parameter("capture_id").value).strip()
        if not capture_id:
            capture_id = datetime.now(timezone.utc).strftime("mount-%Y%m%dT%H%M%SZ")
        if not self.robot_id or not self.sensor_id:
            raise RuntimeError("robot_id and sensor_id are required")
        if not capture_id.replace("-", "").replace("_", "").isalnum():
            raise RuntimeError("capture_id may contain only letters, digits, - and _")

        root = Path(str(self.get_parameter("recording_root").value)).resolve()
        root.mkdir(parents=True, exist_ok=True)
        self.final_path = root / (capture_id + ".jsonl")
        self.partial_path = root / (capture_id + ".inprogress.jsonl")
        if self.final_path.exists() or self.partial_path.exists():
            raise RuntimeError("capture already exists: %s" % capture_id)
        self.stream = self.partial_path.open("x", encoding="utf-8", buffering=1)
        self.capture_id = capture_id
        self.counts = {"body_pose": 0, "lidar_pose": 0, "rejected": 0}
        self.closed = False
        self.recording_enabled = bool(
            self.get_parameter("recording_enabled").value
        )
        self.ever_armed = self.recording_enabled
        self.recording_started = (
            self._host_clock() if self.recording_enabled else None
        )
        self.recording_finished = None
        self._write(
            {
                "recordType": "metadata",
                "schema": SCHEMA,
                "captureId": capture_id,
                "robotId": self.robot_id,
                "sensorId": self.sensor_id,
                "createdAt": _utc_now(),
                "bodyTopic": "/lf/sportmodestate",
                "lidarTopic": "/calibration/lidar_odometry",
                "bodyPoseSource": "SportModeState.position plus imu_state.rpy",
                "clockPolicy": "retain source and host receive clocks; solve affine alignment offline",
                "recordingPolicy": (
                    "subscriptions remain passive during sensor preflight; samples are "
                    "written only inside one explicitly armed recording window"
                ),
                "motionCommandCapability": False,
            }
        )

        self.add_on_set_parameters_callback(self._recording_parameter_callback)

        self.create_subscription(
            SportModeState,
            "/lf/sportmodestate",
            self._body_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/calibration/lidar_odometry",
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().warn(
            "Passive mount-calibration recorder ready at %s armed=%s; this node cannot command motion"
            % (self.partial_path, self.recording_enabled)
        )

    def _host_clock(self) -> dict:
        return {
            "receiveEpochNs": time.time_ns(),
            "receiveMonotonicNs": time.monotonic_ns(),
        }

    def _write(self, payload: dict) -> None:
        self.stream.write(
            json.dumps(payload, ensure_ascii=False, sort_keys=True, allow_nan=False)
            + "\n"
        )

    def _recording_parameter_callback(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name != "recording_enabled":
                return SetParametersResult(
                    successful=False,
                    reason="only recording_enabled may change after startup",
                )
            requested = bool(parameter.value)
            if requested and self.recording_finished is not None:
                return SetParametersResult(
                    successful=False,
                    reason="a sealed recording window cannot be re-armed",
                )
            if requested and not self.recording_enabled:
                self.recording_started = self._host_clock()
                self.recording_enabled = True
                self.ever_armed = True
                self.get_logger().warn(
                    "Passive mount-calibration recording ARMED; operator-controlled motion may now be recorded"
                )
            elif not requested and self.recording_enabled:
                self.recording_finished = self._host_clock()
                self.recording_enabled = False
                self.get_logger().info(
                    "Passive mount-calibration recording DISARMED"
                )
        return SetParametersResult(successful=True)

    def _reject(self, source: str, reason: str) -> None:
        self.counts["rejected"] += 1
        if self.counts["rejected"] <= 10:
            self.get_logger().error("Rejected %s sample: %s" % (source, reason))

    def _body_callback(self, message: SportModeState) -> None:
        if not self.recording_enabled:
            return
        try:
            source_stamp_ns = _stamp_ns(message.stamp)
            payload = {
                "recordType": "body_pose",
                "sourceStampNs": source_stamp_ns,
                **self._host_clock(),
                "positionM": _finite_list(message.position, 3, "body position"),
                "rpyRad": _finite_list(message.imu_state.rpy, 3, "body rpy"),
                "gyroscopeRadS": _finite_list(
                    message.imu_state.gyroscope, 3, "body gyroscope"
                ),
                "velocityMS": _finite_list(message.velocity, 3, "body velocity"),
                "yawSpeedRadS": float(message.yaw_speed),
                "bodyHeightM": float(message.body_height),
                "gaitType": int(message.gait_type),
                "progress": float(message.progress),
                "errorCode": int(message.error_code),
                "mode": int(message.mode),
            }
            if source_stamp_ns <= 0:
                raise ValueError("body source timestamp is zero")
            if not math.isfinite(payload["yawSpeedRadS"]):
                raise ValueError("body yaw speed is not finite")
            if not math.isfinite(payload["bodyHeightM"]):
                raise ValueError("body height is not finite")
            if not math.isfinite(payload["progress"]):
                raise ValueError("body progress is not finite")
            self._write(payload)
            self.counts["body_pose"] += 1
        except (AttributeError, TypeError, ValueError) as exc:
            self._reject("body", str(exc))

    def _lidar_callback(self, message: Odometry) -> None:
        if not self.recording_enabled:
            return
        try:
            if message.header.frame_id != "calibration_odom":
                raise ValueError("unexpected parent frame %r" % message.header.frame_id)
            if message.child_frame_id != "lidar_link":
                raise ValueError("unexpected child frame %r" % message.child_frame_id)
            source_stamp_ns = _stamp_ns(message.header.stamp)
            pose = message.pose.pose
            payload = {
                "recordType": "lidar_pose",
                "sourceStampNs": source_stamp_ns,
                **self._host_clock(),
                "positionM": _finite_list(
                    [pose.position.x, pose.position.y, pose.position.z],
                    3,
                    "lidar position",
                ),
                "quaternionXyzw": _finite_list(
                    [
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ],
                    4,
                    "lidar quaternion",
                ),
                "linearVelocityMS": _finite_list(
                    [
                        message.twist.twist.linear.x,
                        message.twist.twist.linear.y,
                        message.twist.twist.linear.z,
                    ],
                    3,
                    "lidar velocity",
                ),
                "angularVelocityRadS": _finite_list(
                    [
                        message.twist.twist.angular.x,
                        message.twist.twist.angular.y,
                        message.twist.twist.angular.z,
                    ],
                    3,
                    "lidar angular velocity",
                ),
            }
            if source_stamp_ns <= 0:
                raise ValueError("lidar source timestamp is zero")
            quaternion_norm = math.sqrt(
                sum(value * value for value in payload["quaternionXyzw"])
            )
            if abs(quaternion_norm - 1.0) > 1.0e-3:
                raise ValueError("lidar quaternion is not normalized")
            self._write(payload)
            self.counts["lidar_pose"] += 1
        except (AttributeError, TypeError, ValueError) as exc:
            self._reject("lidar", str(exc))

    def close_capture(self) -> None:
        if self.closed:
            return
        self.closed = True
        if self.recording_enabled:
            self.recording_finished = self._host_clock()
            self.recording_enabled = False
        self._write(
            {
                "recordType": "summary",
                "closedAt": _utc_now(),
                "counts": dict(self.counts),
                "recordingWindow": {
                    "armed": self.ever_armed,
                    "started": self.recording_started,
                    "finished": self.recording_finished,
                },
            }
        )
        self.stream.flush()
        os.fsync(self.stream.fileno())
        self.stream.close()
        if not self.ever_armed:
            self.get_logger().warn(
                "Recorder closed before arming; leaving audit-only inprogress file: %s"
                % self.partial_path
            )
            return
        os.replace(self.partial_path, self.final_path)
        digest = hashlib.sha256(self.final_path.read_bytes()).hexdigest()
        self.get_logger().info(
            "Mount-calibration capture closed: %s sha256=%s counts=%s"
            % (self.final_path, digest, self.counts)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MountCalibrationRecorder()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.close_capture()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
