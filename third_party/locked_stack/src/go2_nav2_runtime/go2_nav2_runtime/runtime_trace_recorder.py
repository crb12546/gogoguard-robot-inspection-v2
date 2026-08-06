#!/usr/bin/env python3
"""Persist a bounded production flight trace for the warm patrol runtime."""

from __future__ import annotations

import json
import math
import os
import shutil
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from unitree_go.msg import SportModeState


TRACE_SCHEMA = "go2.runtime_trace.v1"
TRACE_STATUS_SCHEMA = "go2.runtime_trace_status.v1"


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _yaw(quaternion) -> float:
    siny = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


class RuntimeTraceRecorder(Node):
    """Record every gate and velocity boundary without storing raw point data."""

    def __init__(self):
        super().__init__("runtime_trace_recorder")
        self.declare_parameter("output_dir", "/data/go2/runtime_logs")
        self.declare_parameter("site_id", "")
        self.declare_parameter("map_version", "")
        self.declare_parameter("manifest_hash", "")
        self.declare_parameter("route_hash", "")
        self.declare_parameter("runtime_profile_hash", "")
        self.declare_parameter("nav2_profile_hash", "")
        self.declare_parameter("calibration_hash", "")
        self.declare_parameter("robot_id", "")
        self.declare_parameter("sensor_id", "")
        self.declare_parameter("localization_quality_profile_id", "")
        self.declare_parameter("runtime_source", "active")
        self.declare_parameter("max_file_bytes", 67108864)
        self.declare_parameter("retained_files", 24)
        self.declare_parameter("minimum_free_bytes", 536870912)

        self.output_dir = Path(
            str(self.get_parameter("output_dir").value)
        ).resolve()
        self.max_file_bytes = int(
            self.get_parameter("max_file_bytes").value
        )
        self.retained_files = int(
            self.get_parameter("retained_files").value
        )
        self.minimum_free_bytes = int(
            self.get_parameter("minimum_free_bytes").value
        )
        if not 1048576 <= self.max_file_bytes <= 1073741824:
            raise RuntimeError("max_file_bytes must be within [1 MiB, 1 GiB]")
        if not 2 <= self.retained_files <= 256:
            raise RuntimeError("retained_files must be within [2, 256]")
        if not 67108864 <= self.minimum_free_bytes <= 10737418240:
            raise RuntimeError(
                "minimum_free_bytes must be within [64 MiB, 10 GiB]"
            )

        self.binding = {
            "siteId": str(self.get_parameter("site_id").value),
            "mapVersion": str(self.get_parameter("map_version").value),
            "manifestHash": str(self.get_parameter("manifest_hash").value),
            "routeHash": str(self.get_parameter("route_hash").value),
            "runtimeProfileHash": str(
                self.get_parameter("runtime_profile_hash").value
            ),
            "nav2ProfileHash": str(
                self.get_parameter("nav2_profile_hash").value
            ),
            "controllerProfileId": "go2-nav2-mppi-omni-v1",
            "collisionProfileId": "go2-mid360-collision-v1",
            "calibrationHash": str(
                self.get_parameter("calibration_hash").value
            ),
            "robotId": str(self.get_parameter("robot_id").value),
            "sensorId": str(self.get_parameter("sensor_id").value),
            "localizationQualityProfileId": str(
                self.get_parameter("localization_quality_profile_id").value
            ),
            "runtimeSource": str(
                self.get_parameter("runtime_source").value
            ),
        }
        if (
            not self.binding["siteId"]
            or not self.binding["mapVersion"]
            or not self.binding["localizationQualityProfileId"]
        ):
            raise RuntimeError(
                "runtime trace requires site, map and localization quality identity"
            )

        self.output_dir.mkdir(parents=True, exist_ok=True)
        if self.output_dir.is_symlink() or not self.output_dir.is_dir():
            raise RuntimeError("runtime trace output must be a real directory")
        self.base_name = "%s-%d" % (
            datetime.now().astimezone().strftime("%Y%m%d-%H%M%S"),
            os.getpid(),
        )
        self.part = 0
        self.path = None
        self._trace_handle = None
        self.bytes_written = 0
        self.records_written = 0
        self.last_fsync_mono = 0.0
        self.last_fsync_wall = None
        self.last_error = ""
        self.writable = False
        self.last_written = {}
        self.last_signature = {}
        self.motion_authorized = False
        self._open_next_file()
        self._write(
            "recorder",
            {
                "event": "start",
                "schema": TRACE_SCHEMA,
                "binding": self.binding,
                "path": str(self.path),
            },
        )

        self.status_publisher = self.create_publisher(
            String, "/go2/runtime/trace_status", 10
        )
        self.create_subscription(
            String, "/localization/status", self._localization, 10
        )
        self.create_subscription(
            String, "/go2/runtime/status", self._runtime, 10
        )
        self.create_subscription(
            String, "/go2/safety/status", self._safety, 10
        )
        self.create_subscription(
            Bool,
            "/go2/runtime/motion_authorized",
            self._authorization,
            10,
        )
        for topic, stream in (
            ("/nav2/raw_cmd_vel", "nav2_raw_cmd"),
            ("/nav2/smoothed_cmd_vel", "nav2_smoothed_cmd"),
            ("/patrol_cmd", "collision_monitored_cmd"),
            ("/cmd_vel", "final_cmd"),
        ):
            self.create_subscription(
                Twist,
                topic,
                lambda message, label=stream: self._command(label, message),
                10,
            )
        self.create_subscription(
            Odometry, "/Odometry", self._odometry, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            "/navigation/cloud_body",
            self._cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SportModeState,
            "/lf/sportmodestate",
            self._robot_state,
            qos_profile_sensor_data,
        )
        # Motion authorization consumes this as a production evidence gate.
        # Publish at 10 Hz so five consecutive missed beats are required
        # before the manager's default 0.50 s timeout revokes authorization.
        self.create_timer(0.10, self._publish_trace_status)
        self.get_logger().info("runtime trace: %s" % self.path)

    def _trace_files(self):
        return sorted(
            self.output_dir.glob("runtime-*.jsonl"),
            key=lambda path: (path.stat().st_mtime_ns, path.name),
        )

    def _prune(self):
        files = self._trace_files()
        while len(files) >= self.retained_files:
            candidate = files.pop(0)
            if self.path is not None and candidate == self.path:
                continue
            candidate.unlink(missing_ok=True)

    def _open_next_file(self):
        self._close_handle()
        self._prune()
        free = shutil.disk_usage(self.output_dir).free
        if free < self.minimum_free_bytes:
            self.writable = False
            self.last_error = "runtime_log_disk_reserve"
            return
        self.part += 1
        self.path = self.output_dir / (
            "runtime-%s-part%03d.jsonl" % (self.base_name, self.part)
        )
        try:
            descriptor = os.open(
                self.path,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_APPEND,
                0o600,
            )
            self._trace_handle = os.fdopen(
                descriptor, "a", buffering=1, encoding="utf-8"
            )
            self.bytes_written = 0
            self.writable = True
            self.last_error = ""
        except OSError as exc:
            self._trace_handle = None
            self.writable = False
            self.last_error = "runtime_log_open:%s" % exc

    def _close_handle(self):
        if self._trace_handle is None or self._trace_handle.closed:
            return
        try:
            self._trace_handle.flush()
            os.fsync(self._trace_handle.fileno())
        finally:
            self._trace_handle.close()
            self._trace_handle = None

    def _write(self, stream, payload):
        now_mono = time.monotonic()
        record = {
            "schema": TRACE_SCHEMA,
            "wallTime": time.time(),
            "monotonicS": now_mono,
            "stream": stream,
            "payload": _json_safe(payload),
        }
        encoded = (
            json.dumps(
                record,
                ensure_ascii=False,
                allow_nan=False,
                separators=(",", ":"),
            )
            + "\n"
        )
        encoded_size = len(encoded.encode("utf-8"))
        if self._trace_handle is None or self._trace_handle.closed:
            self._open_next_file()
        if self._trace_handle is None:
            return
        if self.bytes_written + encoded_size > self.max_file_bytes:
            self._open_next_file()
        if self._trace_handle is None:
            return
        try:
            self._trace_handle.write(encoded)
            self.bytes_written += encoded_size
            self.records_written += 1
            if now_mono - self.last_fsync_mono >= 1.0:
                self._trace_handle.flush()
                os.fsync(self._trace_handle.fileno())
                self.last_fsync_mono = now_mono
                self.last_fsync_wall = time.time()
                self.writable = True
                self.last_error = ""
        except (OSError, ValueError) as exc:
            self.writable = False
            self.last_error = "runtime_log_write:%s" % exc
            self._close_handle()

    def _write_bounded(self, stream, signature, payload, period_s):
        now = time.monotonic()
        if (
            signature == self.last_signature.get(stream)
            and now - self.last_written.get(stream, 0.0) < period_s
        ):
            return
        self.last_signature[stream] = signature
        self.last_written[stream] = now
        self._write(stream, payload)

    @staticmethod
    def _json_payload(message):
        try:
            payload = json.loads(message.data)
            return payload if isinstance(payload, dict) else {"value": payload}
        except (TypeError, ValueError):
            return {"invalidJson": True, "raw": str(message.data)[:2048]}

    def _localization(self, message):
        payload = self._json_payload(message)
        signature = (
            payload.get("state"),
            payload.get("reason"),
            payload.get("usable"),
            payload.get("searchMode"),
        )
        self._write_bounded("localization", signature, payload, 0.4)

    def _runtime(self, message):
        payload = self._json_payload(message)
        signature = (
            payload.get("runtimeInstanceId"),
            payload.get("state"),
            payload.get("reason"),
            payload.get("motionAuthorized"),
            (payload.get("startTiming") or {}).get("stage")
            if isinstance(payload.get("startTiming"), dict)
            else None,
        )
        self._write_bounded("runtime", signature, payload, 0.5)

    def _safety(self, message):
        payload = self._json_payload(message)
        signature = (
            payload.get("stopReason"),
            payload.get("authorized"),
            payload.get("localizationUsable"),
            payload.get("commandFresh"),
            payload.get("cloudFresh"),
            payload.get("obstacleDetected"),
            payload.get("cloudValidityReason"),
        )
        self._write_bounded("safety", signature, payload, 0.2)

    def _authorization(self, message):
        self.motion_authorized = message.data is True
        payload = {"authorized": self.motion_authorized}
        self._write_bounded(
            "authorization", self.motion_authorized, payload, 0.5
        )

    @staticmethod
    def _twist_payload(message):
        return {
            "vx": float(message.linear.x),
            "vy": float(message.linear.y),
            "vz": float(message.linear.z),
            "wx": float(message.angular.x),
            "wy": float(message.angular.y),
            "wz": float(message.angular.z),
        }

    def _command(self, stream, message):
        payload = self._twist_payload(message)
        signature = tuple(
            round(payload[name], 3) for name in ("vx", "vy", "wz")
        )
        self._write_bounded(stream, signature, payload, 0.1)

    def _odometry(self, message):
        pose = message.pose.pose
        twist = message.twist.twist
        payload = {
            "stamp": {
                "sec": int(message.header.stamp.sec),
                "nanosec": int(message.header.stamp.nanosec),
            },
            "frameId": message.header.frame_id,
            "childFrameId": message.child_frame_id,
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "yaw": _yaw(pose.orientation),
            "vx": float(twist.linear.x),
            "vy": float(twist.linear.y),
            "wz": float(twist.angular.z),
        }
        signature = tuple(
            round(payload[name], 3)
            for name in ("x", "y", "yaw", "vx", "vy", "wz")
        )
        self._write_bounded("odometry", signature, payload, 0.2)

    def _cloud(self, message):
        payload = {
            "stamp": {
                "sec": int(message.header.stamp.sec),
                "nanosec": int(message.header.stamp.nanosec),
            },
            "frameId": message.header.frame_id,
            "width": int(message.width),
            "height": int(message.height),
            "pointStep": int(message.point_step),
            "rowStep": int(message.row_step),
            "dataBytes": len(message.data),
            "isDense": bool(message.is_dense),
            "fields": [field.name for field in message.fields],
        }
        signature = (
            payload["frameId"],
            payload["width"],
            payload["height"],
            payload["pointStep"],
            payload["dataBytes"],
        )
        self._write_bounded("body_cloud", signature, payload, 0.5)

    def _robot_state(self, message):
        try:
            payload = {
                "errorCode": int(message.error_code),
                "mode": int(message.mode),
                "progress": float(message.progress),
                "gaitType": int(message.gait_type),
                "vx": float(message.velocity[0]),
                "vy": float(message.velocity[1]),
                "yawSpeed": float(message.yaw_speed),
            }
        except (AttributeError, IndexError, TypeError, ValueError):
            payload = {"invalid": True}
        signature = tuple(payload.get(name) for name in sorted(payload))
        self._write_bounded("unitree_state", signature, payload, 0.5)

    def _publish_trace_status(self):
        free = shutil.disk_usage(self.output_dir).free
        status = {
            "schema": TRACE_STATUS_SCHEMA,
            "publishedAt": time.time(),
            "writable": bool(self.writable and self._trace_handle is not None),
            "path": str(self.path) if self.path is not None else None,
            "recordsWritten": self.records_written,
            "bytesWritten": self.bytes_written,
            "lastFsyncAt": self.last_fsync_wall,
            "lastFsyncAgeS": (
                max(0.0, time.monotonic() - self.last_fsync_mono)
                if self.last_fsync_mono > 0.0
                else None
            ),
            "freeBytes": free,
            "minimumFreeBytes": self.minimum_free_bytes,
            "error": self.last_error or None,
            "binding": self.binding,
        }
        message = String()
        message.data = json.dumps(
            status, ensure_ascii=False, allow_nan=False, separators=(",", ":")
        )
        self.status_publisher.publish(message)

    def close(self):
        if self._trace_handle is not None and not self._trace_handle.closed:
            self._write("recorder", {"event": "stop"})
        self._close_handle()


def main(args=None):
    rclpy.init(args=args)
    node = RuntimeTraceRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
