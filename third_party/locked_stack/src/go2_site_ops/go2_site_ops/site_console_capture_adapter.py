#!/usr/bin/env python3
"""Feed live mapping odometry/cloud evidence into the Site Console API."""

from __future__ import annotations

import json
import math
import queue
import re
import shutil
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from .coverage_ingest_core import (
    PassTracker,
    PoseSpeedEstimator,
    ScanQualityEstimator,
    health_state,
    storage_health,
)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _yaw(quaternion) -> float:
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


class ConsolePostError(RuntimeError):
    pass


TOKEN = re.compile(r"^[A-Za-z0-9._~+/=-]{32,512}$")


def _load_console_token(path_text: str) -> str:
    if not path_text:
        raise RuntimeError("console_token_file is required")
    path = Path(path_text)
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise RuntimeError("console_token_file must be an absolute regular non-symlink file")
    if path.stat().st_mode & 0o077:
        raise RuntimeError("console_token_file must not be accessible by group or others")
    token = path.read_text(encoding="utf-8").strip()
    if not TOKEN.fullmatch(token):
        raise RuntimeError("console token must be 32-512 printable token characters")
    return token


class ConsolePostClient:
    def __init__(self, base_url: str, token: str, timeout_s: float):
        base = str(base_url).strip().rstrip("/")
        if not (base.startswith("http://") or base.startswith("https://")):
            raise ConsolePostError("console_base_url must be HTTP(S)")
        if timeout_s <= 0.0:
            raise ConsolePostError("HTTP timeout must be positive")
        if not TOKEN.fullmatch(str(token).strip()):
            raise ConsolePostError("one valid console bearer token is required")
        self.base_url = base
        self.token = str(token).strip()
        self.timeout_s = float(timeout_s)

    def post(self, path: str, payload) -> None:
        data = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")
        headers = {"Content-Type": "application/json; charset=utf-8"}
        headers["Authorization"] = "Bearer " + self.token
        request = urllib.request.Request(
            self.base_url + path,
            data=data,
            headers=headers,
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                if response.status != 200:
                    raise ConsolePostError("console returned HTTP %d" % response.status)
                response.read(4096)
        except urllib.error.HTTPError as exc:
            detail = exc.read(2048).decode("utf-8", errors="replace")
            raise ConsolePostError("console rejected HTTP %d: %s" % (exc.code, detail))
        except (OSError, urllib.error.URLError) as exc:
            raise ConsolePostError("console connection failed: %s" % exc)


class SiteConsoleCaptureAdapter(Node):
    def __init__(self):
        super().__init__("site_console_capture_adapter")
        self.declare_parameter("console_base_url", "http://127.0.0.1:8765")
        self.declare_parameter("console_token_file", "")
        self.declare_parameter("raw_lidar_topic", "/mapping/livox/lidar")
        self.declare_parameter("imu_topic", "/mapping/livox/imu")
        self.declare_parameter("odometry_topic", "/Odometry")
        self.declare_parameter("world_cloud_topic", "/cloud_registered")
        self.declare_parameter("pass_topic", "/mapping/acquisition_pass")
        self.declare_parameter("expected_world_frame", "odom")
        self.declare_parameter("capture_id", "")
        self.declare_parameter("recording_root", "/data/go2/captures")
        self.declare_parameter("scan_publish_rate_hz", 2.0)
        self.declare_parameter("maximum_points", 1200)
        self.declare_parameter("maximum_range_m", 35.0)
        self.declare_parameter("minimum_z_m", -1.5)
        self.declare_parameter("maximum_z_m", 3.0)
        self.declare_parameter("maximum_pose_skew_s", 0.12)
        self.declare_parameter("http_timeout_s", 0.75)
        self.declare_parameter("storage_warning_gb", 20.0)
        self.declare_parameter("storage_fault_gb", 8.0)

        capture_id = str(self.get_parameter("capture_id").value).strip()
        if not capture_id:
            capture_id = time.strftime("capture-%Y%m%d-%H%M%S")
        token_path = str(self.get_parameter("console_token_file").value).strip()
        token = _load_console_token(token_path)
        self.client = ConsolePostClient(
            str(self.get_parameter("console_base_url").value),
            token,
            float(self.get_parameter("http_timeout_s").value),
        )
        self.expected_world_frame = str(
            self.get_parameter("expected_world_frame").value
        ).strip()
        self.recording_root = Path(
            str(self.get_parameter("recording_root").value)
        ).resolve()
        self.scan_period_s = 1.0 / float(
            self.get_parameter("scan_publish_rate_hz").value
        )
        self.maximum_points = int(self.get_parameter("maximum_points").value)
        self.maximum_range_m = float(self.get_parameter("maximum_range_m").value)
        self.minimum_z_m = float(self.get_parameter("minimum_z_m").value)
        self.maximum_z_m = float(self.get_parameter("maximum_z_m").value)
        self.maximum_pose_skew_s = float(
            self.get_parameter("maximum_pose_skew_s").value
        )
        if (
            not self.expected_world_frame
            or self.scan_period_s <= 0.0
            or self.maximum_points < 100
            or self.maximum_range_m <= 0.0
            or self.minimum_z_m >= self.maximum_z_m
            or self.maximum_pose_skew_s <= 0.0
        ):
            raise RuntimeError("invalid capture adapter parameters")
        self.storage_warning_bytes = int(
            float(self.get_parameter("storage_warning_gb").value) * 1024**3
        )
        self.storage_fault_bytes = int(
            float(self.get_parameter("storage_fault_gb").value) * 1024**3
        )
        if self.storage_warning_bytes <= self.storage_fault_bytes:
            raise RuntimeError("storage warning threshold must exceed fault threshold")

        self.quality = ScanQualityEstimator()
        self.pass_tracker = PassTracker(capture_id)
        self.current_pass_id = self.pass_tracker.pass_id
        self.speed_estimator = PoseSpeedEstimator()
        self.latest_pose = None
        self.latest_pose_stamp = 0.0
        self.last_raw_lidar_mono = None
        self.last_imu_mono = None
        self.last_odom_mono = None
        self.last_cloud_mono = None
        self.last_pose_skew_s = None
        self.last_scan_post_mono = 0.0
        self.last_http_success_mono = None
        self.http_attempted = False
        self.work = queue.Queue(maxsize=4)
        self.stop_worker = threading.Event()
        self.worker = threading.Thread(target=self._post_worker, daemon=True)
        self.worker.start()

        self.pass_publisher = self.create_publisher(
            String,
            str(self.get_parameter("pass_topic").value),
            10,
        )

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("raw_lidar_topic").value),
            self._raw_lidar_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._imu_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._odometry_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("world_cloud_topic").value),
            self._world_cloud_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._health_timer)
        self.get_logger().info(
            "guided capture adapter active: console=%s capture=%s"
            % (self.client.base_url, capture_id)
        )

    def _raw_lidar_callback(self, message):
        del message
        self.last_raw_lidar_mono = time.monotonic()
        # Publish once per raw scan. rosbag's receive timestamp binds this
        # marker to the scan without modifying the sensor message or relying
        # on an HTTP log for later cross-pass stability analysis.
        marker = String()
        marker.data = self.current_pass_id
        self.pass_publisher.publish(marker)

    def _imu_callback(self, message):
        del message
        self.last_imu_mono = time.monotonic()

    def _odometry_callback(self, message):
        if (
            message.header.frame_id != "odom"
            or message.child_frame_id != "base_link"
        ):
            self.get_logger().error("rejecting odometry outside odom -> base_link")
            return
        pose = message.pose.pose
        stamp = _stamp_seconds(message.header.stamp)
        speed = self.speed_estimator.update(
            timestamp=stamp,
            x=float(pose.position.x),
            y=float(pose.position.y),
        )
        yaw = _yaw(pose.orientation)
        self.latest_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw,
            float(speed),
        )
        self.current_pass_id = self.pass_tracker.update(
            timestamp=stamp,
            heading_rad=yaw,
            speed_mps=speed,
        )
        self.latest_pose_stamp = stamp
        self.last_odom_mono = time.monotonic()

    def _sample_world_points(self, message, sensor_x, sensor_y):
        total = max(1, int(message.width) * int(message.height))
        stride = max(1, int(math.ceil(total / self.maximum_points)))
        maximum_range_sq = self.maximum_range_m * self.maximum_range_m
        sampled = []
        for index, point in enumerate(
            point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        ):
            if index % stride:
                continue
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not self.minimum_z_m <= z <= self.maximum_z_m:
                continue
            if (x - sensor_x) ** 2 + (y - sensor_y) ** 2 > maximum_range_sq:
                continue
            sampled.append((x, y, z))
            if len(sampled) >= self.maximum_points:
                break
        return sampled

    def _world_cloud_callback(self, message):
        now_mono = time.monotonic()
        self.last_cloud_mono = now_mono
        if now_mono - self.last_scan_post_mono < self.scan_period_s:
            return
        if message.header.frame_id != self.expected_world_frame:
            self.get_logger().error(
                "rejecting guidance cloud frame %s; expected %s"
                % (message.header.frame_id, self.expected_world_frame)
            )
            return
        if self.latest_pose is None:
            return
        stamp = _stamp_seconds(message.header.stamp)
        skew = abs(stamp - self.latest_pose_stamp)
        self.last_pose_skew_s = skew
        if skew > self.maximum_pose_skew_s:
            return
        x, y, yaw, speed = self.latest_pose
        points = self._sample_world_points(message, x, y)
        if len(points) < 50:
            return
        quality = self.quality.estimate(points, sensor_xy=(x, y))
        payload = {
            "sensorX": x,
            "sensorY": y,
            "headingRad": yaw,
            "observedPoints": [[point[0], point[1]] for point in points],
            # Coverage remains a 2D operator-guidance calculation, while this
            # bounded sample preserves z so the field console can show the
            # exact registered LiDAR frame that supported that calculation.
            "previewPoints": [[point[0], point[1], point[2]] for point in points],
            "previewFrame": message.header.frame_id,
            "sourcePointCount": int(message.width) * int(message.height),
            "passId": self.current_pass_id,
            "timestamp": stamp,
            "overlap": quality.overlap,
            "information": quality.information,
            # Dynamic truth is deliberately absent. It is produced later by
            # cross-acquisition stability analysis, not guessed online.
            "speedMps": speed,
        }
        self._enqueue("/api/v1/scan", payload)
        self.last_scan_post_mono = now_mono

    @staticmethod
    def _age(now, last):
        return None if last is None else max(0.0, now - last)

    def _health_timer(self):
        now = time.monotonic()
        self.recording_root.mkdir(parents=True, exist_ok=True)
        free_bytes = shutil.disk_usage(str(self.recording_root)).free
        lidar = health_state(self._age(now, self.last_raw_lidar_mono), 1.0, 3.0)
        imu = health_state(self._age(now, self.last_imu_mono), 1.0, 3.0)
        odom = health_state(self._age(now, self.last_odom_mono), 1.0, 3.0)
        cloud = health_state(self._age(now, self.last_cloud_mono), 1.0, 3.0)
        if self.last_pose_skew_s is None:
            time_sync = "unknown"
        elif self.last_pose_skew_s <= self.maximum_pose_skew_s and odom == "ready" and cloud == "ready":
            time_sync = "ready"
        elif odom == "fault" or cloud == "fault":
            time_sync = "fault"
        else:
            time_sync = "warning"
        if self.last_http_success_mono is not None and now - self.last_http_success_mono <= 3.0:
            network = "ready"
        elif self.http_attempted:
            network = "fault"
        else:
            network = "unknown"
        self._enqueue(
            "/api/v1/health",
            {
                "health": {
                    "lidar": lidar,
                    "imu": imu,
                    "timeSync": time_sync,
                    "storage": storage_health(
                        free_bytes,
                        self.storage_warning_bytes,
                        self.storage_fault_bytes,
                    ),
                    "network": network,
                }
            },
        )

    def _enqueue(self, path, payload):
        item = (path, payload)
        try:
            self.work.put_nowait(item)
        except queue.Full:
            try:
                self.work.get_nowait()
            except queue.Empty:
                pass
            self.work.put_nowait(item)

    def _post_worker(self):
        while not self.stop_worker.is_set():
            try:
                path, payload = self.work.get(timeout=0.2)
            except queue.Empty:
                continue
            self.http_attempted = True
            try:
                self.client.post(path, payload)
                self.last_http_success_mono = time.monotonic()
            except ConsolePostError as exc:
                self.get_logger().warning(str(exc))

    def destroy_node(self):
        self.stop_worker.set()
        self.worker.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SiteConsoleCaptureAdapter()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
