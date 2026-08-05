from __future__ import annotations

import math
import shutil
import threading
import time
from collections import deque
from copy import deepcopy
from pathlib import Path
from typing import Protocol

from gogoguard_contracts import DeviceStatus, SensorSnapshot, utc_now


class SensorGateway(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...


class SnapshotStore:
    def __init__(self, robot_id: str, mode: str, data_root: Path) -> None:
        self._lock = threading.Lock()
        self._snapshot = SensorSnapshot(device=DeviceStatus(robot_id=robot_id, mode=mode))
        self._data_root = data_root

    def replace(self, snapshot: SensorSnapshot) -> None:
        snapshot.device.disk_free_bytes = shutil.disk_usage(self._data_root).free
        with self._lock:
            self._snapshot = deepcopy(snapshot)

    def get(self) -> SensorSnapshot:
        with self._lock:
            return deepcopy(self._snapshot)


class DemoSensorGateway:
    """Deterministic room scan used to prove the full UI/record/map loop."""

    def __init__(self, store: SnapshotStore, robot_id: str) -> None:
        self.store = store
        self.robot_id = robot_id
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="demo-sensors", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    @staticmethod
    def _cloud(phase: float) -> list[list[float]]:
        points: list[list[float]] = []
        for index in range(260):
            angle = (index / 260.0) * math.tau
            distance = 4.0 / max(abs(math.cos(angle)), abs(math.sin(angle)), 0.2)
            distance += 0.08 * math.sin(index * 1.7 + phase)
            points.append([
                distance * math.cos(angle),
                distance * math.sin(angle),
                0.25 + (index % 18) * 0.11,
                0.5 + 0.5 * math.sin(angle * 3 + phase),
            ])
        for index in range(80):
            angle = (index / 80.0) * math.tau
            points.append([1.1 + 0.55 * math.cos(angle), -0.7 + 0.55 * math.sin(angle), (index % 12) * 0.14, 0.9])
        return points

    def _run(self) -> None:
        trajectory: deque[list[float]] = deque(maxlen=900)
        sequence = 0
        started = time.monotonic()
        while not self._stop.is_set():
            elapsed = time.monotonic() - started
            x = 2.2 * math.sin(elapsed * 0.18)
            y = 1.7 * math.sin(elapsed * 0.36) / 2
            yaw = math.atan2(math.cos(elapsed * 0.36), math.cos(elapsed * 0.18) + 0.01)
            trajectory.append([x, y, 0.0])
            device = DeviceStatus(
                robot_id=self.robot_id,
                mode="demo",
                online=True,
                lidar_hz=10.0,
                imu_hz=200.0,
                odometry_hz=20.0,
                lidar_age_s=0.0,
                imu_age_s=0.0,
                message="模拟传感器数据；尚未连接机器狗",
                observed_at=utc_now(),
            )
            self.store.replace(SensorSnapshot(
                sequence=sequence,
                captured_at=utc_now(),
                pose={"x": x, "y": y, "z": 0.0, "yaw": yaw},
                trajectory=list(trajectory),
                points=self._cloud(elapsed),
                device=device,
            ))
            sequence += 1
            self._stop.wait(0.1)


class Ros2SensorGateway:
    """ROS 2 boundary. ROS-specific types never leak into product modules."""

    def __init__(self, store: SnapshotStore, robot_id: str, topics: dict[str, str]) -> None:
        self.store = store
        self.robot_id = robot_id
        self.topics = topics
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._error: str | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="ros2-sensors", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    def _run(self) -> None:
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu, PointCloud2
            from sensor_msgs_py import point_cloud2
        except ImportError as exc:
            self._error = f"ROS 2 Python 依赖不可用: {exc}"
            self._publish_error()
            return

        trajectory: deque[list[float]] = deque(maxlen=1800)
        points: list[list[float]] = []
        pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        sequence = 0
        counts = {"lidar": 0, "imu": 0, "odom": 0}
        last_seen = {"lidar": None, "imu": None, "odom": None}
        rates = {"lidar": 0.0, "imu": 0.0, "odom": 0.0}
        window_started = time.monotonic()

        rclpy.init(args=None)
        node = rclpy.create_node("gogoguard_sensor_gateway")

        def lidar_callback(message: PointCloud2) -> None:
            nonlocal points
            values = point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
            sampled: list[list[float]] = []
            for index, value in enumerate(values):
                if index % 8 == 0:
                    sampled.append([float(value[0]), float(value[1]), float(value[2]), 0.7])
                if len(sampled) >= 1200:
                    break
            points = sampled
            counts["lidar"] += 1
            last_seen["lidar"] = time.monotonic()

        def imu_callback(_: Imu) -> None:
            counts["imu"] += 1
            last_seen["imu"] = time.monotonic()

        def odom_callback(message: Odometry) -> None:
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            pose.update(x=float(position.x), y=float(position.y), z=float(position.z), yaw=math.atan2(
                2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z),
            ))
            trajectory.append([pose["x"], pose["y"], pose["z"]])
            counts["odom"] += 1
            last_seen["odom"] = time.monotonic()

        node.create_subscription(PointCloud2, self.topics["lidar"], lidar_callback, 10)
        node.create_subscription(Imu, self.topics["imu"], imu_callback, 50)
        node.create_subscription(Odometry, self.topics["odometry"], odom_callback, 20)
        try:
            while rclpy.ok() and not self._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                elapsed = now - window_started
                if elapsed >= 1.0:
                    for name in rates:
                        rates[name] = counts[name] / elapsed
                        counts[name] = 0
                    window_started = now
                online = last_seen["lidar"] is not None and now - last_seen["lidar"] < 2.0
                device = DeviceStatus(
                    robot_id=self.robot_id, mode="robot", online=online,
                    lidar_hz=round(rates["lidar"], 1), imu_hz=round(rates["imu"], 1),
                    odometry_hz=round(rates["odom"], 1),
                    lidar_age_s=None if last_seen["lidar"] is None else round(now - last_seen["lidar"], 3),
                    imu_age_s=None if last_seen["imu"] is None else round(now - last_seen["imu"], 3),
                    message="传感器在线" if online else "等待 Livox 点云",
                )
                self.store.replace(SensorSnapshot(sequence=sequence, pose=dict(pose), trajectory=list(trajectory), points=list(points), device=device))
                sequence += 1
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def _publish_error(self) -> None:
        snapshot = self.store.get()
        snapshot.device.online = False
        snapshot.device.message = self._error or "ROS 2 gateway failed"
        self.store.replace(snapshot)


def create_gateway(mode: str, store: SnapshotStore, robot_id: str, topics: dict[str, str]) -> SensorGateway:
    if mode == "demo":
        return DemoSensorGateway(store, robot_id)
    if mode == "robot":
        return Ros2SensorGateway(store, robot_id, topics)
    raise ValueError(f"unsupported runtime mode: {mode}")
