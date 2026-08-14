"""Bounded passive flight recorder for navigation incidents.

The recorder is deliberately outside the motion path.  It serializes ROS
messages into a bounded lightweight ring during every patrol, optionally adds
heavy development streams, and leaves all decisions to navigation. Recorder
health is observable but never authorizes or blocks motion.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import os
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from gogoguard_contracts import DiagnosticMode, IncidentBundle, IncidentState, json_ready
from .incidents import DiagnosticProfileStore, IncidentStore, new_incident_id


ACTIVE_STATES = {
    "STARTING", "LOCALIZING", "PATROLLING", "HOLDING", "RESUMING",
    "REPLANNING", "DETOURING", "REJOINING", "RETRYING", "RECOVERING",
    "SEARCHING_PATH",
}
TRIGGER_STATES = {
    "BLOCKED", "FAULT", "HOLDING", "RECOVERING", "SEARCHING_PATH",
}
TERMINAL_STATES = {
    "IDLE", "STOPPED", "COMPLETE", "COMPLETED", "BLOCKED", "FAULT",
}

LIGHTWEIGHT_PRODUCTION_TOPICS = frozenset(
    {
        "/Odometry",
        "/localization/pose",
        "/localization/status",
        "/go2/runtime/status",
        "/go2/safety/status",
        "/go2/runtime/planner_diagnostics",
        "/go2/runtime/route",
        "/nav2/raw_cmd_vel",
        "/patrol_cmd",
        "/cmd_vel",
    }
)


@dataclass(frozen=True)
class SerializedSample:
    topic: str
    type_name: str
    timestamp_ns: int
    data: bytes


class RollingSamples:
    def __init__(self) -> None:
        self._values: collections.deque[SerializedSample] = collections.deque()
        self._bytes = 0
        self._lock = threading.Lock()

    def append(self, sample: SerializedSample, *, keep_s: float, byte_limit: int) -> None:
        with self._lock:
            self._values.append(sample)
            self._bytes += len(sample.data)
            cutoff = sample.timestamp_ns - int(max(0.0, keep_s) * 1e9)
            while self._values and (
                self._values[0].timestamp_ns < cutoff or self._bytes > byte_limit
            ):
                removed = self._values.popleft()
                self._bytes -= len(removed.data)

    def snapshot(self) -> list[SerializedSample]:
        with self._lock:
            return list(self._values)


class RollingPreview:
    def __init__(self) -> None:
        self._values: collections.deque[dict[str, Any]] = collections.deque()
        self._lock = threading.Lock()

    def append(self, value: dict[str, Any], *, keep_s: float) -> None:
        with self._lock:
            self._values.append(value)
            cutoff = float(value["timestamp"]) - max(0.0, keep_s)
            while self._values and float(self._values[0]["timestamp"]) < cutoff:
                self._values.popleft()

    def snapshot(self) -> list[dict[str, Any]]:
        with self._lock:
            return list(self._values)


class CameraRing:
    """H.264 remux only: no camera decode or encode runs on the robot."""

    def __init__(self, root: Path, source: str) -> None:
        self.root = root
        self.source = source
        self.process: subprocess.Popen | None = None

    def start(self) -> bool:
        if self.process and self.process.poll() is None:
            return True
        if not shutil.which("ffmpeg"):
            return False
        self.root.mkdir(parents=True, exist_ok=True)
        pattern = str(self.root / "camera-%Y%m%dT%H%M%SZ.mp4")
        self.process = subprocess.Popen(
            [
                "ffmpeg", "-nostdin", "-loglevel", "error", "-rtsp_transport", "tcp",
                "-i", self.source, "-map", "0:v:0", "-c:v", "copy", "-an",
                "-f", "segment", "-segment_time", "5", "-reset_timestamps", "1",
                "-strftime", "1", pattern,
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        return True

    def stop(self) -> None:
        if not self.process:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.process = None

    def copy_window(self, destination: Path, start_epoch: float, end_epoch: float) -> list[Path]:
        destination.mkdir(parents=True, exist_ok=True)
        copied = []
        for source in sorted(self.root.glob("camera-*.mp4")):
            modified = source.stat().st_mtime
            # A five-second segment may start before the requested interval.
            if modified < start_epoch - 6.0 or modified > end_epoch + 6.0:
                continue
            target = destination / source.name
            shutil.copy2(source, target)
            copied.append(target)
        return copied

    def prune(self, keep_s: float = 90.0) -> None:
        cutoff = time.time() - keep_s
        for path in self.root.glob("camera-*.mp4"):
            try:
                if path.stat().st_mtime < cutoff:
                    path.unlink()
            except OSError:
                continue


class IncidentRecorder:
    def __init__(
        self,
        *,
        data_root: Path,
        site_id: str,
        robot_id: str,
        sensor_id: str,
        camera_source: str,
    ) -> None:
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        self.rclpy = rclpy
        self.data_root = Path(data_root)
        self.site_id = site_id
        self.robot_id = robot_id
        self.sensor_id = sensor_id
        self.profile_store = DiagnosticProfileStore(self.data_root)
        self.incidents = IncidentStore(self.data_root)
        self.samples = RollingSamples()
        self.preview = RollingPreview()
        self.camera = CameraRing(self.data_root / "diagnostics" / "camera-ring", camera_source)
        self.node = rclpy.create_node("gogoguard_incident_recorder")
        self._last_cloud_preview = 0.0
        self._last_costmap_preview = 0.0
        self._last_runtime_state = ""
        self._patrol_active = False
        self._patrol_profile = None
        self._pending = False
        self._pending_lock = threading.Lock()
        self._current_pose: dict[str, Any] | None = None
        self._current_route: list[list[float]] = []
        self._current_runtime: dict[str, Any] = {}

        subscriptions = [
            (PointCloud2, "/navigation/cloud_body", self._cloud, qos_profile_sensor_data),
            (Odometry, "/Odometry", self._generic("/Odometry", "nav_msgs/msg/Odometry"), qos_profile_sensor_data),
            (TFMessage, "/tf", self._generic("/tf", "tf2_msgs/msg/TFMessage"), 50),
            (TFMessage, "/tf_static", self._generic("/tf_static", "tf2_msgs/msg/TFMessage"), 10),
            (PoseWithCovarianceStamped, "/localization/pose", self._pose, 10),
            (String, "/localization/status", self._text("/localization/status"), 10),
            (String, "/go2/runtime/status", self._runtime, 10),
            (String, "/go2/safety/status", self._text("/go2/safety/status"), 10),
            (String, "/go2/runtime/planner_diagnostics", self._text("/go2/runtime/planner_diagnostics"), 10),
            (NavPath, "/go2/runtime/route", self._route, 10),
            (OccupancyGrid, "/local_costmap/costmap", self._costmap, 10),
            (Twist, "/nav2/raw_cmd_vel", self._generic("/nav2/raw_cmd_vel", "geometry_msgs/msg/Twist"), 20),
            (Twist, "/patrol_cmd", self._generic("/patrol_cmd", "geometry_msgs/msg/Twist"), 20),
            (Twist, "/cmd_vel", self._generic("/cmd_vel", "geometry_msgs/msg/Twist"), 20),
        ]
        self._subscriptions = [
            self.node.create_subscription(message_type, topic, callback, qos)
            for message_type, topic, callback, qos in subscriptions
        ]
        self.node.create_timer(0.5, self._tick)

    def _profile(self):
        return self._patrol_profile or self.profile_store.get()

    def _capture_enabled(self) -> bool:
        return self._patrol_active

    def _store(self, topic: str, type_name: str, message: Any) -> None:
        if not self._capture_enabled() and topic != "/go2/runtime/status":
            return
        from rclpy.serialization import serialize_message

        profile = self._profile()
        if (
            profile.mode == DiagnosticMode.PRODUCTION
            and topic not in LIGHTWEIGHT_PRODUCTION_TOPICS
        ):
            return
        sample = SerializedSample(topic, type_name, time.time_ns(), bytes(serialize_message(message)))
        # Protect the Orin even if a malformed profile or unusually dense cloud appears.
        byte_limit = min(profile.max_storage_bytes // 4, 256 * 1024 * 1024)
        self.samples.append(sample, keep_s=profile.pre_trigger_s, byte_limit=byte_limit)

    def _generic(self, topic: str, type_name: str) -> Callable[[Any], None]:
        return lambda message: self._store(topic, type_name, message)

    def _text(self, topic: str) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            self._store(topic, "std_msgs/msg/String", message)
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError):
                payload = {"raw": str(message.data)}
            self.preview.append(
                {"kind": "decision", "topic": topic, "timestamp": time.time(), "payload": payload},
                keep_s=self._profile().pre_trigger_s,
            )
        return callback

    def _runtime(self, message: Any) -> None:
        self._store("/go2/runtime/status", "std_msgs/msg/String", message)
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = {"raw": str(message.data)}
        self._current_runtime = payload
        state = str(payload.get("state") or "").upper()
        self.preview.append(
            {"kind": "runtime", "topic": "/go2/runtime/status", "timestamp": time.time(), "payload": payload},
            keep_s=self._profile().pre_trigger_s,
        )
        if state in ACTIVE_STATES and self._last_runtime_state not in ACTIVE_STATES:
            # Freeze one profile for the complete patrol. A one-patrol limit is
            # consumed at the terminal state, but its post-trigger window must
            # still use the profile that was active at patrol start.
            self._patrol_profile = self.profile_store.get()
            self._patrol_active = True
            if self._profile().record_camera:
                self.camera.start()
        if state in TRIGGER_STATES and state != self._last_runtime_state:
            self.trigger(state.lower(), payload)
        if state in TERMINAL_STATES and self._last_runtime_state in ACTIVE_STATES:
            self.profile_store.consume_patrol()
            if not self._pending:
                self._patrol_active = False
                self.camera.stop()
                self._patrol_profile = None
        self._last_runtime_state = state

    def _pose(self, message: Any) -> None:
        self._store("/localization/pose", "geometry_msgs/msg/PoseWithCovarianceStamped", message)
        pose = message.pose.pose
        q = pose.orientation
        self._current_pose = {
            "x": float(pose.position.x), "y": float(pose.position.y), "z": float(pose.position.z),
            "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        }
        self.preview.append(
            {"kind": "pose", "timestamp": time.time(), "pose": self._current_pose},
            keep_s=self._profile().pre_trigger_s,
        )

    def _route(self, message: Any) -> None:
        self._store("/go2/runtime/route", "nav_msgs/msg/Path", message)
        self._current_route = [[float(item.pose.position.x), float(item.pose.position.y)] for item in message.poses]
        self.preview.append(
            {"kind": "route", "timestamp": time.time(), "points": self._current_route},
            keep_s=self._profile().pre_trigger_s,
        )

    def _cloud(self, message: Any) -> None:
        profile = self._profile()
        if not self._capture_enabled() or profile.point_cloud_hz <= 0:
            return
        now = time.monotonic()
        if now - self._last_cloud_preview < 1.0 / profile.point_cloud_hz:
            return
        self._last_cloud_preview = now
        self._store("/navigation/cloud_body", "sensor_msgs/msg/PointCloud2", message)
        try:
            from sensor_msgs_py import point_cloud2

            values = point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
            points = []
            stride = max(1, int(message.width) // 1200)
            for index, value in enumerate(values):
                if index % stride == 0:
                    points.append([round(float(value[0]), 3), round(float(value[1]), 3), round(float(value[2]), 3)])
                    if len(points) >= 1200:
                        break
            self.preview.append(
                {"kind": "cloud", "timestamp": time.time(), "frame": message.header.frame_id, "points": points},
                keep_s=profile.pre_trigger_s,
            )
        except Exception as exc:
            self.node.get_logger().warning(f"point-cloud preview unavailable: {exc}")

    def _costmap(self, message: Any) -> None:
        profile = self._profile()
        if not self._capture_enabled() or not profile.record_costmap:
            return
        self._store("/local_costmap/costmap", "nav_msgs/msg/OccupancyGrid", message)
        now = time.monotonic()
        if now - self._last_costmap_preview < 0.5:
            return
        self._last_costmap_preview = now
        info = message.info
        self.preview.append(
            {
                "kind": "costmap", "timestamp": time.time(), "frame": message.header.frame_id,
                "width": int(info.width), "height": int(info.height), "resolution": float(info.resolution),
                "origin": [float(info.origin.position.x), float(info.origin.position.y)],
                "costs": [int(value) for value in message.data],
            },
            keep_s=profile.pre_trigger_s,
        )

    def trigger(self, trigger: str, payload: dict[str, Any] | None = None) -> None:
        with self._pending_lock:
            if self._pending:
                return
            self._pending = True
        profile = self._profile()
        before_samples = self.samples.snapshot()
        before_preview = self.preview.snapshot()
        triggered_epoch = time.time()
        threading.Thread(
            target=self._finish,
            args=(trigger, payload or {}, profile, before_samples, before_preview, triggered_epoch),
            name="incident-sealer",
            daemon=True,
        ).start()

    def _finish(self, trigger, payload, profile, before_samples, before_preview, triggered_epoch) -> None:
        try:
            if profile.post_trigger_s:
                time.sleep(profile.post_trigger_s)
            after_samples = [item for item in self.samples.snapshot() if item.timestamp_ns > int(triggered_epoch * 1e9)]
            after_preview = [item for item in self.preview.snapshot() if float(item["timestamp"]) > triggered_epoch]
            incident_id = new_incident_id()
            root = self.incidents.root / incident_id
            root.mkdir(parents=True, exist_ok=False)
            candidate = self._read_json(self.data_root / "navigation" / "selected-candidate.json")
            nav_profile = self._read_json(self.data_root / "navigation" / "profiles" / "active.json")
            replay = {
                "schema": "gogoguard.incident_replay.v1",
                "triggered_at_epoch": triggered_epoch,
                "events": sorted(before_preview + after_preview, key=lambda value: float(value["timestamp"])),
                "route": self._current_route,
                "last_pose": self._current_pose,
                "runtime": payload,
            }
            (root / "replay.json").write_text(json.dumps(replay, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")
            (root / "decisions.json").write_text(json.dumps({"trigger": payload, "runtime": self._current_runtime}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            (root / "parameters.json").write_text(json.dumps({"diagnostics": json_ready(profile), "navigation": nav_profile}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            (root / "map-reference.json").write_text(json.dumps(candidate or {}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            route_source = Path(str((candidate or {}).get("route") or ""))
            if route_source.is_file():
                shutil.copy2(route_source, root / "route.json")
            samples = sorted(before_samples + after_samples, key=lambda item: item.timestamp_ns)
            raw_written = self._write_mcap(root / "sensors", samples) if samples else False
            sample_topics = {item.topic for item in samples}
            camera_files = []
            if profile.record_camera:
                self.camera.stop()
                camera_files = self.camera.copy_window(
                    root / "camera", triggered_epoch - profile.pre_trigger_s,
                    triggered_epoch + profile.post_trigger_s,
                )
            map_version = (candidate or {}).get("map_version") or payload.get("mapVersion")
            route_id = (candidate or {}).get("route_id") or payload.get("routeId")
            present = ["runtime_decisions", "parameters"]
            missing = []
            if raw_written:
                present.append("raw_ros_messages")
            if "/navigation/cloud_body" in sample_topics:
                present.append("point_cloud")
            elif profile.point_cloud_hz > 0:
                missing.append("point_cloud")
            if "/localization/pose" in sample_topics:
                present.append("localization")
            else:
                missing.append("localization")
            if sample_topics.intersection({"/nav2/raw_cmd_vel", "/patrol_cmd", "/cmd_vel"}):
                present.append("commands")
            else:
                missing.append("commands")
            if self._current_route or (root / "route.json").is_file():
                present.append("route")
            else:
                missing.append("route")
            if candidate:
                present.append("map_reference")
            else:
                missing.append("map_reference")
            if "/local_costmap/costmap" in sample_topics or any(item.get("kind") == "costmap" for item in replay["events"]):
                present.append("local_costmap")
            elif profile.record_costmap:
                missing.append("local_costmap")
            planner_present = any(
                item.get("kind") == "decision"
                and item.get("topic") == "/go2/runtime/planner_diagnostics"
                for item in replay["events"]
            )
            if planner_present:
                present.append("planner_search")
            elif profile.record_planner_detail and trigger in {"blocked", "fault"}:
                missing.append("planner_search")
            if camera_files:
                present.append("camera")
            elif profile.record_camera:
                missing.append("camera")
            bundle = IncidentBundle(
                incident_id=incident_id,
                trigger=trigger,
                triggered_at=datetime.fromtimestamp(triggered_epoch, timezone.utc).isoformat(timespec="milliseconds"),
                started_at=datetime.fromtimestamp(triggered_epoch - profile.pre_trigger_s, timezone.utc).isoformat(timespec="milliseconds"),
                ended_at=datetime.fromtimestamp(triggered_epoch + profile.post_trigger_s, timezone.utc).isoformat(timespec="milliseconds"),
                site_id=self.site_id,
                robot_id=self.robot_id,
                sensor_id=self.sensor_id,
                map_version=map_version,
                route_id=route_id,
                navigation_profile_revision=(nav_profile or {}).get("revision"),
                diagnostic_profile_revision=profile.revision,
                diagnostic_mode=profile.mode,
                summary={"runtimeState": payload.get("state"), "runtimeReason": payload.get("reason"), "sampleCount": len(samples), "previewEventCount": len(replay["events"])},
            )
            state = IncidentState.SEALED if not missing else IncidentState.PARTIAL
            self.incidents.seal(bundle, evidence_present=present, evidence_missing=missing, state=state)
        except Exception as exc:
            self.node.get_logger().error(f"incident sealing failed: {exc}")
        finally:
            self._pending = False
            if self._last_runtime_state not in ACTIVE_STATES:
                self._patrol_active = False
                self.camera.stop()
                self._patrol_profile = None

    @staticmethod
    def _write_mcap(uri: Path, samples: list[SerializedSample]) -> bool:
        try:
            import rosbag2_py
        except ImportError:
            return False
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(uri), storage_id="mcap"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
        topics = {(sample.topic, sample.type_name) for sample in samples}
        for topic, type_name in sorted(topics):
            writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=type_name, serialization_format="cdr", offered_qos_profiles=""))
        for sample in samples:
            writer.write(sample.topic, sample.data, sample.timestamp_ns)
        return True

    @staticmethod
    def _read_json(path: Path) -> dict[str, Any] | None:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            return value if isinstance(value, dict) else None
        except (OSError, ValueError):
            return None

    def _tick(self) -> None:
        for request in self.incidents.pop_requests():
            self.trigger(str(request.get("trigger") or "manual"), request)
        self.camera.prune()

    def run(self) -> None:
        try:
            self.rclpy.spin(self.node)
        finally:
            self.camera.stop()
            self.node.destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description="GoGoGuard bounded incident recorder")
    parser.add_argument("--data-root", type=Path, default=Path("/var/lib/gogoguard"))
    parser.add_argument("--site-id", default="local-first-site")
    parser.add_argument("--robot-id", default="LLYJ0001")
    parser.add_argument("--sensor-id", default="ARMCP6B0035634")
    parser.add_argument("--camera-source", default="rtsp://192.168.144.108/")
    args = parser.parse_args()

    import rclpy

    rclpy.init(args=None)
    recorder = IncidentRecorder(
        data_root=args.data_root,
        site_id=args.site_id,
        robot_id=args.robot_id,
        sensor_id=args.sensor_id,
        camera_source=args.camera_source,
    )
    try:
        recorder.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
