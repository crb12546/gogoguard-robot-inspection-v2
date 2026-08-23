from __future__ import annotations

import argparse
import json
import math
import socket
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from gogoguard_contracts import utc_now


def _finite(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("pose value is not numeric")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("pose value is not finite")
    return number


def _optional_finite(value: Any) -> float | None:
    try:
        return _finite(value)
    except ValueError:
        return None


def build_pose_stream_payload(
    *,
    robot_id: str,
    sequence: int,
    runtime_instance_id: str,
    map_version: str,
    route_id: str,
    frame_id: str,
    x: float,
    y: float,
    z: float,
    yaw_rad: float,
    source_at: str,
    localization: dict[str, Any],
    mission: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if not robot_id or not runtime_instance_id or not map_version or not route_id:
        raise ValueError("pose identity, runtime, and map binding are required")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError("pose sequence is invalid")
    if frame_id != "map" or not source_at:
        raise ValueError("pose must use the map frame and a source timestamp")
    confidence = _optional_finite(localization.get("confidence"))
    age_s = localization.get("ageS", localization.get("age_s"))
    age_s = _optional_finite(age_s)
    payload = {
        "schema": "gogoguard.robot_pose.v1",
        "robotId": robot_id,
        "sequence": sequence,
        "runtimeInstanceId": runtime_instance_id,
        "mapVersion": map_version,
        "routeId": route_id,
        "frameId": "map",
        "pose": {
            "position": {
                "x": _finite(x),
                "y": _finite(y),
                "z": _finite(z),
            },
            "yawRad": _finite(yaw_rad),
        },
        "localization": {
            "usable": localization.get("usable") is True,
            "confidence": confidence,
            "reason": localization.get("reason"),
            "ageS": age_s,
        },
        "sourceAt": source_at,
        "observedAt": utc_now(),
    }
    if mission is not None:
        payload["mission"] = mission
    return payload


def build_mission_pose_context(
    runtime: dict[str, Any],
    platform_checkpoint: dict[str, Any],
    *,
    spin_progress_rad: float | None = None,
) -> dict[str, Any] | None:
    checkpoint = runtime.get("checkpoint")
    checkpoint = checkpoint if isinstance(checkpoint, dict) else {}
    mission_id = str(checkpoint.get("missionId") or "")
    if not mission_id:
        return None
    nav_phase = str(checkpoint.get("phase") or "TRAVELING")
    phase = {
        "TRAVELING": "traveling",
        "PAUSING": "stopping",
        "SETTLING": "stopping",
        "POSE_REQUESTED": "posing",
        "POSING": "posing",
        "SPIN_REQUESTED": "spinning",
        "SPINNING": "spinning",
        "WAITING_VERDICT": "waiting_verdict",
    }.get(nav_phase, "idle")
    if nav_phase == "WAITING_PLATFORM":
        stage = str(platform_checkpoint.get("stage") or "")
        phase = "announcing" if stage in {"new", "announcing"} else "capturing"
    camera = platform_checkpoint.get("camera")
    if not isinstance(camera, dict):
        camera = checkpoint.get("camera")
    if not isinstance(camera, dict):
        camera = {"pan": 0.0, "tilt": 0.0}
    value: dict[str, Any] = {
        "missionId": mission_id,
        "checkpointId": checkpoint.get("activeCheckpointId"),
        "phase": phase,
        "camera": camera,
    }
    if phase == "spinning" and spin_progress_rad is not None:
        value["spinProgressRad"] = max(0.0, float(spin_progress_rad))
    return value


class InteractionPoseClient:
    def __init__(self, socket_path: Path, *, timeout_s: float = 0.05) -> None:
        self.socket_path = Path(socket_path)
        self.timeout_s = float(timeout_s)

    def publish(self, payload: dict[str, Any]) -> bool:
        encoded = (
            json.dumps(
                {"action": "publish_pose", "payload": payload},
                ensure_ascii=False,
                separators=(",", ":"),
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
        if len(encoded) > 16384:
            raise ValueError("pose request exceeds 16 KiB")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(self.timeout_s)
            client.connect(str(self.socket_path))
            client.sendall(encoded)
            raw = client.makefile("rb").readline(65537)
        if not raw or len(raw) > 65536:
            raise RuntimeError("interaction pose response is invalid")
        response = json.loads(raw.decode("utf-8"))
        if not isinstance(response, dict) or response.get("ok") is not True:
            raise RuntimeError("interaction service rejected pose")
        result = response.get("result")
        return isinstance(result, dict) and result.get("accepted") is True


def _read_navigation_status(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def _source_time(stamp: Any) -> str:
    seconds = int(getattr(stamp, "sec", 0))
    nanoseconds = int(getattr(stamp, "nanosec", 0))
    if seconds <= 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("pose source timestamp is invalid")
    instant = datetime.fromtimestamp(
        seconds + nanoseconds / 1_000_000_000.0,
        tz=timezone.utc,
    )
    return instant.isoformat(timespec="milliseconds")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="publish map-bound localization pose through the existing LiveKit session"
    )
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--topic", default="/localization/pose")
    parser.add_argument(
        "--navigation-status",
        type=Path,
        default=Path("/var/lib/gogoguard/navigation/status.json"),
    )
    parser.add_argument(
        "--interaction-socket",
        type=Path,
        default=Path("/var/lib/gogoguard/interaction/control.sock"),
    )
    parser.add_argument("--max-hz", type=float, default=10.0)
    parser.add_argument(
        "--checkpoint-state",
        type=Path,
        default=Path("/var/lib/gogoguard/platform/checkpoint-state.json"),
    )
    args = parser.parse_args()
    if not 1.0 <= args.max_hz <= 20.0:
        raise SystemExit("max-hz must be between 1 and 20")

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rclpy.init()
    node = rclpy.create_node("gogoguard_platform_pose_stream")
    client = InteractionPoseClient(args.interaction_socket)
    sequence = 0
    last_attempt = 0.0
    last_log = 0.0
    spin_progress = 0.0
    last_spin_yaw: float | None = None

    def pose_callback(message: PoseWithCovarianceStamped) -> None:
        nonlocal sequence, last_attempt, last_log, spin_progress, last_spin_yaw
        now = time.monotonic()
        if now - last_attempt < 1.0 / args.max_hz:
            return
        last_attempt = now
        status = _read_navigation_status(args.navigation_status)
        runtime = status.get("runtime")
        runtime = runtime if isinstance(runtime, dict) else {}
        localization = status.get("localization")
        localization = localization if isinstance(localization, dict) else {}
        map_version = str(runtime.get("mapVersion") or "")
        route_id = str(runtime.get("routeId") or "")
        runtime_instance_id = str(runtime.get("runtimeInstanceId") or "")
        if (
            localization.get("usable") is not True
            or not runtime_instance_id
            or not map_version
            or not route_id
        ):
            return
        pose = message.pose.pose
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        checkpoint = runtime.get("checkpoint")
        checkpoint = checkpoint if isinstance(checkpoint, dict) else {}
        if checkpoint.get("phase") == "SPINNING":
            if last_spin_yaw is not None:
                spin_progress += abs(
                    math.atan2(math.sin(yaw - last_spin_yaw), math.cos(yaw - last_spin_yaw))
                )
            last_spin_yaw = yaw
        else:
            spin_progress = 0.0
            last_spin_yaw = None
        try:
            payload = build_pose_stream_payload(
                robot_id=args.robot_id,
                sequence=sequence,
                runtime_instance_id=runtime_instance_id,
                map_version=map_version,
                route_id=route_id,
                frame_id=str(message.header.frame_id),
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
                yaw_rad=yaw,
                source_at=_source_time(message.header.stamp),
                localization=localization,
                mission=build_mission_pose_context(
                    runtime,
                    _read_navigation_status(args.checkpoint_state),
                    spin_progress_rad=spin_progress,
                ),
            )
            accepted = client.publish(payload)
            if accepted:
                sequence += 1
        except Exception as exc:
            # Live interaction is optional to navigation. A closed room or a
            # transient Unix timeout must never stop localization or patrol.
            if now - last_log >= 10.0:
                node.get_logger().warning(
                    f"pose stream unavailable: {type(exc).__name__}"
                )
                last_log = now

    node.create_subscription(
        PoseWithCovarianceStamped,
        args.topic,
        pose_callback,
        10,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
