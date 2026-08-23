from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
import time
from pathlib import Path


ACTIVE_PATROL_STATES = {
    "PATROLLING",
    "RECOVERING",
    "SEARCHING_PATH",
    "BLOCKED",
    "PAUSED",
    "CHECKPOINT_PAUSING",
    "CHECKPOINT_SETTLING",
    "INSPECTING",
}
RUNTIME_STATUS_TIMEOUT_S = 3.0


def fail_closed_runtime_after_timeout(
    runtime: object,
    *,
    last_runtime_monotonic: float | None,
    now_monotonic: float,
    timeout_s: float = RUNTIME_STATUS_TIMEOUT_S,
) -> object:
    """Turn a lost active runtime publisher into a durable terminal status.

    The observer outlives an individually launched Nav2 generation.  Without
    this liveness fence its last PATROLLING sample would be written forever
    after that generation exited, making the platform believe a dead patrol
    was still running.  Monotonic time is used only for local publisher
    liveness; no wall-clock comparison or stale-data fallback is involved.
    """
    if not isinstance(runtime, dict):
        return runtime
    state = str(runtime.get("state") or "")
    if state not in ACTIVE_PATROL_STATES or last_runtime_monotonic is None:
        return runtime
    if now_monotonic - last_runtime_monotonic <= float(timeout_s):
        return runtime
    terminal = dict(runtime)
    terminal.update(
        {
            "state": "INTERRUPTED",
            "reason": "NAV_RUNTIME_LOST",
            "operatorMessage": "导航运行时已退出，巡检中断，未继续发送运动控制",
            "motionAuthorized": False,
            "runtimeStatusStale": True,
        }
    )
    return terminal


def _parse(value: str):
    try:
        return json.loads(value)
    except (TypeError, ValueError):
        return {"raw": value}


def _atomic(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, separators=(",", ":"))
            handle.write("\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--edge-generation-id", required=True)
    args = parser.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Path as NavPath
    from std_msgs.msg import String

    rclpy.init(args=None)
    node = rclpy.create_node("gogoguard_navigation_observer")
    state = {
        "edge_generation_id": args.edge_generation_id,
        "runtime": None,
        "localization": None,
        "localization_pose": None,
        "safety": None,
        "route": [],
        "commands": {"nav2": None, "collision_filtered": None, "final": None},
        "observed_at": None,
    }
    last_runtime_monotonic: float | None = None

    def text_callback(name):
        def callback(message: String):
            nonlocal last_runtime_monotonic
            state[name] = _parse(message.data)
            if name == "runtime":
                last_runtime_monotonic = time.monotonic()
        return callback

    def pose_callback(message: PoseWithCovarianceStamped):
        pose = message.pose.pose
        q = pose.orientation
        state["localization_pose"] = {
            "frame": message.header.frame_id,
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        }

    def route_callback(message: NavPath):
        state["route"] = [
            [float(item.pose.position.x), float(item.pose.position.y)]
            for item in message.poses
        ]

    def command_callback(name):
        def callback(message: Twist):
            state["commands"][name] = {
                "vx": float(message.linear.x), "vy": float(message.linear.y),
                "wz": float(message.angular.z),
            }
        return callback

    node.create_subscription(String, "/go2/runtime/status", text_callback("runtime"), 10)
    node.create_subscription(String, "/localization/status", text_callback("localization"), 10)
    node.create_subscription(String, "/go2/safety/status", text_callback("safety"), 10)
    node.create_subscription(PoseWithCovarianceStamped, "/localization/pose", pose_callback, 10)
    node.create_subscription(NavPath, "/go2/runtime/route", route_callback, 10)
    node.create_subscription(Twist, "/nav2/raw_cmd_vel", command_callback("nav2"), 10)
    node.create_subscription(Twist, "/patrol_cmd", command_callback("collision_filtered"), 10)
    node.create_subscription(Twist, "/cmd_vel", command_callback("final"), 10)

    last_write = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_write >= 0.2:
                state["runtime"] = fail_closed_runtime_after_timeout(
                    state.get("runtime"),
                    last_runtime_monotonic=last_runtime_monotonic,
                    now_monotonic=now,
                )
                state["observed_at"] = time.time()
                _atomic(args.output, state)
                last_write = now
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
