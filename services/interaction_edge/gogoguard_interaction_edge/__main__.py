from __future__ import annotations

import argparse
import os
import signal
import threading
from pathlib import Path

from .service import InteractionEdgeService, InteractionUnixServer


def main() -> None:
    parser = argparse.ArgumentParser(description="GoGoGuard robot realtime interaction service")
    parser.add_argument("--robot-id", default=os.environ.get("GOGOGUARD_ROBOT_ID", "LLYJ0001"))
    parser.add_argument("--hardware-config", type=Path, required=True)
    parser.add_argument("--persona-config", type=Path, required=True)
    parser.add_argument("--socket", type=Path, default=Path("/var/lib/gogoguard/interaction/control.sock"))
    parser.add_argument("--status", type=Path, default=Path("/var/lib/gogoguard/interaction/status.json"))
    parser.add_argument(
        "--mission-inbox",
        type=Path,
        default=Path("/var/lib/gogoguard/platform/checkpoint-inbox.jsonl"),
    )
    parser.add_argument("--allow-insecure-ws", action="store_true")
    parser.add_argument(
        "--volume-executable",
        type=Path,
        default=Path("/opt/gogoguard/ros_ws/install/lib/go2_cmd_vel_bridge/go2_vui_control"),
    )
    args = parser.parse_args()
    key = os.environ.get("UNITREE_AES_128_KEY", "")
    if not key:
        raise SystemExit("UNITREE_AES_128_KEY is required when interaction is enabled")
    service = InteractionEdgeService(
        robot_id=args.robot_id,
        hardware_config=args.hardware_config,
        persona_config=args.persona_config,
        unitree_aes_128_key=key,
        volume_executable=args.volume_executable,
        status_path=args.status,
        mission_inbox_path=args.mission_inbox,
        allow_insecure_ws=args.allow_insecure_ws,
    )
    server = InteractionUnixServer(args.socket, service)
    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    signal.signal(signal.SIGINT, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    server.serve_forever()


if __name__ == "__main__":
    main()
