from __future__ import annotations

import argparse
import os
import signal
import threading
from pathlib import Path

from .heartbeat import (
    CommandLedger,
    InteractionControlClient,
    NavigationControlClient,
    PlatformHeartbeatService,
    UrllibJsonPoster,
    command_result_url,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="GoGoGuard robot platform heartbeat adapter")
    parser.add_argument("--robot-id", default=os.environ.get("GOGOGUARD_ROBOT_ID", "LLYJ0001"))
    parser.add_argument("--url", required=True)
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument(
        "--interaction-socket",
        type=Path,
        default=Path("/var/lib/gogoguard/interaction/control.sock"),
    )
    parser.add_argument(
        "--navigation-socket",
        type=Path,
        default=Path("/var/lib/gogoguard/navigation/supervisor.sock"),
    )
    parser.add_argument(
        "--navigation-status",
        type=Path,
        default=Path("/var/lib/gogoguard/navigation/status.json"),
    )
    parser.add_argument(
        "--battery-status",
        type=Path,
        default=Path("/var/lib/gogoguard/device/battery.json"),
    )
    parser.add_argument(
        "--interaction-status",
        type=Path,
        default=Path("/var/lib/gogoguard/interaction/status.json"),
    )
    parser.add_argument(
        "--capabilities",
        type=Path,
        default=Path("/opt/gogoguard/app/config/robot/inspection-capabilities.json"),
    )
    parser.add_argument(
        "--state-root",
        type=Path,
        default=Path("/var/lib/gogoguard/platform"),
    )
    parser.add_argument("--tls-insecure", action="store_true")
    parser.add_argument("--host-header", default="")
    parser.add_argument("--allow-insecure-http", action="store_true")
    args = parser.parse_args()

    poster = UrllibJsonPoster(
        device_token=os.environ.get("GOGOGUARD_DEVICE_TOKEN", ""),
        tls_insecure=args.tls_insecure,
        host_header=args.host_header,
        allow_insecure_http=args.allow_insecure_http,
    )
    service = PlatformHeartbeatService(
        robot_id=args.robot_id,
        heartbeat_url=args.url,
        navigation_status_path=args.navigation_status,
        battery_status_path=args.battery_status,
        interaction_status_path=args.interaction_status,
        capabilities_path=args.capabilities,
        service_status_path=args.state_root / "status.json",
        ledger=CommandLedger(args.state_root / "command-ledger.json"),
        interaction_client=InteractionControlClient(args.interaction_socket),
        navigation_client=NavigationControlClient(args.navigation_socket),
        post_json=poster,
        command_result_url=command_result_url(args.url),
        post_result=poster,
        interval_s=args.interval,
        timeout_s=args.timeout,
    )
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    service.run(stop)


if __name__ == "__main__":
    main()
