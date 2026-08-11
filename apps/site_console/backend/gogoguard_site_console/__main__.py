from __future__ import annotations

import argparse
import json
import signal
import threading
from pathlib import Path

from .application import InspectionApplication
from .server import SiteConsoleServer


def main() -> None:
    parser = argparse.ArgumentParser(description="Gogoguard on-site mapping console")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--mode", choices=("demo", "robot"), default="demo")
    parser.add_argument("--map-worker", choices=("none", "demo", "ssh"), default="demo")
    parser.add_argument("--data-root", type=Path, default=Path("runtime-data"))
    parser.add_argument("--robot-id", default="go2-100tops-01")
    parser.add_argument("--sensor-id", default="ARMCP6B0035634")
    parser.add_argument("--site-id", default="local-first-site")
    parser.add_argument("--config", type=Path, default=Path("config/default.json"))
    parser.add_argument(
        "--capabilities-config",
        type=Path,
        default=Path("config/robot/inspection-capabilities.json"),
    )
    parser.add_argument("--static-root", type=Path, default=Path("apps/site_console/frontend"))
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    capabilities = json.loads(args.capabilities_config.read_text(encoding="utf-8"))
    app = InspectionApplication(data_root=args.data_root, mode=args.mode, map_worker=args.map_worker,
                                robot_id=args.robot_id, site_id=args.site_id,
                                sensor_id=args.sensor_id,
                                topics=config["topics"], cloud=config.get("cloud"),
                                camera=config.get("camera"), capabilities=capabilities,
                                gimbal=config.get("gimbal"))
    app.start()
    server = SiteConsoleServer((args.host, args.port), app, args.static_root)
    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    print(f"Gogoguard site console: http://{args.host}:{args.port} mode={args.mode} map_worker={args.map_worker}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        app.close()


if __name__ == "__main__":
    main()
