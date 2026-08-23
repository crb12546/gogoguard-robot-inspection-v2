from __future__ import annotations

import argparse
import json
import signal
import threading
from pathlib import Path

from gogoguard_site_console.server import SiteConsoleServer

from .application import FieldWorkstationApplication


def main() -> None:
    parser = argparse.ArgumentParser(description="GoGoGuard Mac field workstation")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--data-root", type=Path, default=Path("workstation-data"))
    parser.add_argument("--map-worker", choices=("demo", "ssh"), default="ssh")
    parser.add_argument("--config", type=Path, default=Path("config/workstation.json"))
    parser.add_argument("--static-root", type=Path, default=Path("apps/site_console/frontend"))
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    app = FieldWorkstationApplication(
        data_root=args.data_root,
        robot=config["robot"],
        cloud=config["cloud"],
        map_worker=args.map_worker,
        site_id=str(config.get("site_id") or "local-first-site"),
        sensor_id=str(config.get("sensor_id") or "ARMCP6B0035634"),
        platform=config.get("platform"),
    )
    app.start()
    server = SiteConsoleServer((args.host, args.port), app, args.static_root)
    signal.signal(
        signal.SIGTERM,
        lambda *_: threading.Thread(target=server.shutdown, daemon=True).start(),
    )
    print(
        f"GoGoGuard field workstation: http://{args.host}:{args.port} robot={config['robot']['base_url']}",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        app.close()


if __name__ == "__main__":
    main()
