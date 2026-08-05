from __future__ import annotations

import sys
from pathlib import Path


dockerfile, service, installer = (Path(item) for item in sys.argv[1:4])
checks = {
    dockerfile: ["ros:humble-ros-base-jammy", "linux/arm64", "ros-humble-rosbag2", "Livox-SDK2", "fast_lio", "LD_LIBRARY_PATH=/usr/local/lib", "gogoguard-edge-entrypoint"],
    service: ["/opt/gogoguard/bin/run-edge", "ExecStop=/usr/bin/docker stop --timeout 15 gogoguard-edge", "Restart=on-failure", "StartLimitBurst=3", "After=docker.service"],
    installer: ["sha256sum --check", "docker load --input", "systemctl daemon-reload", "Service is not started or enabled"],
}
for path, needles in checks.items():
    if not path.is_file():
        raise SystemExit(f"missing deployment file: {path}")
    content = path.read_text(encoding="utf-8")
    for needle in needles:
        if needle not in content:
            raise SystemExit(f"{path}: missing required contract {needle}")
print("robot container contract: ok (image not built by this check)")
