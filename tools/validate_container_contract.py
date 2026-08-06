from __future__ import annotations

import sys
from pathlib import Path


dockerfile, service, installer, cloud_adapter = (Path(item) for item in sys.argv[1:5])
checks = {
    dockerfile: ["ros:humble-ros-base-jammy", "linux/arm64", "ros-humble-rosbag2", "Livox-SDK2", "fast_lio", "2da379972ba86627632aa7e3f779c680ba04a5ee26ef2a20dc61cefcc24f73b8", "8189/udp", "LD_LIBRARY_PATH=/opt/gogoguard/deps/lib:/usr/local/lib", "gogoguard-edge-entrypoint"],
    service: ["/opt/gogoguard/bin/run-edge", "ExecStartPre=/opt/gogoguard/bin/wait-clock-sync 60", "systemd-timesyncd.service", "ExecStop=/usr/bin/docker stop -t 15 gogoguard-edge", "SuccessExitStatus=143", "Restart=on-failure", "StartLimitBurst=3", "After=docker.service"],
    installer: ["sha256sum --check", "docker load --input", "wait-clock-sync", "systemctl daemon-reload", "Service is not started or enabled"],
    cloud_adapter: ["/opt/go2/jobs/map-", "run_pipeline prepare-session", "run_pipeline run-session", "run_pipeline produce-review-artifacts", "overview.svg", "map.json"],
}
for path, needles in checks.items():
    if not path.is_file():
        raise SystemExit(f"missing deployment file: {path}")
    content = path.read_text(encoding="utf-8")
    for needle in needles:
        if needle not in content:
            raise SystemExit(f"{path}: missing required contract {needle}")
run_edge = service.parent / "run-edge"
entrypoint = dockerfile.parent / "edge-entrypoint"
clock_gate = service.parent / "wait-clock-sync"
if not clock_gate.is_file() or "NTPSynchronized" not in clock_gate.read_text(encoding="utf-8"):
    raise SystemExit("robot release must gate Livox startup on real NTP readiness")
if "cloud_ssh" in run_edge.read_text(encoding="utf-8"):
    raise SystemExit("robot release must not mount a cloud SSH key")
if "GOGOGUARD_MAP_WORKER:-none" not in entrypoint.read_text(encoding="utf-8"):
    raise SystemExit("robot Edge Agent must default to map worker none")
print("robot container contract: ok (image not built by this check)")
