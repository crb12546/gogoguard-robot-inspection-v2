from __future__ import annotations

import sys
from pathlib import Path


dockerfile, service, installer, cloud_adapter = (Path(item) for item in sys.argv[1:5])
checks = {
    dockerfile: ["ros:humble-ros-base-jammy", "linux/arm64", "ros-humble-rosbag2", "ros-humble-rosbag2-storage-mcap", "ffmpeg", "alsa-utils", "Livox-SDK2", "fast_lio", "modules/navigation/ros/go2_nav2_runtime", "modules/device_io/ros/go2_cmd_vel_bridge", "go2_vui_control", "interaction-requirements.txt", "2da379972ba86627632aa7e3f779c680ba04a5ee26ef2a20dc61cefcc24f73b8", "8189/udp", "LD_LIBRARY_PATH=/opt/gogoguard/deps/lib:/usr/local/lib", "gogoguard-edge-entrypoint"],
    service: ["/opt/gogoguard/bin/run-edge", "ExecStartPre=/opt/gogoguard/bin/wait-clock-sync 60", "systemd-timesyncd.service", "ExecStop=/usr/bin/docker stop -t 15 gogoguard-edge", "SuccessExitStatus=143", "Restart=on-failure", "StartLimitBurst=3", "After=docker.service"],
    installer: ["sha256sum --check", "docker load --input", "configure-combined-joint-test", "runtime.env.install", "wait-clock-sync", "systemctl daemon-reload", "Service is not started or enabled"],
    cloud_adapter: ["/opt/go2/jobs/map-", "run_pipeline prepare-session", "run_pipeline run-session", "run_pipeline produce-review-artifacts", "overview.svg", "map.json"],
}
cloud_editor = cloud_adapter.parent / "gogoguard-glim-editor"
cloud_exporter = cloud_adapter.parent / "export_glim_artifact.py"
checks[cloud_editor] = [
    "install/lib/glim_ros/map_editor",
    "install/lib/glim_ros/offline_viewer",
    "working-map",
    "SAVED_MAP",
    "x11vnc",
    "websockify",
    "127.0.0.1",
    "systemd-run",
]
checks[cloud_exporter] = [
    "parent_map_job_id",
    "official-glim-map-editor",
    "map.ply",
    "glim-build.json",
]
for path, needles in checks.items():
    if not path.is_file():
        raise SystemExit(f"missing deployment file: {path}")
    content = path.read_text(encoding="utf-8")
    for needle in needles:
        if needle not in content:
            raise SystemExit(f"{path}: missing required contract {needle}")
run_edge = service.parent / "run-edge"
joint_test = service.parent / "configure-combined-joint-test"
entrypoint = dockerfile.parent / "edge-entrypoint"
combined_dockerfile = dockerfile.parent / "Dockerfile.combined"
interaction_requirements = dockerfile.parent / "interaction-requirements.txt"
clock_gate = service.parent / "wait-clock-sync"
if not clock_gate.is_file() or "NTPSynchronized" not in clock_gate.read_text(encoding="utf-8"):
    raise SystemExit("robot release must gate Livox startup on real NTP readiness")
if "cloud_ssh" in run_edge.read_text(encoding="utf-8"):
    raise SystemExit("robot release must not mount a cloud SSH key")
if "GOGOGUARD_MAP_WORKER:-none" not in entrypoint.read_text(encoding="utf-8"):
    raise SystemExit("robot Edge Agent must default to map worker none")
if "gogoguard-incident-recorder" not in entrypoint.read_text(encoding="utf-8"):
    raise SystemExit("robot image must retain the dormant incident recorder")
if "GOGOGUARD_INTERACTION_ENABLED:-0" not in entrypoint.read_text(encoding="utf-8"):
    raise SystemExit("robot interaction must remain explicitly gated until commissioned")
if "gogoguard-interaction-edge" not in entrypoint.read_text(encoding="utf-8"):
    raise SystemExit("robot image must contain the formal interaction service")
if "GOGOGUARD_PLATFORM_HEARTBEAT_ENABLED:-0" not in entrypoint.read_text(encoding="utf-8"):
    raise SystemExit("robot platform heartbeat must remain explicitly gated until commissioned")
if "gogoguard-platform-edge" not in entrypoint.read_text(encoding="utf-8"):
    raise SystemExit("robot image must contain the formal platform heartbeat adapter")
if "platform heartbeat requires the interaction service" not in entrypoint.read_text(encoding="utf-8"):
    raise SystemExit("platform live control must not start without the interaction service")
joint_text = joint_test.read_text(encoding="utf-8")
for required in (
    "--temporary-icp-fallback",
    "https://39.96.37.187/api/v1/robot/heartbeat",
    "GOGOGUARD_INTERACTION_ALLOW_INSECURE_WS 1",
    "GOGOGUARD_PLATFORM_HEARTBEAT_TLS_INSECURE 1",
    "GOGOGUARD_PLATFORM_HEARTBEAT_ALLOW_HTTP 0",
):
    if required not in joint_text:
        raise SystemExit(f"temporary joint-test profile is incomplete: {required}")
if not combined_dockerfile.is_file():
    raise SystemExit("combined release Dockerfile is missing")
combined_content = combined_dockerfile.read_text(encoding="utf-8")
for required in (
    "v2-edge-20260810-v6-r4@sha256:b8ca1b65ede57fa0f7ca2cfefa3c1490b905717aa3681d245128fa4e9081269c",
    "--target /opt/gogoguard/interaction-python",
    'numpy.__version__ == "1.21.5"',
    'numpy.__version__ == "2.0.2"',
):
    if required not in combined_content:
        raise SystemExit(f"combined release is missing frozen-baseline contract: {required}")
if "apt-get" in combined_content:
    raise SystemExit("combined release must not mutate the accepted V6 OS/ROS packages")
requirements = [
    line.strip()
    for line in interaction_requirements.read_text(encoding="utf-8").splitlines()
    if line.strip() and not line.lstrip().startswith("#")
]
if len(requirements) < 25 or any(line.count("==") != 1 for line in requirements):
    raise SystemExit("realtime media requirements must retain the fully pinned probe lock")
print("robot container contract: ok (image not built by this check)")
