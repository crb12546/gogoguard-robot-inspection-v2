from __future__ import annotations

import sys
from pathlib import Path


dockerfile, service, installer, cloud_adapter = (Path(item) for item in sys.argv[1:5])
checks = {
    dockerfile: ["ros:humble-ros-base-jammy", "linux/arm64", "ros-humble-rosbag2", "ros-humble-rosbag2-storage-mcap", "ffmpeg", "alsa-utils", "Livox-SDK2", "fast_lio", "modules/navigation/ros/go2_nav2_runtime", "modules/device_io/ros/go2_cmd_vel_bridge", "go2_vui_control", "interaction-requirements.txt", "2da379972ba86627632aa7e3f779c680ba04a5ee26ef2a20dc61cefcc24f73b8", "8189/udp", "LD_LIBRARY_PATH=/opt/gogoguard/deps/lib:/usr/local/lib", "gogoguard-edge-entrypoint"],
    service: ["/opt/gogoguard/bin/run-edge", "ExecStartPre=/opt/gogoguard/bin/validate-runtime-env /etc/gogoguard/runtime.env", "ExecStartPre=/opt/gogoguard/bin/wait-clock-sync 60", "systemd-timesyncd.service", "ExecStop=/usr/bin/docker stop -t 15 gogoguard-edge", "SuccessExitStatus=143", "Restart=on-failure", "StartLimitBurst=3", "After=docker.service"],
    installer: ["sha256sum --check", "docker load --input", "commission-device-token", "configure-combined-joint-test", "validate-runtime-env", "runtime.env.install", "wait-clock-sync", "systemctl daemon-reload", "Service is not started or enabled"],
    cloud_adapter: ["/opt/go2/jobs/map-", "run_pipeline prepare-session", "run_pipeline run-session", "run_pipeline produce-review-artifacts", "overview.svg", "map.json", "trajectory-poses.json", "gogoguard-export-glim-artifact"],
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
    "trajectory-poses.json",
    "gogoguard.optimized_trajectory.v1",
    "gogoguard.recording_route_coverage.v1",
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
entrypoint_text = entrypoint.read_text(encoding="utf-8")
if "Each Unix client connects lazily" not in entrypoint_text:
    raise SystemExit("platform clients must retain lazy module boundaries")
if "gogoguard-platform-pose-stream" not in entrypoint_text:
    raise SystemExit("robot image must contain the map-bound pose data publisher")
if entrypoint_text.index("navigation supervisor did not become ready") > entrypoint_text.index(
    "gogoguard-platform-edge"
):
    raise SystemExit("platform patrol control must start after the navigation owner")
joint_text = joint_test.read_text(encoding="utf-8")
for required in (
    "--temporary-icp-fallback",
    "http://39.96.37.187/api/v1/robot/heartbeat",
    "GOGOGUARD_INTERACTION_ALLOW_INSECURE_WS 1",
    "GOGOGUARD_PLATFORM_HEARTBEAT_TLS_INSECURE 0",
    "GOGOGUARD_PLATFORM_HEARTBEAT_ALLOW_HTTP 1",
):
    if required not in joint_text:
        raise SystemExit(f"temporary joint-test profile is incomplete: {required}")
if not combined_dockerfile.is_file():
    raise SystemExit("combined release Dockerfile is missing")
combined_content = combined_dockerfile.read_text(encoding="utf-8")
for required in (
    "v2-edge-20260817-mission-plan-r1@sha256:41f8b4914c212a84cdb92452f98e0e1dd388b63842f0d02c7ec75f13bb7d6673",
    "--target /opt/gogoguard/interaction-python",
    'numpy.__version__ == "1.21.5"',
    'numpy.__version__ == "2.0.2"',
    "src/go2_vui_control.cpp",
    "/opt/gogoguard/ros_ws/install/lib/go2_cmd_vel_bridge/go2_vui_control",
    "site-packages/go2_nav2_runtime/runtime_core.py",
    "site-packages/go2_nav2_runtime/patrol_runtime_manager.py",
    "share/go2_nav2_runtime/launch/active_map_patrol.launch.py",
    "share/go2_nav2_runtime/config/go2_nav2_patrol.yaml",
    "navigation_map_path: Path",
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
