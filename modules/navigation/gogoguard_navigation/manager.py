from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

from gogoguard_route import RouteManager


SAFE_ID = re.compile(r"^map-[A-Za-z0-9]{12}$")
SDK_RECEIVER = Path(
    "/opt/gogoguard/ros_ws/install/lib/go2_cmd_vel_bridge/"
    "go2_sdk2_udp_receiver"
)
UNITREE_SDK_LIBRARY_PATH = "/opt/gogoguard/deps/lib:/usr/local/lib"


class NavigationManager:
    """Own one coherent localization/Nav2/Unitree runtime generation."""

    def __init__(self, data_root: Path, *, site_id: str, robot_id: str, sensor_id: str) -> None:
        self.data_root = Path(data_root)
        self.site_id = site_id
        self.robot_id = robot_id
        self.sensor_id = sensor_id
        self.root = self.data_root / "navigation"
        self.log_root = self.root / "logs"
        self.status_path = self.root / "status.json"
        self.root.mkdir(parents=True, exist_ok=True)
        self.log_root.mkdir(parents=True, exist_ok=True)
        self.routes = RouteManager(self.data_root, site_id=site_id)
        self._process: subprocess.Popen | None = None
        self._log_handle = None
        self._receiver_process: subprocess.Popen | None = None
        self._receiver_log_handle = None
        self._candidate: dict[str, Any] | None = None
        self._lock = threading.Lock()

    def close(self) -> None:
        self.stop_runtime()

    def prepare(self, job_id: str) -> dict[str, Any]:
        candidate = self.routes.prepare_map_job(job_id)
        with self._lock:
            self._candidate = candidate
        return candidate

    def _loaded_candidate(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self._candidate) if self._candidate else None

    def status(self) -> dict[str, Any]:
        candidate = self._loaded_candidate()
        runtime: dict[str, Any] = {}
        try:
            runtime = json.loads(self.status_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            pass
        process_running = bool(self._process and self._process.poll() is None)
        process_exit = None if not self._process or process_running else self._process.returncode
        receiver_running = bool(
            self._receiver_process and self._receiver_process.poll() is None
        )
        receiver_exit = (
            None
            if not self._receiver_process or receiver_running
            else self._receiver_process.returncode
        )
        return {
            "schema": "gogoguard.navigation_status.v1",
            "candidate": candidate,
            "runtime_process": {"running": process_running, "exit_code": process_exit},
            "motion_bridge": {"running": receiver_running, "exit_code": receiver_exit},
            **runtime,
        }

    @staticmethod
    def _launch_arguments(candidate: dict[str, Any], *, site_id: str, robot_id: str,
                          sensor_id: str, log_root: Path) -> list[str]:
        return [
            "ros2", "launch", "go2_nav2_runtime", "active_map_patrol.launch.py",
            "runtime_source:=candidate",
            f"map_store_root:={candidate['localization_map']}",
            f"site_id:={site_id}",
            f"expected_map_version:={candidate['map_version']}",
            f"candidate_localization_map:={candidate['localization_map']}",
            f"candidate_route:={candidate['route']}",
            f"candidate_runtime_profile:={candidate['runtime_profile']}",
            f"localization_map_hash:={candidate['localization_map_hash']}",
            f"route_hash:={candidate['route_hash']}",
            f"runtime_profile_hash:={candidate['runtime_profile_hash']}",
            "hardware_output_enabled:=true",
            "sdk_receiver_enabled:=false",
            "sdk_interface:=eth0",
            f"runtime_log_dir:={log_root}",
            f"robot_id:={robot_id}",
            f"sensor_id:={sensor_id}",
        ]

    def _receiver_environment(self) -> dict[str, str]:
        receiver_env = dict(os.environ)
        # ROS 2 Humble also ships libddsc.so. Unitree SDK2 must load its paired
        # libddsc/libddscxx build from the locked dependency prefix; mixing the
        # ROS C library with Unitree's C++ library aborts during ChannelFactory
        # initialization with `free(): invalid pointer`.
        receiver_env["LD_LIBRARY_PATH"] = UNITREE_SDK_LIBRARY_PATH
        receiver_env["GO2_SDK_EVENT_LOG"] = str(
            self.log_root / "sdk-events.jsonl"
        )
        return receiver_env

    def start_runtime(self, candidate_id: str) -> dict[str, Any]:
        if not SAFE_ID.fullmatch(candidate_id):
            raise ValueError("invalid navigation candidate id")
        candidate = self.routes.get(candidate_id)
        already_running = False
        with self._lock:
            if self._process and self._process.poll() is None:
                if self._candidate and self._candidate.get("candidate_id") == candidate_id:
                    already_running = True
                else:
                    raise RuntimeError("another navigation runtime is already active")
            if not already_running:
                self._candidate = candidate
                log_path = self.log_root / f"runtime-{candidate_id}.log"
                self._log_handle = log_path.open("ab", buffering=0)
                receiver_log_path = self.log_root / f"motion-bridge-{candidate_id}.log"
                self._receiver_log_handle = receiver_log_path.open("ab", buffering=0)
                self._receiver_process = subprocess.Popen(
                    [str(SDK_RECEIVER), "eth0", "5005", "prepare-posture"],
                    stdin=subprocess.DEVNULL,
                    stdout=self._receiver_log_handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env=self._receiver_environment(),
                )
                # The receiver performs an explicit StandUp + BalanceStand
                # before advertising readiness. Give that bounded posture
                # transition time to fail closed before Nav2 is launched.
                time.sleep(3.75)
                if self._receiver_process.poll() is not None:
                    exit_code = self._receiver_process.returncode
                    self._receiver_process = None
                    self._receiver_log_handle.close()
                    self._receiver_log_handle = None
                    self._log_handle.close()
                    self._log_handle = None
                    raise RuntimeError(
                        f"Unitree motion bridge failed readiness check: exit {exit_code}"
                    )
                command = self._launch_arguments(
                    candidate, site_id=self.site_id, robot_id=self.robot_id,
                    sensor_id=self.sensor_id, log_root=self.log_root,
                )
                self._process = subprocess.Popen(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=self._log_handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env=dict(os.environ),
                )
        return self.status()

    def _service(self, service: str, type_name: str, request: str, timeout: int = 12) -> dict[str, Any]:
        completed = subprocess.run(
            ["ros2", "service", "call", service, type_name, request],
            check=False, capture_output=True, text=True, timeout=timeout,
        )
        output = (completed.stdout + completed.stderr).strip()
        if completed.returncode != 0:
            raise RuntimeError(output or f"service call failed: {service}")
        success = "success=True" in output or "success: true" in output.lower()
        return {"success": success, "output": output}

    def start_patrol(self) -> dict[str, Any]:
        candidate = self._loaded_candidate()
        if not candidate:
            raise RuntimeError("navigation candidate is not prepared")
        if not self._process or self._process.poll() is not None:
            raise RuntimeError("navigation runtime is not running")
        if not self._receiver_process or self._receiver_process.poll() is not None:
            raise RuntimeError("Unitree motion bridge is not running")
        request = (
            "{expected_map_version: '" + candidate["map_version"] +
            "', expected_route_id: '" + candidate["route_id"] + "'}"
        )
        result = self._service(
            "/go2/patrol/start", "go2_nav2_interfaces/srv/StartPatrol", request
        )
        if not result["success"]:
            raise RuntimeError(result["output"] or "patrol start was rejected")
        return result

    def stop_patrol(self) -> dict[str, Any]:
        result: dict[str, Any] = {"success": True, "output": "runtime is not active"}
        if self._process and self._process.poll() is None:
            result = self._service("/go2/patrol/stop", "std_srvs/srv/Trigger", "{}")
        subprocess.run(
            ["ros2", "run", "go2_cmd_vel_bridge", "go2_sdk2_motion_probe", "--iface", "eth0", "stop"],
            check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=12,
        )
        return result

    def reset_localization(self) -> dict[str, Any]:
        return self._service("/localization/reset", "std_srvs/srv/Trigger", "{}")

    def stop_runtime(self) -> dict[str, Any]:
        with self._lock:
            process = self._process
            receiver_process = self._receiver_process
        if process and process.poll() is None:
            try:
                self.stop_patrol()
            except Exception:
                pass
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        if receiver_process and receiver_process.poll() is None:
            os.killpg(receiver_process.pid, signal.SIGINT)
            try:
                receiver_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(receiver_process.pid, signal.SIGTERM)
                receiver_process.wait(timeout=3)
        with self._lock:
            self._process = None
            self._receiver_process = None
            if self._log_handle:
                self._log_handle.close()
                self._log_handle = None
            if self._receiver_log_handle:
                self._receiver_log_handle.close()
                self._receiver_log_handle = None
        return self.status()
