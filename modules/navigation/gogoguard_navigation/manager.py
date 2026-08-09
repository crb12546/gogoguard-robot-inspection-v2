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
from .profiles import DEFAULT_PROFILE, NavigationProfileStore, validate_profile


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
        self.selected_path = self.root / "selected-candidate.json"
        self.root.mkdir(parents=True, exist_ok=True)
        self.log_root.mkdir(parents=True, exist_ok=True)
        self.routes = RouteManager(self.data_root, site_id=site_id)
        self.profiles = NavigationProfileStore(self.data_root)
        self._process: subprocess.Popen | None = None
        self._log_handle = None
        self._receiver_process: subprocess.Popen | None = None
        self._receiver_log_handle = None
        self._last_runtime_exit: int | None = None
        self._last_receiver_exit: int | None = None
        self._candidate: dict[str, Any] | None = self._read_json(self.selected_path)
        self._lock = threading.Lock()

    def close(self) -> None:
        self.stop_runtime()

    def prepare(self, job_id: str) -> dict[str, Any]:
        candidate = self.routes.prepare_map_job(job_id)
        with self._lock:
            self._candidate = candidate
            self._atomic_json(self.selected_path, candidate)
        return candidate

    @staticmethod
    def _read_json(path: Path) -> dict[str, Any] | None:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            return value if isinstance(value, dict) else None
        except (OSError, ValueError):
            return None

    @staticmethod
    def _atomic_json(path: Path, value: dict[str, Any]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        os.replace(temporary, path)

    def profile(self) -> dict[str, Any]:
        return self.profiles.get()

    def update_profile(self, profile: dict[str, Any]) -> dict[str, Any]:
        if self._process and self._process.poll() is None:
            raise RuntimeError("请先停止巡检并关闭定位与 Nav2，再保存参数")
        return self.profiles.update(profile)

    def rollback_profile(self) -> dict[str, Any]:
        if self._process and self._process.poll() is None:
            raise RuntimeError("请先停止巡检并关闭定位与 Nav2，再回滚参数")
        return self.profiles.rollback()

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
        process_exit = (
            None if process_running
            else self._process.returncode if self._process
            else self._last_runtime_exit
        )
        receiver_running = bool(
            self._receiver_process and self._receiver_process.poll() is None
        )
        receiver_exit = (
            None if receiver_running
            else self._receiver_process.returncode if self._receiver_process
            else self._last_receiver_exit
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
                          sensor_id: str, log_root: Path, profile: dict[str, Any] | None = None) -> list[str]:
        profile = profile or validate_profile(DEFAULT_PROFILE)
        motion = profile["motion"]
        avoidance = profile["avoidance"]
        recovery = profile["recovery"]
        localization = profile["localization"]
        controller = profile["controller"]
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
            f"straight_speed_mps:={motion['straightSpeedMps']}",
            f"detour_speed_mps:={motion['detourSpeedMps']}",
            f"turn_speed_radps:={motion['turnSpeedRadps']}",
            f"lateral_speed_mps:={motion['lateralSpeedMps']}",
            f"acceleration_mps2:={motion['accelerationMps2']}",
            f"stop_zone_front_m:={avoidance['stopZoneFrontM']}",
            f"stop_zone_rear_m:={avoidance['stopZoneRearM']}",
            f"stop_zone_half_width_m:={avoidance['stopZoneHalfWidthM']}",
            f"slow_zone_front_m:={avoidance['slowZoneFrontM']}",
            f"slow_zone_rear_m:={avoidance['slowZoneRearM']}",
            f"slow_zone_half_width_m:={avoidance['slowZoneHalfWidthM']}",
            f"slowdown_ratio:={avoidance['slowdownRatio']}",
            f"rejoin_lookahead_m:={avoidance['rejoinLookaheadM']}",
            f"obstruction_cost_threshold:={avoidance['obstructionCostThreshold']}",
            f"obstruction_min_samples:={avoidance['obstructionMinSamples']}",
            f"obstruction_confirmation_s:={avoidance['obstructionConfirmationS']}",
            f"progress_timeout_s:={recovery['progressTimeoutS']}",
            f"mppi_retry_limit:={recovery['mppiRetryLimit']}",
            f"detour_attempt_limit:={recovery['detourAttemptLimit']}",
            f"localization_status_timeout_s:={localization['statusTimeoutS']}",
            f"localization_dropout_grace_s:={localization['dropoutGraceS']}",
            f"localization_recovery_stable_s:={localization['recoveryStableS']}",
            f"controller_frequency_hz:={controller['frequencyHz']}",
            f"mppi_time_steps:={controller['timeSteps']}",
            f"mppi_batch_size:={controller['batchSize']}",
            f"mppi_iteration_count:={controller['iterationCount']}",
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

    @staticmethod
    def _terminate_process(
        process: subprocess.Popen | None,
        *,
        interrupt_timeout: float,
        terminate_timeout: float,
    ) -> int | None:
        if not process:
            return None
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=interrupt_timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=terminate_timeout)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=3)
        return process.returncode

    @staticmethod
    def _runtime_failure_detail(log_path: Path) -> str:
        try:
            lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            return ""
        markers = (
            "caught exception in launch",
            "invalid_",
            "process has died",
            "traceback",
            "fatal",
            "error",
        )
        for line in reversed(lines[-200:]):
            if any(marker in line.lower() for marker in markers):
                return line.strip()[-600:]
        return ""

    def _reap_runtime_generation(
        self,
        process: subprocess.Popen,
        receiver_process: subprocess.Popen,
        log_handle: Any,
        receiver_log_handle: Any,
    ) -> None:
        runtime_exit = process.wait()
        with self._lock:
            if self._process is not process:
                return
            # Keep the generation owned while the bridge is being terminated;
            # otherwise a concurrent restart can race the old UDP 5005 socket.
            receiver_exit = self._terminate_process(
                receiver_process, interrupt_timeout=5, terminate_timeout=3
            )
            self._process = None
            self._receiver_process = None
            self._log_handle = None
            self._receiver_log_handle = None
            self._last_runtime_exit = runtime_exit
            self._last_receiver_exit = receiver_exit
        log_handle.close()
        receiver_log_handle.close()

    def start_runtime(self, candidate_id: str) -> dict[str, Any]:
        if not SAFE_ID.fullmatch(candidate_id):
            raise ValueError("invalid navigation candidate id")
        candidate = self.routes.get(candidate_id)
        already_running = False
        stale_generation = bool(
            (self._process and self._process.poll() is not None)
            or (self._receiver_process and self._receiver_process.poll() is None)
        )
        if stale_generation:
            self.stop_runtime()
        with self._lock:
            if self._process and self._process.poll() is None:
                if self._candidate and self._candidate.get("candidate_id") == candidate_id:
                    already_running = True
                else:
                    raise RuntimeError("another navigation runtime is already active")
            if not already_running:
                self._candidate = candidate
                self._last_runtime_exit = None
                self._last_receiver_exit = None
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
                    profile=self.profiles.get(),
                )
                self._process = subprocess.Popen(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=self._log_handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env=dict(os.environ),
                )
                process = self._process
                receiver_process = self._receiver_process
                log_handle = self._log_handle
                receiver_log_handle = self._receiver_log_handle
                # A launch process that exits immediately is not a successful
                # runtime start. This catches contract/configuration failures
                # before the asynchronous operation is marked complete.
                time.sleep(1.25)
                if process.poll() is not None:
                    exit_code = process.returncode
                    self._process = None
                    self._receiver_process = None
                    self._log_handle = None
                    self._receiver_log_handle = None
                    self._last_runtime_exit = exit_code
                    receiver_exit = self._terminate_process(
                        receiver_process, interrupt_timeout=5, terminate_timeout=3
                    )
                    self._last_receiver_exit = receiver_exit
                    log_handle.close()
                    receiver_log_handle.close()
                    detail = self._runtime_failure_detail(log_path)
                    suffix = f": {detail}" if detail else ""
                    raise RuntimeError(
                        f"Nav2 failed readiness check: exit {exit_code}{suffix}"
                    )
                threading.Thread(
                    target=self._reap_runtime_generation,
                    args=(process, receiver_process, log_handle, receiver_log_handle),
                    name=f"navigation-reaper-{candidate_id}",
                    daemon=True,
                ).start()
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
        before = self.status()
        before_health = (before.get("runtime") or {}).get("costmapHealth") or {}
        before_sequence = int(before_health.get("sequence") or 0)
        self._service(
            "/local_costmap/clear_entirely_local_costmap",
            "nav2_msgs/srv/ClearEntireCostmap",
            "{}",
        )
        deadline = time.monotonic() + 4.0
        last_reason = "COSTMAP_MISSING"
        while time.monotonic() < deadline:
            current = self.status()
            health = (current.get("runtime") or {}).get("costmapHealth") or {}
            last_reason = str(health.get("reason") or last_reason)
            if (
                int(health.get("sequence") or 0) > before_sequence
                and health.get("healthy") is True
            ):
                break
            if not current.get("runtime_process", {}).get("running"):
                raise RuntimeError("导航运行进程已退出，无法开始巡检")
            time.sleep(0.05)
        else:
            raise RuntimeError(
                f"代价地图清理后未恢复健康：{last_reason}；未向机器狗提交巡检"
            )
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

    def diagnostics(self) -> dict[str, Any]:
        status = self.status()
        runtime = status.get("runtime") or {}
        localization = status.get("localization") or {}
        safety = status.get("safety") or {}
        operations: list[dict[str, Any]] = []
        for path in sorted(self.log_root.glob("*.log"), key=lambda item: item.stat().st_mtime, reverse=True)[:8]:
            try:
                tail = path.read_text(encoding="utf-8", errors="replace").splitlines()[-80:]
            except OSError:
                continue
            interesting = [line for line in tail if any(
                token in line.lower() for token in ("error", "fail", "fault", "timeout", "reject", "blocked")
            )]
            operations.append({"name": path.name, "updatedAt": path.stat().st_mtime, "highlights": interesting[-8:]})
        checks = [
            {"name": "唯一导航进程", "ok": bool(status["runtime_process"]["running"]),
             "detail": "运行中" if status["runtime_process"]["running"] else "尚未启动"},
            {"name": "Unitree 运动桥", "ok": bool(status["motion_bridge"]["running"]),
             "detail": "运行中" if status["motion_bridge"]["running"] else "尚未启动"},
            {"name": "固定地图定位", "ok": bool(localization.get("usable")),
             "detail": str(localization.get("reason") or localization.get("state") or "无数据")},
            {"name": "巡检任务", "ok": runtime.get("state") not in {"FAULT", "BLOCKED"},
             "detail": str(runtime.get("operatorMessage") or runtime.get("reason") or "未开始")},
            {"name": "最终运动链", "ok": safety.get("stopReason", safety.get("stop_reason", "normal")) in {None, "normal"},
             "detail": str(safety.get("stopReason") or safety.get("stop_reason") or "normal")},
        ]
        runtime_state = str(runtime.get("state") or "")
        if runtime_state == "BLOCKED":
            summary = "巡检受阻"
        elif runtime_state == "FAULT":
            summary = "导航故障"
        elif not status["runtime_process"]["running"]:
            summary = "导航未启动"
        elif not localization.get("usable"):
            summary = "等待定位"
        elif runtime_state in {
            "STARTING", "PATROLLING", "REPLANNING", "DETOURING",
            "REJOINING", "RETRYING", "HOLDING", "RESUMING",
        }:
            summary = "巡检运行中"
        elif runtime_state == "COMPLETED":
            summary = "巡检已完成"
        else:
            summary = "可开始巡检"
        return {
            "schema": "gogoguard.navigation_diagnostics.v1",
            "generatedAt": time.time(),
            "summary": summary,
            "checks": checks,
            "logs": operations,
            "profile": self.profile(),
        }

    def stop_patrol(self) -> dict[str, Any]:
        # Operator stop means relinquishing the SDK motion owner, not merely
        # cancelling a FollowPath goal while a zero-command publisher keeps
        # competing with the handheld remote.
        return self.stop_runtime()

    def reset_localization(self) -> dict[str, Any]:
        return self._service("/localization/reset", "std_srvs/srv/Trigger", "{}")

    def recover_runtime(self) -> dict[str, Any]:
        candidate = self._loaded_candidate()
        if not candidate:
            raise RuntimeError("尚未选择地图与路线")
        self.stop_runtime()
        return self.start_runtime(str(candidate["candidate_id"]))

    def stop_runtime(self) -> dict[str, Any]:
        with self._lock:
            process = self._process
            receiver_process = self._receiver_process
            log_handle = self._log_handle
            receiver_log_handle = self._receiver_log_handle
            self._process = None
            self._receiver_process = None
            self._log_handle = None
            self._receiver_log_handle = None
        if process and process.poll() is None:
            try:
                self._service("/go2/patrol/stop", "std_srvs/srv/Trigger", "{}")
            except Exception:
                pass
        runtime_exit = self._terminate_process(
            process, interrupt_timeout=12, terminate_timeout=3
        )
        receiver_exit = self._terminate_process(
            receiver_process, interrupt_timeout=5, terminate_timeout=3
        )
        probe_returncode = 0
        if process is not None or receiver_process is not None:
            try:
                probe = subprocess.run(
                    [
                        "ros2",
                        "run",
                        "go2_cmd_vel_bridge",
                        "go2_sdk2_motion_probe",
                        "--iface",
                        "eth0",
                        "stop",
                    ],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=12,
                )
                probe_returncode = probe.returncode
            except (OSError, subprocess.TimeoutExpired):
                probe_returncode = -1
        with self._lock:
            self._last_runtime_exit = runtime_exit
            self._last_receiver_exit = receiver_exit
        if log_handle:
            log_handle.close()
        if receiver_log_handle:
            receiver_log_handle.close()
        # Verify the process objects themselves. They were deliberately
        # detached from manager status before termination so a new operation
        # cannot treat this generation as owned; using status here would make
        # every termination look successful even if a child survived.
        runtime_stopped = process is None or process.poll() is not None
        bridge_stopped = (
            receiver_process is None or receiver_process.poll() is not None
        )
        remote_control_released = runtime_stopped and bridge_stopped
        return {
            "schema": "gogoguard.motion_release.v1",
            "success": remote_control_released,
            "patrolCancelled": True,
            "runtimeStopped": runtime_stopped,
            "motionBridgeStopped": bridge_stopped,
            "stopMoveConfirmed": probe_returncode == 0,
            "remoteControlReleased": remote_control_released,
            "runtimeExitCode": runtime_exit,
            "motionBridgeExitCode": receiver_exit,
        }
