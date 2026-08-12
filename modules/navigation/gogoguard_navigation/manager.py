from __future__ import annotations

import hashlib
import json
import math
import os
import re
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

from gogoguard_contracts import is_safe_external_id
from gogoguard_route import RouteManager
from .profiles import DEFAULT_PROFILE, NavigationProfileStore, validate_profile


SAFE_ID = re.compile(r"^map-[A-Za-z0-9]{12}$")
SDK_RECEIVER = Path(
    "/opt/gogoguard/ros_ws/install/lib/go2_cmd_vel_bridge/"
    "go2_sdk2_udp_receiver"
)
SDK_MOTION_PROBE = Path(
    "/opt/gogoguard/ros_ws/install/lib/go2_cmd_vel_bridge/"
    "go2_sdk2_motion_probe"
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
        self.mission_path = self.root / "mission-plan.json"
        self.checkpoint_control_path = (
            self.data_root / "platform" / "checkpoint-control.json"
        )
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
        self._launched_mission_hash: str | None = None
        self._lock = threading.Lock()

    def close(self) -> None:
        self.stop_runtime()

    def prepare(self, job_id: str) -> dict[str, Any]:
        candidate = self.routes.prepare_map_job(job_id)
        with self._lock:
            # A mission is an execution-time binding, not part of the selected
            # map release.  A newly published route must never inherit a task
            # written for an older revision.
            mission_path = getattr(self, "mission_path", None)
            mission = self._read_json(mission_path) if mission_path else None
            if mission_path and Path(mission_path).exists() and (
                mission is None
                or not self._mission_matches_candidate(mission, candidate)
            ):
                Path(mission_path).unlink(missing_ok=True)
            self._candidate = candidate
            self._atomic_json(self.selected_path, candidate)
        return candidate

    @staticmethod
    def _mission_matches_candidate(
        mission: dict[str, Any], candidate: dict[str, Any]
    ) -> bool:
        return bool(
            mission.get("schema") == "gogoguard.navigation_mission.v1"
            and mission.get("missionHash")
            and mission.get("mapVersion") == candidate.get("map_version")
            and mission.get("routeId") == candidate.get("route_id")
        )

    def _mission_path_for_launch(
        self,
        candidate: dict[str, Any],
        mission: dict[str, Any] | None,
    ) -> Path | None:
        # Plain localization/Nav2 startup is intentionally mission-free.  A
        # mission is loaded only by the selected-patrol operation that just
        # validated and persisted that exact map/route-bound payload.
        if mission is None:
            return None
        if not self._mission_matches_candidate(mission, candidate):
            raise RuntimeError("navigation mission map or route binding is invalid")
        mission_path = getattr(self, "mission_path", None)
        persisted = self._read_json(mission_path) if mission_path else None
        if (
            persisted is None
            or not self._mission_matches_candidate(persisted, candidate)
            or persisted.get("missionHash") != mission.get("missionHash")
        ):
            raise RuntimeError("navigation mission was not persisted coherently")
        return Path(mission_path)

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

    @staticmethod
    def _validate_runtime_candidate(candidate: dict[str, Any]) -> None:
        required = {
            "allowed_area_mask",
            "allowed_area_mask_image",
            "allowed_area_mask_hash",
            "allowed_area_mask_image_hash",
            "navigation_map",
            "navigation_map_image",
            "navigation_map_hash",
            "navigation_map_image_hash",
        }
        missing = sorted(key for key in required if not candidate.get(key))
        try:
            generation = int(candidate.get("candidate_generation") or 0)
        except (TypeError, ValueError):
            generation = 0
        if generation < 9 or missing:
            raise RuntimeError(
                "当前地图与路线是旧版本，还没有经过复核的静态导航地图；"
                "请在 Mac 工作台确认红色障碍和绿色可走区后，"
                "重新点击“发布所选地图与路线到机器狗”"
            )

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
                          sensor_id: str, log_root: Path, profile: dict[str, Any] | None = None,
                          mission_plan_path: Path | None = None) -> list[str]:
        profile = profile or validate_profile(DEFAULT_PROFILE)
        motion = profile["motion"]
        avoidance = profile["avoidance"]
        recovery = profile["recovery"]
        localization = profile["localization"]
        controller = profile["controller"]
        arguments = [
            "ros2", "launch", "go2_nav2_runtime", "active_map_patrol.launch.py",
            "runtime_source:=candidate",
            f"map_store_root:={candidate['localization_map']}",
            f"site_id:={site_id}",
            f"expected_map_version:={candidate['map_version']}",
            f"candidate_localization_map:={candidate['localization_map']}",
            f"candidate_route:={candidate['route']}",
            f"candidate_runtime_profile:={candidate['runtime_profile']}",
            f"candidate_allowed_area_mask:={candidate['allowed_area_mask']}",
            f"candidate_allowed_area_mask_image:={candidate['allowed_area_mask_image']}",
            f"localization_map_hash:={candidate['localization_map_hash']}",
            f"route_hash:={candidate['route_hash']}",
            f"runtime_profile_hash:={candidate['runtime_profile_hash']}",
            f"allowed_area_mask_hash:={candidate['allowed_area_mask_hash']}",
            f"allowed_area_mask_image_hash:={candidate['allowed_area_mask_image_hash']}",
            "hardware_output_enabled:=true",
            "sdk_receiver_enabled:=false",
            "sdk_interface:=eth0",
            f"runtime_log_dir:={log_root}",
            f"robot_id:={robot_id}",
            f"sensor_id:={sensor_id}",
            f"target_cruise_mps:={motion['targetCruiseMps']}",
            f"max_forward_mps:={motion['maxForwardMps']}",
            f"turn_speed_radps:={motion['turnSpeedRadps']}",
            f"lateral_speed_mps:={motion['lateralSpeedMps']}",
            f"acceleration_mps2:={motion['accelerationMps2']}",
            f"deceleration_mps2:={motion['decelerationMps2']}",
            f"rejoin_lookahead_m:={avoidance['rejoinLookaheadM']}",
            f"obstruction_cost_threshold:={avoidance['obstructionCostThreshold']}",
            f"obstruction_min_samples:={avoidance['obstructionMinSamples']}",
            f"obstruction_confirmation_s:={avoidance['obstructionConfirmationS']}",
            f"progress_timeout_s:={recovery['progressTimeoutS']}",
            f"replan_interval_s:={recovery['replanIntervalS']}",
            f"localization_status_timeout_s:={localization['statusTimeoutS']}",
            f"localization_dropout_grace_s:={localization['dropoutGraceS']}",
            f"localization_recovery_stable_s:={localization['recoveryStableS']}",
            f"controller_frequency_hz:={controller['frequencyHz']}",
            f"mppi_time_steps:={controller['timeSteps']}",
            f"mppi_batch_size:={controller['batchSize']}",
            f"mppi_iteration_count:={controller['iterationCount']}",
        ]
        if candidate.get("navigation_map"):
            arguments.extend(
                [
                    f"candidate_navigation_map:={candidate['navigation_map']}",
                    f"candidate_navigation_map_image:={candidate['navigation_map_image']}",
                    f"navigation_map_hash:={candidate['navigation_map_hash']}",
                    f"navigation_map_image_hash:={candidate['navigation_map_image_hash']}",
                ]
            )
        if mission_plan_path is not None:
            arguments.append(f"mission_plan_path:={mission_plan_path}")
        return arguments

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
            "malformed launch argument",
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

    def start_runtime(
        self,
        candidate_id: str,
        *,
        mission: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        if not SAFE_ID.fullmatch(candidate_id):
            raise ValueError("invalid navigation candidate id")
        candidate = self.routes.get(candidate_id)
        self._validate_runtime_candidate(candidate)
        mission_plan_path = self._mission_path_for_launch(candidate, mission)
        mission_hash = str((mission or {}).get("missionHash") or "")
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
                    if mission is not None and self._launched_mission_hash != mission_hash:
                        raise RuntimeError(
                            "navigation runtime is already active with another mission"
                        )
                else:
                    raise RuntimeError("another navigation runtime is already active")
            if not already_running:
                # status.json belongs to the observer inside one Nav2 runtime
                # generation.  Keeping the previous file while a replacement
                # process boots lets readiness checks mistake old localization
                # and costmap health for the new generation.
                status_path = getattr(self, "status_path", None)
                if status_path is not None:
                    Path(status_path).unlink(missing_ok=True)
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
                    mission_plan_path=mission_plan_path,
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
                # Starting the process and proving that localization/costmaps
                # are ready are separate lifecycle boundaries.  Catch an
                # immediate launch/configuration failure here; the selected-
                # patrol operation below owns the generation, localization and
                # post-clear costmap readiness checks before motion is allowed.
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
                self._launched_mission_hash = mission_hash
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
            current_sequence = int(health.get("sequence") or 0)
            if current_sequence > before_sequence:
                last_reason = str(health.get("reason") or "COSTMAP_UNHEALTHY")
                if health.get("healthy") is True:
                    break
            else:
                # A pre-clear healthy=OK value is not evidence that the clear
                # completed.  Name the missing freshness proof explicitly.
                last_reason = "COSTMAP_REFRESH_PENDING"
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

    def start_selected_patrol(
        self,
        *,
        expected_map_version: str | None = None,
        expected_route_id: str | None = None,
        mission_plan: dict[str, Any] | None = None,
        readiness_timeout_s: float = 180.0,
    ) -> dict[str, Any]:
        """Start the selected release, wait for real readiness, then patrol.

        This is the single platform-facing lifecycle operation. It never
        accepts a path or velocity from the platform: map and route selection
        still belongs to the commissioned field-workstation release flow.
        """
        candidate = self._loaded_candidate()
        if not candidate:
            raise RuntimeError("尚未发布可用的地图与路线，无法从平台开始巡检")
        expected_map_version = str(expected_map_version or "").strip()
        expected_route_id = str(expected_route_id or "").strip()
        if expected_map_version and expected_map_version != candidate.get("map_version"):
            raise RuntimeError("平台任务地图版本与机器狗已选版本不一致")
        if expected_route_id and expected_route_id != candidate.get("route_id"):
            raise RuntimeError("平台任务路线版本与机器狗已选版本不一致")
        if not 5.0 <= float(readiness_timeout_s) <= 300.0:
            raise ValueError("navigation readiness timeout is outside 5..300 seconds")

        mission = self._configure_mission(candidate, mission_plan)
        status = self.status()
        previous_runtime = status.get("runtime")
        previous_runtime = previous_runtime if isinstance(previous_runtime, dict) else {}
        previous_runtime_instance_id = str(
            previous_runtime.get("runtimeInstanceId") or ""
        )
        require_new_runtime_generation = False
        if (
            status.get("runtime_process", {}).get("running")
            and self._launched_mission_hash != mission["missionHash"]
        ):
            self.stop_runtime()
            status = self.status()
        if not status.get("runtime_process", {}).get("running"):
            require_new_runtime_generation = True
            self.start_runtime(str(candidate["candidate_id"]), mission=mission)

        deadline = time.monotonic() + float(readiness_timeout_s)
        last_localization_reason = "LOCALIZATION_MISSING"
        last_costmap_reason = "COSTMAP_MISSING"
        while time.monotonic() < deadline:
            status = self.status()
            if not status.get("runtime_process", {}).get("running"):
                raise RuntimeError("定位与 Nav2 启动后退出，未向机器狗提交巡检")
            if not status.get("motion_bridge", {}).get("running"):
                raise RuntimeError("Unitree 运动桥未就绪，未向机器狗提交巡检")
            localization = status.get("localization")
            localization = localization if isinstance(localization, dict) else {}
            runtime = status.get("runtime")
            runtime = runtime if isinstance(runtime, dict) else {}
            runtime_instance_id = str(runtime.get("runtimeInstanceId") or "")
            if require_new_runtime_generation and (
                not runtime_instance_id
                or runtime_instance_id == previous_runtime_instance_id
            ):
                last_localization_reason = "NAV2_RUNTIME_GENERATION_PENDING"
                last_costmap_reason = "NAV2_RUNTIME_GENERATION_PENDING"
                time.sleep(0.1)
                continue
            health = runtime.get("costmapHealth")
            health = health if isinstance(health, dict) else {}
            last_localization_reason = str(
                localization.get("reason")
                or localization.get("state")
                or last_localization_reason
            )
            last_costmap_reason = str(health.get("reason") or last_costmap_reason)
            if localization.get("usable") is True and health.get("healthy") is True:
                result = self.start_patrol()
                return {
                    "schema": "gogoguard.selected_patrol_start.v1",
                    "mapVersion": candidate["map_version"],
                    "routeId": candidate["route_id"],
                    "missionId": mission["missionId"],
                    "checkpointCount": len(mission["checkpoints"]),
                    "accepted": True,
                    "nav2": result,
                }
            time.sleep(0.1)
        raise RuntimeError(
            "等待定位与代价地图就绪超时："
            f"localization={last_localization_reason}, costmap={last_costmap_reason}；"
            "未向机器狗提交巡检"
        )

    def _configure_mission(
        self,
        candidate: dict[str, Any],
        mission_plan: dict[str, Any] | None,
    ) -> dict[str, Any]:
        if mission_plan is None:
            mission_plan = {}
        if not isinstance(mission_plan, dict):
            raise ValueError("missionPlan must be an object")
        map_version = str(mission_plan.get("mapVersion") or candidate["map_version"])
        route_id = str(mission_plan.get("routeId") or candidate["route_id"])
        if map_version != candidate["map_version"] or route_id != candidate["route_id"]:
            raise RuntimeError("平台点位任务与机器狗已选地图路线不一致")
        mission_id = str(
            mission_plan.get("missionId")
            or f"selected:{candidate['map_version']}:{candidate['workspace_revision']}"
        )
        if not is_safe_external_id(mission_id):
            raise ValueError("missionId is invalid")
        decision_mode = str(mission_plan.get("decisionMode") or "platform")
        if decision_mode not in {"platform", "local_operator"}:
            raise ValueError("decisionMode must be platform or local_operator")
        checkpoints = mission_plan.get("checkpoints", [])
        if not isinstance(checkpoints, list) or len(checkpoints) > 256:
            raise ValueError("mission checkpoints must be a list of at most 256 items")
        route_point_count = int(candidate.get("execution_route_point_count") or 0)
        normalized: list[dict[str, Any]] = []
        asset_by_id: dict[str, dict[str, Any]] = {}
        checkpoint_asset = candidate.get("checkpoint_asset")
        if checkpoint_asset and Path(str(checkpoint_asset)).is_file():
            asset = json.loads(Path(str(checkpoint_asset)).read_text(encoding="utf-8"))
            if (
                asset.get("schema") != "gogoguard.checkpoints.v1"
                or asset.get("mapVersion") != map_version
                or asset.get("routeId") != route_id
            ):
                raise ValueError("checkpoint asset map or route binding is invalid")
            asset_by_id = {
                str(item.get("checkpointId")): item
                for item in asset.get("checkpoints") or []
                if isinstance(item, dict)
            }
        seen: set[str] = set()
        previous_index = -1
        for raw in checkpoints:
            if not isinstance(raw, dict):
                raise ValueError("mission checkpoint must be an object")
            checkpoint_id = str(raw.get("checkpointId") or "")
            route_index = raw.get("routeProgressIndex")
            if not is_safe_external_id(checkpoint_id) or checkpoint_id in seen:
                raise ValueError("checkpointId is invalid or duplicated")
            if (
                isinstance(route_index, bool)
                or not isinstance(route_index, int)
                or route_index < previous_index
                or route_index < 0
                or route_index >= route_point_count
            ):
                raise ValueError("checkpoint routeProgressIndex is outside execution route")
            seen.add(checkpoint_id)
            previous_index = route_index
            recorded = asset_by_id.get(checkpoint_id)
            if checkpoint_asset and recorded is None:
                raise ValueError("mission checkpoint is absent from the activated asset")
            if recorded is not None and recorded.get("routeProgressIndex") != route_index:
                raise ValueError("mission checkpoint route index differs from the activated asset")
            if recorded is None:
                normalized.append(
                    {
                        "checkpointId": checkpoint_id,
                        "routeProgressIndex": route_index,
                        "action": "body_spin_360",
                        "settleBeforeS": 0.5,
                        "targetYawRad": round(2.0 * math.pi, 9),
                    }
                )
            else:
                camera = recorded.get("camera") if isinstance(recorded.get("camera"), dict) else {}
                body_yaw = float(recorded.get("bodyYaw") or 0.0)
                camera_angles = {
                    "pan": float(camera.get("pan") or 0.0),
                    "tilt": float(camera.get("tilt") or 0.0),
                    "roll": float(camera.get("roll") or 0.0),
                }
                if not math.isfinite(body_yaw) or any(
                    not math.isfinite(value) for value in camera_angles.values()
                ):
                    raise ValueError("activated checkpoint pose contains a non-finite angle")
                spin = raw.get("spin", recorded.get("spin", True))
                dwell = raw.get("dwellSec", recorded.get("dwellSec", 3))
                if not isinstance(spin, bool):
                    raise ValueError("mission checkpoint spin must be boolean")
                if (
                    isinstance(dwell, bool)
                    or not isinstance(dwell, (int, float))
                    or not math.isfinite(float(dwell))
                    or not 0.0 <= float(dwell) <= 30.0
                ):
                    raise ValueError("mission checkpoint dwellSec is invalid")
                normalized.append(
                    {
                        "checkpointId": checkpoint_id,
                        "routeProgressIndex": route_index,
                        "action": "platform_checkpoint",
                        "settleBeforeS": 0.5,
                        "bodyYawRad": body_yaw,
                        "camera": camera_angles,
                        "spin": spin,
                        "dwellSec": float(dwell),
                    }
                )
        verdict_timeout = mission_plan.get("verdictTimeoutSec", 15)
        max_retakes = mission_plan.get("maxRetakeAttempts", 2)
        if isinstance(verdict_timeout, bool) or not isinstance(verdict_timeout, int) or not 5 <= verdict_timeout <= 120:
            raise ValueError("verdictTimeoutSec must be an integer between 5 and 120")
        if isinstance(max_retakes, bool) or not isinstance(max_retakes, int) or not 0 <= max_retakes <= 5:
            raise ValueError("maxRetakeAttempts must be an integer between 0 and 5")
        canonical = {
            "schema": "gogoguard.navigation_mission.v1",
            "missionId": mission_id,
            "mapVersion": map_version,
            "routeId": route_id,
            "decisionMode": decision_mode,
            "verdictTimeoutSec": verdict_timeout,
            "maxRetakeAttempts": max_retakes,
            "checkpoints": normalized,
        }
        canonical["missionHash"] = hashlib.sha256(
            json.dumps(
                canonical, ensure_ascii=False, sort_keys=True, separators=(",", ":")
            ).encode("utf-8")
        ).hexdigest()
        self._atomic_json(self.mission_path, canonical)
        return canonical

    def checkpoint_control(self, action: str) -> dict[str, Any]:
        """Apply an operator verdict to an active local checkpoint only."""
        action = str(action or "").strip()
        if action not in {"capture", "continue", "retake", "skip"}:
            raise ValueError("checkpoint action is invalid")
        runtime = self.status().get("runtime")
        runtime = runtime if isinstance(runtime, dict) else {}
        checkpoint = runtime.get("checkpoint")
        checkpoint = checkpoint if isinstance(checkpoint, dict) else {}
        if checkpoint.get("decisionMode") != "local_operator":
            raise RuntimeError("当前不是本地人工验收任务")
        mission_id = str(checkpoint.get("missionId") or "")
        checkpoint_id = str(checkpoint.get("activeCheckpointId") or "")
        phase = str(checkpoint.get("phase") or "TRAVELING")
        attempt = int(checkpoint.get("attempt") or 0)
        if not mission_id or not checkpoint_id or attempt < 1:
            if action in {"continue", "skip"} and checkpoint.get(
                "lastCompletedCheckpointId"
            ):
                return {
                    "schema": "gogoguard.local_checkpoint_control_receipt.v1",
                    "accepted": True,
                    "duplicate": True,
                    "action": action,
                    "phase": phase,
                }
            raise RuntimeError("当前没有等待验收的巡检点")
        allowed_phases = {
            "capture": {"WAITING_PLATFORM"},
            "continue": {"WAITING_VERDICT"},
            "retake": {"WAITING_VERDICT"},
            "skip": {"WAITING_VERDICT"},
        }
        if phase not in allowed_phases[action]:
            if action == "capture" and phase in {
                "SPIN_REQUESTED", "SPINNING", "WAITING_VERDICT"
            }:
                return {
                    "schema": "gogoguard.local_checkpoint_control_receipt.v1",
                    "accepted": True,
                    "duplicate": True,
                    "action": action,
                    "phase": phase,
                    "missionId": mission_id,
                    "checkpointId": checkpoint_id,
                    "attempt": attempt,
                }
            raise RuntimeError(f"当前阶段 {phase} 不能执行 {action}")
        seed = f"{mission_id}:{checkpoint_id}:{attempt}:{action}"
        control = {
            "schema": "gogoguard.checkpoint_control.v1",
            "controlId": "local_" + hashlib.sha256(seed.encode()).hexdigest()[:28],
            "missionId": mission_id,
            "checkpointId": checkpoint_id,
            "attempt": attempt,
            "action": action,
            "issuedAt": time.time(),
        }
        self.checkpoint_control_path.parent.mkdir(parents=True, exist_ok=True)
        self._atomic_json(self.checkpoint_control_path, control)
        return {
            "schema": "gogoguard.local_checkpoint_control_receipt.v1",
            "accepted": True,
            "duplicate": False,
            "action": action,
            "phase": phase,
            "missionId": mission_id,
            "checkpointId": checkpoint_id,
            "attempt": attempt,
        }

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
            self._launched_mission_hash = None
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
                        str(SDK_MOTION_PROBE),
                        "--iface",
                        "eth0",
                        "stop",
                    ],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=12,
                    # The Unitree SDK2 receiver and one-shot probe must use
                    # the exact same paired native-library boundary.  Going
                    # through `ros2 run` inherits ROS libraries first and can
                    # abort in ChannelFactory before StopMove is sent.
                    env=self._receiver_environment(),
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
