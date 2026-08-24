from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

from gogoguard_contracts import json_ready
from gogoguard_data_capture import CaptureManager
from gogoguard_device_io import SnapshotStore, Z1ProGimbal, create_camera_gateway, create_gateway
from gogoguard_evidence import DiagnosticProfileStore, EventJournal, IncidentStore
from gogoguard_map_factory import MapJobManager
from gogoguard_navigation import NavigationManager, NavigationSupervisorClient
from gogoguard_route import NavigationWorkspaceStore, RouteManager
from gogoguard_transfer import EdgeArtifactExchange


class InspectionApplication:
    def __init__(self, *, data_root: Path, mode: str, map_worker: str, robot_id: str, site_id: str,
                 topics: dict[str, str], sensor_id: str = "ARMCP6B0035634", cloud: dict | None = None,
                 camera: dict | None = None, capabilities: dict | None = None,
                 gimbal: dict | None = None) -> None:
        data_root.mkdir(parents=True, exist_ok=True)
        self.data_root = data_root
        self.mode = mode
        self.robot_id = robot_id
        self.site_id = site_id
        self.sensor_id = sensor_id
        self.capability_profile = dict(capabilities or {})
        self.store = SnapshotStore(robot_id, mode, data_root)
        self.journal = EventJournal(data_root / "events" / "runtime.jsonl")
        self.diagnostic_profiles = DiagnosticProfileStore(data_root)
        self.incident_store = IncidentStore(data_root)
        self.gateway = create_gateway(mode, self.store, robot_id, topics)
        self.camera = create_camera_gateway(mode, camera)
        gimbal_config = dict(gimbal or {})
        self.gimbal = Z1ProGimbal(
            host=str(gimbal_config.get("host") or "192.168.144.108"),
            port=int(gimbal_config.get("port") or 2332),
            timeout_s=float(gimbal_config.get("timeout_s") or 1.5),
            command_hz=float(gimbal_config.get("command_hz") or 40.0),
            move_timeout_s=float(gimbal_config.get("move_timeout_s") or 3.0),
            angle_tolerance_deg=float(
                gimbal_config.get("angle_tolerance_deg") or 2.0
            ),
            confirmation_samples=int(gimbal_config.get("confirmation_samples") or 3),
            commissioned=mode == "demo" or bool(gimbal_config.get("commissioned", False)),
        )
        self._gimbal_angles = {"pan": 0.0, "tilt": 0.0, "roll": 0.0}
        self._gimbal_lock = threading.RLock()
        self.capture = CaptureManager(data_root, self.store, self.journal, mode, topics)
        self.maps = None if map_worker == "none" else MapJobManager(
            data_root, map_worker, self.journal, cloud
        )
        self.exchange = EdgeArtifactExchange(data_root)
        self.navigation_workspaces = NavigationWorkspaceStore(
            data_root, sensor_id=sensor_id
        )
        self.routes = RouteManager(data_root, site_id=site_id, sensor_id=sensor_id)
        self.navigation = (
            NavigationSupervisorClient(data_root / "navigation" / "supervisor.sock")
            if mode == "robot"
            else NavigationManager(
                data_root, site_id=site_id, robot_id=robot_id, sensor_id=sensor_id
            )
        )

    def start(self) -> None:
        self.gateway.start()
        self.journal.append("runtime.started", mode=self.mode, robot_id=self.robot_id)

    def close(self) -> None:
        self.capture.close()
        self.navigation.close()
        self.gateway.stop()
        self.journal.append("runtime.stopped", mode=self.mode, robot_id=self.robot_id)

    def status(self) -> dict:
        snapshot = self.store.get()
        active = self.capture.active()
        return {
            "schema": "gogoguard.runtime_status.v1",
            "runtime": {"mode": self.mode, "robot_id": self.robot_id, "site_id": self.site_id},
            "device": json_ready(snapshot.device),
            "recording": json_ready(active) if active else None,
        }

    def live(self) -> dict:
        return json_ready(self.store.get())

    def camera_status(self) -> dict:
        return json_ready(self.camera.status())

    def gimbal_status(self) -> dict:
        with self._gimbal_lock:
            value = {**self.gimbal.capability(), "angles": dict(self._gimbal_angles)}
            if self.mode == "demo":
                return value | {"online": True}
            try:
                reply = self.gimbal.probe()
                angles = reply.angle_control_feedback()
                if all(item is not None for item in angles.values()):
                    self._gimbal_angles = {key: float(item) for key, item in angles.items()}
                return value | {
                    "online": True,
                    "angles": dict(self._gimbal_angles),
                    "feedbackFrames": {
                        "pan": "carrier_relative_z",
                        "tilt": "absolute_euler_pitch",
                        "roll": "absolute_euler_roll",
                    },
                    "relativeAngles": {
                        "x": reply.relative_roll_deg,
                        "y": reply.relative_tilt_deg,
                        "z": reply.relative_pan_deg,
                    },
                    "absoluteAngles": {
                        "roll": reply.absolute_roll_deg,
                        "pitch": reply.absolute_pitch_deg,
                        "yaw": reply.absolute_yaw_deg,
                    },
                }
            except Exception as exc:
                return value | {"online": False, "reason": type(exc).__name__}

    def move_gimbal(self, payload: dict) -> dict:
        with self._gimbal_lock:
            angles = self._validated_gimbal_angles(payload)
            precision = payload.get("precision")
            if precision not in {None, "fine"}:
                raise ValueError("gimbal precision must be fine when provided")
            tolerance_deg = 0.5 if precision == "fine" else None
            pan, tilt, roll = angles["pan"], angles["tilt"], angles["roll"]
            try:
                if self.mode == "demo":
                    actual = {"pan": pan, "tilt": tilt, "roll": roll}
                else:
                    reply = self.gimbal.move_for_inspection(
                        pan_body_deg=pan,
                        tilt_euler_deg=tilt,
                        roll_euler_deg=roll,
                        tolerance_deg=tolerance_deg,
                    )
                    feedback = reply.angle_control_feedback()
                    if any(value is None for value in feedback.values()):
                        raise RuntimeError(
                            "Z1Pro GCU response contains no angle-control feedback"
                        )
                    actual = {
                        name: float(value)
                        for name, value in feedback.items()
                    }
                self._gimbal_angles = actual
                self.journal.append(
                    "gimbal.move_converged",
                    target=angles,
                    actual=actual,
                    tolerance_deg=(
                        self.gimbal.angle_tolerance_deg
                        if tolerance_deg is None
                        else tolerance_deg
                    ),
                )
                return {
                    **self.gimbal.capability(),
                    "online": True,
                    "angles": dict(self._gimbal_angles),
                }
            except Exception as exc:
                self.journal.append(
                    "gimbal.move_failed",
                    target=angles,
                    error_type=type(exc).__name__,
                    error=str(exc),
                )
                raise

    def _validated_gimbal_angles(self, payload: dict) -> dict[str, float]:
        if not isinstance(payload, dict):
            raise ValueError("gimbal angles must be an object")
        result: dict[str, float] = {}
        for name, bounds in (
            ("pan", self.gimbal.PAN_RANGE_DEG),
            ("tilt", self.gimbal.TILT_RANGE_DEG),
            ("roll", self.gimbal.ROLL_RANGE_DEG),
        ):
            raw = payload.get(name, self._gimbal_angles[name])
            if isinstance(raw, bool) or not isinstance(raw, (int, float)):
                raise ValueError(f"gimbal {name} angle must be numeric")
            value = float(raw)
            if not math.isfinite(value) or not bounds[0] <= value <= bounds[1]:
                raise ValueError(f"gimbal {name} angle is outside the supported range")
            result[name] = value
        return result

    def center_gimbal(self) -> dict:
        return self.move_gimbal(
            {
                "pan": 0.0,
                "tilt": 0.0,
                "roll": 0.0,
                "precision": "fine",
            }
        )

    def capabilities(self) -> dict:
        value = json_ready(self.capability_profile)
        if value.get("schema") != "gogoguard.robot_capabilities.v1":
            return {
                "schema": "gogoguard.robot_capabilities.v1",
                "revision": 0,
                "robotId": self.robot_id,
                "available": False,
                "reason": "capability_profile_unavailable",
            }
        return {**value, "robotId": self.robot_id, "available": True}

    def interaction_status(self) -> dict:
        path = self.data_root / "interaction" / "status.json"
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(value, dict) or value.get("schema") != "gogoguard.interaction_edge_status.v1":
                raise ValueError("invalid interaction status schema")
            return value
        except FileNotFoundError:
            return {
                "schema": "gogoguard.interaction_edge_status.v1",
                "enabled": False,
                "message": "实时对话服务尚未启用",
                "motionCommandsPermitted": False,
            }

    def platform_status(self) -> dict:
        path = self.data_root / "platform" / "status.json"
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(value, dict) or value.get("schema") != "gogoguard.platform_edge_status.v1":
                raise ValueError("invalid platform status schema")
            return value
        except FileNotFoundError:
            return {
                "schema": "gogoguard.platform_edge_status.v1",
                "robotId": self.robot_id,
                "online": False,
                "enabled": False,
                "message": "GoGoGuard 平台心跳服务尚未启用",
                "motionCommandsPermitted": False,
            }

    def start_recording(self) -> dict:
        return json_ready(self.capture.start(self.site_id, self.robot_id))

    def stop_recording(self, session_id: str) -> dict:
        session = self.capture.stop(session_id)
        if self.maps is None:
            return {"session": json_ready(self.capture.get(session_id)), "map_job": None}
        job = self.maps.submit(session)
        self.capture.link_map_job(session_id, job.job_id)
        return {"session": json_ready(self.capture.get(session_id)), "map_job": json_ready(job)}

    def recording_checkpoints(self, session_id: str) -> dict:
        return self.capture.checkpoints(session_id)

    def mark_recording_checkpoint(self, session_id: str, payload: dict) -> dict:
        requested = payload.get("camera")
        requested = requested if isinstance(requested, dict) else self._gimbal_angles
        angles = self._validated_gimbal_angles(requested)
        return self.capture.mark_checkpoint(
            session_id,
            camera_pan_deg=angles["pan"],
            camera_tilt_deg=angles["tilt"],
            camera_roll_deg=angles["roll"],
            note=str(payload.get("note") or ""),
            spin=payload.get("spin") is not False,
            sample_jpeg=self.camera.capture_jpeg(),
        )

    def delete_recording_checkpoint(self, session_id: str, checkpoint_id: str) -> dict:
        return self.capture.delete_checkpoint(session_id, checkpoint_id)

    def sessions(self) -> list[dict]:
        return [json_ready(item) for item in self.capture.list()]

    def session(self, session_id: str) -> dict:
        return json_ready(self.capture.get(session_id))

    def map_job(self, job_id: str) -> dict:
        if self.maps is None:
            raise KeyError(job_id)
        return json_ready(self.maps.get(job_id))

    def map_jobs(self) -> list[dict]:
        if self.maps is None:
            root = self.data_root / "map-jobs"
            jobs = []
            for path in sorted(root.glob("map-*/job.json"), reverse=True):
                try:
                    jobs.append(json.loads(path.read_text(encoding="utf-8")))
                except (OSError, ValueError):
                    continue
            return jobs
        return [json_ready(item) for item in self.maps.list()]

    def navigation_workspace(self, job_id: str) -> dict:
        if self.maps is not None:
            self.maps.get(job_id)
        value = self.navigation_workspaces.get(job_id)
        # The immutable recorded route remains separately available after the
        # operator edits the working route.  It is already converted to
        # base_link by the same commissioned frame boundary as publication.
        return value | {
            "recordedRoute": self.navigation_workspaces.default(job_id)["route"]
        }

    def update_navigation_workspace(self, job_id: str, payload: dict) -> dict:
        if self.maps is not None:
            self.maps.get(job_id)
        value = self.navigation_workspaces.update(
            job_id, payload, require_current=True
        )
        self.journal.append(
            "map.navigation_workspace_updated",
            job_id=job_id,
            revision=value["revision"],
            workspace_hash=value["workspaceHash"],
            ready=value["ready"],
        )
        return value | {
            "recordedRoute": self.navigation_workspaces.default(job_id)["route"]
        }

    def navigation_surface_preview(self, job_id: str) -> dict:
        if self.maps is not None:
            self.maps.get(job_id)
        return self.routes.navigation_surface_preview(job_id)

    def navigation_plan_preview(self, job_id: str, payload: dict) -> dict:
        if self.maps is not None:
            self.maps.get(job_id)
        return self.routes.plan_preview(
            job_id,
            start=payload.get("start"),
            goal=payload.get("goal"),
            workspace=payload.get("workspace"),
        )

    def recording_export(self, session_id: str) -> dict:
        return self.exchange.recording_descriptor(session_id)

    def recording_export_file(self, session_id: str, relative_name: str) -> Path:
        return self.exchange.recording_file(session_id, relative_name)

    def receive_map_artifact(self, job_id: str, name: str, reader, length: int, sha256: str) -> dict:
        return self.exchange.receive_map_artifact(job_id, name, reader, length, sha256)

    def map_import_descriptor(self, job_id: str) -> dict:
        return self.exchange.map_import_descriptor(job_id)

    def commit_map_import(self, job_id: str, payload: dict) -> dict:
        return self.exchange.commit_map_import(job_id, payload)

    def navigation_status(self) -> dict:
        return self.navigation.status()

    def prepare_navigation(self, job_id: str) -> dict:
        return self.navigation.prepare(job_id)

    def start_navigation_runtime(self, candidate_id: str) -> dict:
        return self.navigation.start_runtime(candidate_id)

    def stop_navigation_runtime(self) -> dict:
        return self.navigation.stop_runtime()

    def reset_localization(self) -> dict:
        return self.navigation.reset_localization()

    def start_patrol(self) -> dict:
        return self.navigation.start_patrol()

    def start_selected_patrol(self, payload: dict) -> dict:
        return self.navigation.start_selected_patrol(
            expected_map_version=payload.get("expected_map_version"),
            expected_route_id=payload.get("expected_route_id"),
            mission_plan=payload.get("mission_plan"),
        )

    def checkpoint_control(self, payload: dict) -> dict:
        action = str(payload.get("action") or "")
        if action == "capture":
            status = self.navigation.status()
            runtime = status.get("runtime")
            runtime = runtime if isinstance(runtime, dict) else {}
            checkpoint = runtime.get("checkpoint")
            checkpoint = checkpoint if isinstance(checkpoint, dict) else {}
            if (
                checkpoint.get("decisionMode") == "local_operator"
                and checkpoint.get("phase") == "WAITING_PLATFORM"
            ):
                camera = checkpoint.get("camera")
                camera = camera if isinstance(camera, dict) else {}
                self.move_gimbal(camera)
                time.sleep(max(0.0, min(30.0, float(checkpoint.get("dwellSec") or 0.0))))
                actual_status = self.gimbal_status()
                actual = actual_status.get("angles")
                actual = actual if isinstance(actual, dict) else {}
                errors = {
                    name: abs(
                        float(actual.get(name, math.inf))
                        - float(camera.get(name) or 0.0)
                    )
                    for name in ("pan", "tilt", "roll")
                }
                if any(not math.isfinite(value) or value > 2.0 for value in errors.values()):
                    raise RuntimeError(
                        "Z1Pro did not reach checkpoint view: "
                        + ", ".join(
                            f"{name} error={value:.1f}deg"
                            for name, value in errors.items()
                        )
                    )
        return self.navigation.checkpoint_control(action)

    def stop_patrol(self) -> dict:
        return self.navigation.stop_patrol()

    def recover_navigation_runtime(self) -> dict:
        return self.navigation.recover_runtime()

    def navigation_profile(self) -> dict:
        return self.navigation.profile()

    def update_navigation_profile(self, profile: dict) -> dict:
        return self.navigation.update_profile(profile)

    def rollback_navigation_profile(self) -> dict:
        return self.navigation.rollback_profile()

    def navigation_diagnostics(self) -> dict:
        return self.navigation.diagnostics()

    def diagnostic_profile(self) -> dict:
        return json_ready(self.diagnostic_profiles.get())

    def update_diagnostic_profile(self, value: dict) -> dict:
        return {
            "profile": json_ready(self.diagnostic_profiles.update(value)),
            "message": "诊断档位已更新；重采集只在巡检活跃时运行",
        }

    def incidents(self) -> list[dict]:
        return self.incident_store.list()

    def incident(self, incident_id: str) -> dict:
        return self.incident_store.get(incident_id)

    def incident_export(self, incident_id: str) -> dict:
        return self.incident_store.descriptor(incident_id)

    def incident_file(self, incident_id: str, relative_name: str) -> Path:
        return self.incident_store.file(incident_id, relative_name)

    def capture_incident(self) -> dict:
        return self.incident_store.request_capture("manual")
