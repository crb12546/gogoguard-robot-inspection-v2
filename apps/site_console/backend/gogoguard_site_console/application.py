from __future__ import annotations

import json
from pathlib import Path

from gogoguard_contracts import json_ready
from gogoguard_data_capture import CaptureManager
from gogoguard_device_io import SnapshotStore, create_camera_gateway, create_gateway
from gogoguard_evidence import DiagnosticProfileStore, EventJournal, IncidentStore
from gogoguard_map_factory import MapJobManager
from gogoguard_navigation import NavigationManager, NavigationSupervisorClient
from gogoguard_transfer import EdgeArtifactExchange


class InspectionApplication:
    def __init__(self, *, data_root: Path, mode: str, map_worker: str, robot_id: str, site_id: str,
                 topics: dict[str, str], sensor_id: str = "ARMCP6B0035634", cloud: dict | None = None,
                 camera: dict | None = None, capabilities: dict | None = None) -> None:
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
        self.capture = CaptureManager(data_root, self.store, self.journal, mode, topics)
        self.maps = None if map_worker == "none" else MapJobManager(
            data_root, map_worker, self.journal, cloud
        )
        self.exchange = EdgeArtifactExchange(data_root)
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
