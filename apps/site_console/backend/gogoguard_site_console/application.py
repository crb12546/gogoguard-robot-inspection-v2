from __future__ import annotations

from pathlib import Path

from gogoguard_contracts import json_ready
from gogoguard_data_capture import CaptureManager
from gogoguard_device_io import SnapshotStore, create_camera_gateway, create_gateway
from gogoguard_evidence import EventJournal
from gogoguard_map_factory import MapJobManager


class InspectionApplication:
    def __init__(self, *, data_root: Path, mode: str, map_worker: str, robot_id: str, site_id: str,
                 topics: dict[str, str], cloud: dict | None = None,
                 camera: dict | None = None) -> None:
        data_root.mkdir(parents=True, exist_ok=True)
        self.data_root = data_root
        self.mode = mode
        self.robot_id = robot_id
        self.site_id = site_id
        self.store = SnapshotStore(robot_id, mode, data_root)
        self.journal = EventJournal(data_root / "events" / "runtime.jsonl")
        self.gateway = create_gateway(mode, self.store, robot_id, topics)
        self.camera = create_camera_gateway(mode, camera)
        self.capture = CaptureManager(data_root, self.store, self.journal, mode, topics)
        self.maps = MapJobManager(data_root, map_worker, self.journal, cloud)

    def start(self) -> None:
        self.gateway.start()
        self.journal.append("runtime.started", mode=self.mode, robot_id=self.robot_id)

    def close(self) -> None:
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

    def start_recording(self) -> dict:
        return json_ready(self.capture.start(self.site_id, self.robot_id))

    def stop_recording(self, session_id: str) -> dict:
        session = self.capture.stop(session_id)
        job = self.maps.submit(session)
        self.capture.link_map_job(session_id, job.job_id)
        return {"session": json_ready(self.capture.get(session_id)), "map_job": json_ready(job)}

    def sessions(self) -> list[dict]:
        return [json_ready(item) for item in self.capture.list()]

    def session(self, session_id: str) -> dict:
        return json_ready(self.capture.get(session_id))

    def map_job(self, job_id: str) -> dict:
        return json_ready(self.maps.get(job_id))
