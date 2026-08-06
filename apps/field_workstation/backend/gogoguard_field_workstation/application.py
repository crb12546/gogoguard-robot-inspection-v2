from __future__ import annotations

import json
import threading
from pathlib import Path

from gogoguard_contracts import RecordingSession, RecordingState, json_ready
from gogoguard_evidence import EventJournal
from gogoguard_map_factory import MapJobManager

from .robot_client import RobotClient, RobotConnectionError


class FieldWorkstationApplication:
    """Mac-owned field workflow; robot algorithms remain behind Edge Agent APIs."""

    def __init__(
        self,
        *,
        data_root: Path,
        robot: dict,
        cloud: dict,
        map_worker: str = "ssh",
        site_id: str = "local-first-site",
    ) -> None:
        self.data_root = Path(data_root)
        self.data_root.mkdir(parents=True, exist_ok=True)
        self.site_id = site_id
        self.robot = RobotClient(
            str(robot.get("base_url") or "http://192.168.123.18:8080"),
            timeout_s=float(robot.get("timeout_s") or 10.0),
        )
        self.journal = EventJournal(self.data_root / "events" / "workstation.jsonl")
        self.maps = MapJobManager(self.data_root, map_worker, self.journal, cloud)
        self.catalog_path = self.data_root / "catalog.json"
        self._catalog_lock = threading.Lock()
        self._catalog = self._load_catalog()

    def start(self) -> None:
        self.journal.append("workstation.started", site_id=self.site_id)

    def close(self) -> None:
        self.journal.append("workstation.stopped", site_id=self.site_id)

    def status(self) -> dict:
        try:
            robot_status = self.robot.get("api/v1/status")
        except RobotConnectionError as exc:
            robot_status = {"online": False, "error": str(exc)}
        return {
            "schema": "gogoguard.field_workstation_status.v1",
            "runtime": {"mode": "workstation", "site_id": self.site_id},
            "robot": robot_status,
        }

    def live(self) -> dict:
        return self.robot.live()

    def camera_status(self) -> dict:
        return self.robot.camera()

    def start_recording(self) -> dict:
        return self.robot.start_recording()

    def stop_recording(self, session_id: str) -> dict:
        outcome = self.robot.stop_recording(session_id)
        remote = dict(outcome["session"])
        job = self._submit_recording(remote)
        return {"session": remote | {"map_job_id": job.job_id}, "map_job": json_ready(job)}

    def submit_recording(self, session_id: str) -> dict:
        remote = dict(self.session(session_id))
        job = self._submit_recording(remote)
        return {"session": remote | {"map_job_id": job.job_id}, "map_job": json_ready(job)}

    def _submit_recording(self, remote: dict):
        session = RecordingSession(
            **(remote | {"state": RecordingState(remote["state"])})
        )
        if session.state != RecordingState.SEALED:
            raise RuntimeError("only a sealed historical recording can be mapped")
        local_root = self.data_root / "recordings" / session.session_id

        def source_resolver(job_id, report):
            return self.robot.download_recording(session.session_id, local_root, report)

        job = self.maps.submit(session, source_resolver=source_resolver)
        self._link_session(session.session_id, job.job_id, remote)
        return job

    def sessions(self) -> list[dict]:
        values: dict[str, dict] = {}
        try:
            values.update({item["session_id"]: dict(item) for item in self.robot.sessions()})
        except RobotConnectionError:
            pass
        local_root = self.data_root / "recordings"
        for path in local_root.glob("*/session.json"):
            try:
                item = json.loads(path.read_text(encoding="utf-8"))
                values[item["session_id"]] = item
            except (OSError, ValueError, KeyError):
                continue
        for session_id, metadata in self._catalog.get("sessions", {}).items():
            values.setdefault(session_id, dict(metadata)).update(
                {"map_job_id": metadata.get("map_job_id")}
            )
        return [values[key] for key in sorted(values, reverse=True)]

    def session(self, session_id: str) -> dict:
        for item in self.sessions():
            if item.get("session_id") == session_id:
                return item
        raise KeyError(session_id)

    def map_job(self, job_id: str) -> dict:
        return json_ready(self.maps.get(job_id))

    def map_jobs(self) -> list[dict]:
        return [json_ready(item) for item in self.maps.list()]

    def retry_map_job(self, job_id: str) -> dict:
        job = self.maps.get(job_id)
        remote = self.session(job.session_id)
        session = RecordingSession(
            **(remote | {"state": RecordingState(remote["state"])})
        )
        local_root = self.data_root / "recordings" / session.session_id

        def source_resolver(retry_job_id, report):
            return self.robot.download_recording(session.session_id, local_root, report)

        return json_ready(self.maps.retry(job_id, session, source_resolver))

    def navigation_status(self) -> dict:
        return self.robot.get("api/v1/navigation")

    def prepare_navigation(self, job_id: str) -> dict:
        job = self.map_job(job_id)
        if job.get("state") != "complete":
            raise RuntimeError("only a completed GLIM map can be deployed")
        self.robot.deploy_map(job, Path(str(job["artifact_root"])))
        return self.robot.post(
            "api/v1/navigation/prepare",
            {"job_id": job_id},
            timeout_s=max(self.robot.timeout_s, 60.0),
        )

    def start_navigation_runtime(self, candidate_id: str) -> dict:
        return self.robot.post(
            "api/v1/navigation/runtime/start",
            {"candidate_id": candidate_id},
            timeout_s=max(self.robot.timeout_s, 30.0),
        )

    def stop_navigation_runtime(self) -> dict:
        return self.robot.post(
            "api/v1/navigation/runtime/stop",
            timeout_s=max(self.robot.timeout_s, 30.0),
        )

    def reset_localization(self) -> dict:
        return self.robot.post(
            "api/v1/navigation/localization/reset",
            timeout_s=max(self.robot.timeout_s, 30.0),
        )

    def start_patrol(self) -> dict:
        return self.robot.post(
            "api/v1/navigation/patrol/start",
            timeout_s=max(self.robot.timeout_s, 30.0),
        )

    def stop_patrol(self) -> dict:
        return self.robot.post(
            "api/v1/navigation/patrol/stop",
            timeout_s=max(self.robot.timeout_s, 30.0),
        )

    def _load_catalog(self) -> dict:
        try:
            value = json.loads(self.catalog_path.read_text(encoding="utf-8"))
            if value.get("schema") == "gogoguard.workstation_catalog.v1":
                return value
        except (OSError, ValueError):
            pass
        return {"schema": "gogoguard.workstation_catalog.v1", "sessions": {}}

    def _link_session(self, session_id: str, job_id: str, session: dict) -> None:
        with self._catalog_lock:
            self._catalog.setdefault("sessions", {})[session_id] = dict(session) | {
                "map_job_id": job_id
            }
            temporary = self.catalog_path.with_suffix(".tmp")
            temporary.write_text(
                json.dumps(self._catalog, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            temporary.replace(self.catalog_path)
