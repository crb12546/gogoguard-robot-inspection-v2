from __future__ import annotations

import json
import threading
from pathlib import Path

from gogoguard_contracts import RecordingSession, RecordingState, json_ready
from gogoguard_evidence import EventJournal
from gogoguard_map_factory import GlimEditorManager, MapJobManager
from gogoguard_route import NavigationWorkspaceStore, validate_workspace

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
        self.glim_editor = GlimEditorManager(
            self.data_root, self.maps, map_worker, cloud
        )
        self.navigation_workspaces = NavigationWorkspaceStore(self.data_root)
        self.catalog_path = self.data_root / "catalog.json"
        self._catalog_lock = threading.Lock()
        self._catalog = self._load_catalog()
        self.incident_root = self.data_root / "incidents"
        self.incident_root.mkdir(parents=True, exist_ok=True)
        self._incident_stop = threading.Event()
        self._incident_sync_lock = threading.Lock()
        self._incident_thread: threading.Thread | None = None

    def start(self) -> None:
        self.journal.append("workstation.started", site_id=self.site_id)
        self._incident_stop.clear()
        self._incident_thread = threading.Thread(
            target=self._incident_sync_loop,
            name="incident-sync",
            daemon=True,
        )
        self._incident_thread.start()

    def close(self) -> None:
        self._incident_stop.set()
        if self._incident_thread and self._incident_thread is not threading.current_thread():
            self._incident_thread.join(timeout=2)
        self.glim_editor.close()
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
        return self._decorate_map_job(json_ready(self.maps.get(job_id)))

    def map_jobs(self) -> list[dict]:
        values = [self._decorate_map_job(json_ready(item)) for item in self.maps.list()]
        return sorted(
            values,
            key=lambda item: (str(item.get("created_at") or ""), str(item.get("job_id") or "")),
            reverse=True,
        )

    def update_map_label(self, job_id: str, label: str) -> dict:
        self.maps.get(job_id)
        value = str(label or "").strip()
        if len(value) > 80:
            raise ValueError("map label must be 80 characters or fewer")
        if any(ord(character) < 32 for character in value):
            raise ValueError("map label contains control characters")
        with self._catalog_lock:
            labels = self._catalog.setdefault("map_labels", {})
            if value:
                labels[job_id] = value
            else:
                labels.pop(job_id, None)
            self._save_catalog_locked()
        self.journal.append("map.label_updated", job_id=job_id, label=value)
        return self.map_job(job_id)

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

    def navigation_workspace(self, job_id: str) -> dict:
        self.maps.get(job_id)
        return self.navigation_workspaces.get(job_id)

    def update_navigation_workspace(self, job_id: str, payload: dict) -> dict:
        self.maps.get(job_id)
        value = self.navigation_workspaces.update(job_id, payload)
        self.journal.append(
            "map.navigation_workspace_updated",
            job_id=job_id,
            revision=value["revision"],
            workspace_hash=value["workspaceHash"],
            ready=value["ready"],
        )
        return value

    def glim_editor_status(self, job_id: str) -> dict:
        return self.glim_editor.status(job_id)

    def start_glim_editor(self, job_id: str) -> dict:
        value = self.glim_editor.start(job_id)
        self.journal.append(
            "map.editor_started",
            job_id=job_id,
            editor_session_id=value.get("sessionId"),
        )
        return value

    def publish_glim_editor(self, job_id: str) -> dict:
        value = self.glim_editor.publish(job_id)
        new_job_id = str(value["mapJob"]["job_id"])
        parent_label = str(self.map_job(job_id).get("label") or job_id)
        value["mapJob"] = self.update_map_label(
            new_job_id, f"{parent_label}（GLIM已清理）"
        )
        return value

    def stop_glim_editor(self, job_id: str) -> dict:
        return self.glim_editor.stop(job_id)

    def navigation_status(self) -> dict:
        return self.robot.get("api/v1/navigation")

    def prepare_navigation(self, job_id: str) -> dict:
        job = self.map_job(job_id)
        if job.get("state") != "complete":
            raise RuntimeError("only a completed GLIM map can be deployed")
        validate_workspace(self.navigation_workspaces.get(job_id), require_ready=True)
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

    def recover_navigation_runtime(self) -> dict:
        return self.robot.post(
            "api/v1/navigation/runtime/recover",
            timeout_s=max(self.robot.timeout_s, 30.0),
        )

    def navigation_profile(self) -> dict:
        return self.robot.get("api/v1/navigation/profile")

    def update_navigation_profile(self, profile: dict) -> dict:
        return self.robot.post(
            "api/v1/navigation/profile", {"profile": profile},
            timeout_s=max(self.robot.timeout_s, 30.0),
        )

    def rollback_navigation_profile(self) -> dict:
        return self.robot.post(
            "api/v1/navigation/profile/rollback",
            timeout_s=max(self.robot.timeout_s, 30.0),
        )

    def navigation_diagnostics(self) -> dict:
        return self.robot.get("api/v1/navigation/diagnostics")

    def diagnostic_profile(self) -> dict:
        return self.robot.get("api/v1/diagnostics/profile")

    def update_diagnostic_profile(self, value: dict) -> dict:
        return self.robot.post("api/v1/diagnostics/profile", {"profile": value})

    def capture_incident(self) -> dict:
        return self.robot.post("api/v1/incidents/capture")

    def incidents(self) -> list[dict]:
        self.sync_incidents()
        values = []
        for path in sorted(self.incident_root.glob("incident-*/manifest.json"), reverse=True):
            try:
                values.append(json.loads(path.read_text(encoding="utf-8")))
            except (OSError, ValueError):
                continue
        return values

    def incident(self, incident_id: str) -> dict:
        path = self._incident_path(incident_id) / "manifest.json"
        if not path.is_file():
            self.sync_incidents()
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise KeyError(incident_id) from exc

    def incident_export(self, incident_id: str) -> dict:
        incident = self.incident(incident_id)
        return {"schema": "gogoguard.incident_export.v1", "incident": incident, "files": incident.get("files") or []}

    def incident_file(self, incident_id: str, relative_name: str) -> Path:
        root = self._incident_path(incident_id).resolve()
        relative = Path(relative_name)
        if relative.is_absolute() or ".." in relative.parts:
            raise KeyError(relative_name)
        target = (root / relative).resolve()
        if root not in target.parents or not target.is_file():
            raise KeyError(relative_name)
        allowed = {str(item.get("path")) for item in self.incident(incident_id).get("files") or []}
        if relative.as_posix() not in allowed:
            raise KeyError(relative_name)
        return target

    def sync_incidents(self) -> list[str]:
        with self._incident_sync_lock:
            try:
                remote = self.robot.incidents()
            except RobotConnectionError:
                return []
            synced = []
            for item in remote:
                incident_id = str(item.get("incident_id") or "")
                if not incident_id.startswith("incident-") or item.get("state") not in {"sealed", "partial"}:
                    continue
                target = self._incident_path(incident_id)
                manifest = target / "manifest.json"
                if manifest.is_file():
                    continue
                try:
                    self.robot.download_incident(incident_id, target)
                    synced.append(incident_id)
                    self.journal.append("incident.synced", incident_id=incident_id)
                except (RobotConnectionError, OSError, ValueError) as exc:
                    self.journal.append("incident.sync_failed", incident_id=incident_id, error=str(exc))
            return synced

    def _incident_sync_loop(self) -> None:
        while not self._incident_stop.is_set():
            self.sync_incidents()
            self._incident_stop.wait(5.0)

    def _incident_path(self, incident_id: str) -> Path:
        if not incident_id.startswith("incident-") or "/" in incident_id or ".." in incident_id:
            raise KeyError(incident_id)
        return self.incident_root / incident_id

    def _load_catalog(self) -> dict:
        try:
            value = json.loads(self.catalog_path.read_text(encoding="utf-8"))
            if value.get("schema") == "gogoguard.workstation_catalog.v1":
                return value
        except (OSError, ValueError):
            pass
        return {
            "schema": "gogoguard.workstation_catalog.v1",
            "sessions": {},
            "map_labels": {},
        }

    def _decorate_map_job(self, value: dict) -> dict:
        label = str(self._catalog.get("map_labels", {}).get(value.get("job_id"), ""))
        return dict(value) | {"label": label}

    def _link_session(self, session_id: str, job_id: str, session: dict) -> None:
        with self._catalog_lock:
            self._catalog.setdefault("sessions", {})[session_id] = dict(session) | {
                "map_job_id": job_id
            }
            self._save_catalog_locked()

    def _save_catalog_locked(self) -> None:
        temporary = self.catalog_path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(self._catalog, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.catalog_path)
