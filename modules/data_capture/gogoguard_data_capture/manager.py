from __future__ import annotations

import hashlib
import json
import os
import shutil
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path

from gogoguard_contracts import RecordingSession, RecordingState, json_ready, utc_now
from gogoguard_evidence import EventJournal


class CaptureError(RuntimeError):
    pass


class CaptureManager:
    def __init__(self, root: Path, store, journal: EventJournal, mode: str, topics: dict[str, str]) -> None:
        self.root = root / "recordings"
        self.root.mkdir(parents=True, exist_ok=True)
        self.store = store
        self.journal = journal
        self.mode = mode
        self.topics = topics
        self._lock = threading.Lock()
        self._sessions: dict[str, RecordingSession] = {}
        self._active_id: str | None = None
        self._sample_stop = threading.Event()
        self._sample_thread: threading.Thread | None = None
        self._rosbag: subprocess.Popen | None = None
        self._load_existing()

    def _load_existing(self) -> None:
        for path in self.root.glob("*/session.json"):
            try:
                value = json.loads(path.read_text(encoding="utf-8"))
                value["state"] = RecordingState(value["state"])
                self._sessions[value["session_id"]] = RecordingSession(**value)
            except (OSError, ValueError, TypeError, KeyError):
                continue

    def start(self, site_id: str, robot_id: str) -> RecordingSession:
        with self._lock:
            if self._active_id:
                raise CaptureError(f"session {self._active_id} is already recording")
            if shutil.disk_usage(self.root).free < 2 * 1024**3:
                raise CaptureError("free disk space is below 2 GiB")
            session_id = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + uuid.uuid4().hex[:8]
            session_root = self.root / session_id
            (session_root / "samples").mkdir(parents=True)
            (session_root / "raw").mkdir()
            session = RecordingSession(
                session_id=session_id, site_id=site_id, robot_id=robot_id,
                state=RecordingState.RECORDING, started_at=utc_now(), root=str(session_root),
            )
            self._sessions[session_id] = session
            self._active_id = session_id
            self._save(session)
            if self.mode == "robot":
                try:
                    self._start_rosbag(session_root)
                except Exception:
                    session.state = RecordingState.FAILED
                    session.error = "rosbag2 failed to start"
                    self._active_id = None
                    self._save(session)
                    raise
            self._sample_stop.clear()
            self._sample_thread = threading.Thread(target=self._sample, args=(session_id,), daemon=True)
            self._sample_thread.start()
            self.journal.append("recording.started", session_id=session_id, mode=self.mode)
            return self.get(session_id)

    def _start_rosbag(self, session_root: Path) -> None:
        command = ["ros2", "bag", "record", "-o", str(session_root / "raw" / "rosbag"), *self.topics.values()]
        try:
            self._rosbag = subprocess.Popen(command, start_new_session=True)
        except OSError as exc:
            self._active_id = None
            raise CaptureError(f"cannot start rosbag2: {exc}") from exc

    def _sample(self, session_id: str) -> None:
        session = self._sessions[session_id]
        path = Path(session.root) / "samples" / "snapshots.jsonl"
        while not self._sample_stop.is_set():
            snapshot = self.store.get()
            with path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(json_ready(snapshot), ensure_ascii=False, separators=(",", ":")) + "\n")
            session.sample_count += 1
            if session.sample_count % 10 == 0:
                self._save(session)
            self._sample_stop.wait(0.2)

    def stop(self, session_id: str) -> RecordingSession:
        with self._lock:
            if self._active_id != session_id:
                raise CaptureError(f"session {session_id} is not active")
            session = self._sessions[session_id]
            session.state = RecordingState.SEALING
            self._save(session)
            self._sample_stop.set()
            if self._sample_thread:
                self._sample_thread.join(timeout=3)
            if self._rosbag and self._rosbag.poll() is None:
                os.killpg(self._rosbag.pid, signal.SIGINT)
                try:
                    self._rosbag.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(self._rosbag.pid, signal.SIGTERM)
            session.stopped_at = utc_now()
            manifest_path = self._seal(Path(session.root), session)
            session.bundle_manifest = str(manifest_path)
            session.state = RecordingState.SEALED
            self._active_id = None
            self._save(session)
            self.journal.append("recording.sealed", session_id=session_id, samples=session.sample_count, manifest=str(manifest_path))
            return self.get(session_id)

    def _seal(self, root: Path, session: RecordingSession) -> Path:
        files = []
        for path in sorted(root.rglob("*")):
            if not path.is_file() or path.name in {"recording_bundle.json", "session.json"}:
                continue
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            files.append({"path": str(path.relative_to(root)), "sha256": digest, "bytes": path.stat().st_size})
        manifest = {
            "schema": "gogoguard.recording_bundle.v1",
            "session_id": session.session_id,
            "site_id": session.site_id,
            "robot_id": session.robot_id,
            "started_at": session.started_at,
            "stopped_at": session.stopped_at,
            "capture_mode": self.mode,
            "files": files,
        }
        path = root / "recording_bundle.json"
        path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return path

    def link_map_job(self, session_id: str, job_id: str) -> None:
        session = self._sessions[session_id]
        session.map_job_id = job_id
        self._save(session)

    def list(self) -> list[RecordingSession]:
        return [self.get(key) for key in sorted(self._sessions, reverse=True)]

    def get(self, session_id: str) -> RecordingSession:
        if session_id not in self._sessions:
            raise KeyError(session_id)
        value = self._sessions[session_id]
        return RecordingSession(**json_ready(value) | {"state": value.state})

    def active(self) -> RecordingSession | None:
        return self.get(self._active_id) if self._active_id else None

    @staticmethod
    def _save(session: RecordingSession) -> None:
        path = Path(session.root) / "session.json"
        path.write_text(json.dumps(json_ready(session), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
