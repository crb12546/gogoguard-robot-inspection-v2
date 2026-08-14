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
        self._sample_lock = threading.Lock()
        self._sample_thread: threading.Thread | None = None
        self._sample_error: Exception | None = None
        self._rosbag: subprocess.Popen | None = None
        self._closed = False
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
            if self._closed:
                raise CaptureError("capture manager is closed")
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
            self._sample_error = None
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
        try:
            while not self._sample_stop.is_set():
                snapshot = self.store.get()
                if self._sample_stop.is_set():
                    break
                self._append_snapshot(session, path, snapshot)
                self._sample_stop.wait(0.2)
        except Exception as exc:
            self._sample_error = exc
            self._sample_stop.set()
            self.journal.append(
                "recording.sample_failed",
                session_id=session_id,
                error_type=type(exc).__name__,
                error=str(exc),
            )

    def _append_snapshot(self, session: RecordingSession, path: Path, snapshot) -> int:
        with self._sample_lock:
            sample_index = int(session.sample_count)
            with path.open("a", encoding="utf-8") as handle:
                handle.write(
                    json.dumps(
                        json_ready(snapshot),
                        ensure_ascii=False,
                        separators=(",", ":"),
                    )
                    + "\n"
                )
            session.sample_count += 1
            if session.sample_count % 10 == 0:
                self._save(session)
            return sample_index

    def stop(self, session_id: str) -> RecordingSession:
        with self._lock:
            if self._active_id != session_id:
                raise CaptureError(f"session {session_id} is not active")
            session = self._sessions[session_id]
            session.state = RecordingState.SEALING
            self._save(session)
            self._sample_stop.set()
            sampler_stopped = True
            if self._sample_thread:
                self._sample_thread.join(timeout=3)
                sampler_stopped = not self._sample_thread.is_alive()
            self._stop_rosbag(timeout=20)
            if not sampler_stopped:
                session.state = RecordingState.FAILED
                session.error = "recording sampler did not stop"
                session.stopped_at = utc_now()
                self._active_id = None
                self._save(session)
                raise CaptureError(session.error)
            if self._sample_error is not None:
                session.state = RecordingState.FAILED
                session.error = f"snapshot sampling failed: {type(self._sample_error).__name__}"
                session.stopped_at = utc_now()
                self._active_id = None
                self._save(session)
                raise CaptureError(session.error) from self._sample_error
            session.stopped_at = utc_now()
            manifest_path = self._seal(Path(session.root), session)
            session.bundle_manifest = str(manifest_path)
            session.state = RecordingState.SEALED
            self._active_id = None
            self._save(session)
            self.journal.append("recording.sealed", session_id=session_id, samples=session.sample_count, manifest=str(manifest_path))
            return self.get(session_id)

    def close(self) -> None:
        """Stop owned background work without claiming an interrupted bundle is sealed."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            active_id = self._active_id
            self._sample_stop.set()
        thread = self._sample_thread
        if thread is not None:
            thread.join(timeout=3)
        with self._lock:
            self._stop_rosbag(timeout=3)
            if active_id is not None and self._active_id == active_id:
                session = self._sessions[active_id]
                session.state = RecordingState.FAILED
                session.error = "recording interrupted by runtime shutdown"
                session.stopped_at = utc_now()
                self._active_id = None
                try:
                    self._save(session)
                except OSError:
                    pass
                self.journal.append(
                    "recording.interrupted",
                    session_id=active_id,
                    sampler_stopped=not bool(thread and thread.is_alive()),
                )

    def _stop_rosbag(self, *, timeout: float) -> None:
        process = self._rosbag
        if process is None:
            return
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                self._rosbag = None
                return
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    self._rosbag = None
                    return
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        self._rosbag = None
                        return
                    process.wait(timeout=3)
        self._rosbag = None

    def checkpoints(self, session_id: str) -> dict:
        session = self.get(session_id)
        path = Path(session.root) / "samples" / "inspection" / "checkpoints.json"
        if not path.is_file():
            return {
                "schema": "gogoguard.recording_checkpoints.v1",
                "sessionId": session_id,
                "checkpoints": [],
            }
        value = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(value, dict) or value.get("schema") != "gogoguard.recording_checkpoints.v1":
            raise CaptureError("recording checkpoint metadata is invalid")
        return value

    def mark_checkpoint(
        self,
        session_id: str,
        *,
        camera_pan_deg: float,
        camera_tilt_deg: float,
        camera_roll_deg: float = 0.0,
        note: str = "",
        spin: bool = True,
        sample_jpeg: bytes,
    ) -> dict:
        with self._lock:
            if self._active_id != session_id:
                raise CaptureError("checkpoints can only be marked during the active recording")
            if (
                not isinstance(sample_jpeg, (bytes, bytearray))
                or not bytes(sample_jpeg).startswith(b"\xff\xd8")
                or not bytes(sample_jpeg).endswith(b"\xff\xd9")
            ):
                raise CaptureError("checkpoint sample must be a JPEG")
            value = self.checkpoints(session_id)
            items = list(value.get("checkpoints") or [])
            ordinal = max(
                [
                    int(str(item.get("checkpointId") or "cp_00").rsplit("_", 1)[-1])
                    for item in items
                    if str(item.get("checkpointId") or "").startswith("cp_")
                    and str(item.get("checkpointId") or "").rsplit("_", 1)[-1].isdigit()
                ]
                or [0]
            ) + 1
            checkpoint_id = f"cp_{ordinal:02d}"
            root = Path(self._sessions[session_id].root) / "samples" / "inspection"
            root.mkdir(parents=True, exist_ok=True)
            relative_sample = f"{checkpoint_id}-main.jpg"
            sample_path = root / relative_sample
            temporary_sample = sample_path.with_name(f".{sample_path.name}.part")
            with temporary_sample.open("wb") as handle:
                handle.write(bytes(sample_jpeg))
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_sample, sample_path)
            snapshot = self.store.get()
            snapshot_path = Path(self._sessions[session_id].root) / "samples" / "snapshots.jsonl"
            sample_index = self._append_snapshot(
                self._sessions[session_id], snapshot_path, snapshot
            )
            snapshot = json_ready(snapshot)
            pose = dict(snapshot.get("pose") or {})
            item = {
                "checkpointId": checkpoint_id,
                "recordingSampleIndex": sample_index,
                "recordingSequence": int(snapshot.get("sequence") or 0),
                "markedAt": utc_now(),
                "rawPose": {
                    "x": float(pose.get("x") or 0.0),
                    "y": float(pose.get("y") or 0.0),
                    "z": float(pose.get("z") or 0.0),
                    "yaw": float(pose.get("yaw") or 0.0),
                },
                "camera": {
                    "pan": float(camera_pan_deg),
                    "tilt": float(camera_tilt_deg),
                    "roll": float(camera_roll_deg),
                    "frame": "carrier_relative",
                },
                "spin": bool(spin),
                "note": str(note or "")[:200],
                "sampleFrames": [f"samples/inspection/{relative_sample}"],
            }
            items.append(item)
            payload = {
                "schema": "gogoguard.recording_checkpoints.v1",
                "sessionId": session_id,
                "checkpoints": items,
            }
            self._atomic_json(root / "checkpoints.json", payload)
            self.journal.append(
                "recording.checkpoint_marked",
                session_id=session_id,
                checkpoint_id=checkpoint_id,
                sample_index=item["recordingSampleIndex"],
            )
            return item

    def delete_checkpoint(self, session_id: str, checkpoint_id: str) -> dict:
        with self._lock:
            if self._active_id != session_id:
                raise CaptureError("checkpoints can only be changed during the active recording")
            value = self.checkpoints(session_id)
            items = list(value.get("checkpoints") or [])
            target = next((item for item in items if item.get("checkpointId") == checkpoint_id), None)
            if target is None:
                raise KeyError(checkpoint_id)
            root = Path(self._sessions[session_id].root)
            for relative in target.get("sampleFrames") or []:
                path = (root / str(relative)).resolve()
                if root.resolve() in path.parents:
                    path.unlink(missing_ok=True)
            items.remove(target)
            payload = {**value, "checkpoints": items}
            self._atomic_json(root / "samples" / "inspection" / "checkpoints.json", payload)
            return payload

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

    @staticmethod
    def _atomic_json(path: Path, payload: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
