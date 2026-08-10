from __future__ import annotations

import hashlib
import json
import os
import re
import tempfile
import threading
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from gogoguard_contracts import InspectionFrame, json_ready, utc_now


SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")


@dataclass(frozen=True)
class BufferedEvidence:
    frame: InspectionFrame
    payload_path: Path
    buffered_at: str


class EvidenceFrameBuffer:
    """Bounded, checksummed evidence queue; it never makes inspection judgments."""

    def __init__(self, root: Path, *, max_frames: int, max_bytes: int) -> None:
        if max_frames <= 0 or max_bytes <= 0:
            raise ValueError("evidence buffer limits must be positive")
        self.root = Path(root)
        self.pending_root = self.root / "pending"
        self.pending_root.mkdir(parents=True, exist_ok=True)
        self.max_frames = int(max_frames)
        self.max_bytes = int(max_bytes)
        self._lock = threading.Lock()

    def store(self, frame: InspectionFrame, payload: bytes) -> BufferedEvidence:
        self._validate_frame(frame)
        if not payload:
            raise ValueError("inspection frame payload is empty")
        if len(payload) > self.max_bytes:
            raise ValueError("inspection frame exceeds the complete offline buffer")
        digest = hashlib.sha256(payload).hexdigest()
        if frame.sha256 and frame.sha256 != digest:
            raise ValueError("inspection frame hash does not match payload")
        if frame.byte_count not in {0, len(payload)}:
            raise ValueError("inspection frame byte count does not match payload")
        sealed = replace(frame, sha256=digest, byte_count=len(payload))
        buffered_at = utc_now()
        with self._lock:
            if self._metadata_path(frame.frame_id).exists():
                existing = self._read(frame.frame_id)
                if existing.frame.sha256 != digest:
                    raise ValueError("frame id already exists with different content")
                if json_ready(existing.frame) != json_ready(sealed):
                    raise ValueError("frame id already exists with different metadata")
                return existing
            payload_path = self.pending_root / f"{frame.frame_id}.payload"
            self._atomic_bytes(payload_path, payload)
            metadata = {
                "schema": "gogoguard.buffered_inspection_evidence.v1",
                "frame": json_ready(sealed),
                "payloadFile": payload_path.name,
                "bufferedAt": buffered_at,
            }
            self._atomic_json(self._metadata_path(frame.frame_id), metadata)
            self._enforce_limits(protected_id=frame.frame_id)
            self._write_state()
            return BufferedEvidence(sealed, payload_path, buffered_at)

    def pending(self) -> list[BufferedEvidence]:
        with self._lock:
            return [self._read(path.stem) for path in self._metadata_paths()]

    def acknowledge(self, frame_id: str) -> bool:
        self._require_id(frame_id, "frame_id")
        with self._lock:
            metadata_path = self._metadata_path(frame_id)
            if not metadata_path.exists():
                return False
            item = self._read(frame_id)
            item.payload_path.unlink(missing_ok=True)
            metadata_path.unlink(missing_ok=True)
            self._write_state()
            return True

    def status(self) -> dict[str, Any]:
        with self._lock:
            items = [self._read(path.stem) for path in self._metadata_paths()]
            state = self._read_state()
            return {
                "schema": "gogoguard.inspection_buffer_status.v1",
                "maxFrames": self.max_frames,
                "maxBytes": self.max_bytes,
                "pendingFrames": len(items),
                "pendingBytes": sum(item.frame.byte_count for item in items),
                "droppedFrames": int(state.get("droppedFrames", 0)),
                "updatedAt": state.get("updatedAt"),
            }

    def _validate_frame(self, frame: InspectionFrame) -> None:
        for name in (
            "frame_id",
            "mission_id",
            "checkpoint_id",
            "view_id",
            "map_version",
            "route_id",
        ):
            self._require_id(getattr(frame, name), name)
        if frame.content_type not in {"image/jpeg", "image/png"}:
            raise ValueError("inspection evidence must be JPEG or PNG")

    def _metadata_paths(self) -> list[Path]:
        return sorted(
            self.pending_root.glob("*.json"),
            key=lambda path: (path.stat().st_mtime_ns, path.name),
        )

    def _metadata_path(self, frame_id: str) -> Path:
        return self.pending_root / f"{frame_id}.json"

    def _read(self, frame_id: str) -> BufferedEvidence:
        value = json.loads(self._metadata_path(frame_id).read_text(encoding="utf-8"))
        if value.get("schema") != "gogoguard.buffered_inspection_evidence.v1":
            raise ValueError("unsupported buffered evidence schema")
        frame = InspectionFrame(**value["frame"])
        payload_path = self.pending_root / str(value["payloadFile"])
        if payload_path.parent != self.pending_root or not payload_path.is_file():
            raise ValueError("buffered evidence payload is unavailable")
        if payload_path.stat().st_size != frame.byte_count:
            raise ValueError("buffered evidence size changed")
        if hashlib.sha256(payload_path.read_bytes()).hexdigest() != frame.sha256:
            raise ValueError("buffered evidence hash changed")
        return BufferedEvidence(frame, payload_path, str(value["bufferedAt"]))

    def _enforce_limits(self, *, protected_id: str) -> None:
        items = [self._read(path.stem) for path in self._metadata_paths()]
        total_bytes = sum(item.frame.byte_count for item in items)
        dropped = 0
        while len(items) > self.max_frames or total_bytes > self.max_bytes:
            victim_index = next(
                (index for index, item in enumerate(items) if item.frame.frame_id != protected_id),
                None,
            )
            if victim_index is None:
                raise RuntimeError("offline evidence buffer cannot retain the new frame")
            victim = items.pop(victim_index)
            total_bytes -= victim.frame.byte_count
            victim.payload_path.unlink(missing_ok=True)
            self._metadata_path(victim.frame.frame_id).unlink(missing_ok=True)
            dropped += 1
        if dropped:
            state = self._read_state()
            state["droppedFrames"] = int(state.get("droppedFrames", 0)) + dropped
            state["updatedAt"] = utc_now()
            self._atomic_json(self.root / "buffer-state.json", state)

    def _read_state(self) -> dict[str, Any]:
        path = self.root / "buffer-state.json"
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            return value if isinstance(value, dict) else {}
        except FileNotFoundError:
            return {}

    def _write_state(self) -> None:
        state = self._read_state()
        state.update(self.status_without_lock(), updatedAt=utc_now())
        self._atomic_json(self.root / "buffer-state.json", state)

    def status_without_lock(self) -> dict[str, Any]:
        items = [self._read(path.stem) for path in self._metadata_paths()]
        return {
            "schema": "gogoguard.inspection_buffer_status.v1",
            "maxFrames": self.max_frames,
            "maxBytes": self.max_bytes,
            "pendingFrames": len(items),
            "pendingBytes": sum(item.frame.byte_count for item in items),
            "droppedFrames": int(self._read_state().get("droppedFrames", 0)),
        }

    @staticmethod
    def _require_id(value: str, name: str) -> None:
        if not isinstance(value, str) or not SAFE_ID.fullmatch(value):
            raise ValueError(f"{name} is invalid")

    @staticmethod
    def _atomic_bytes(path: Path, payload: bytes) -> None:
        descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
        try:
            with os.fdopen(descriptor, "wb") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    @staticmethod
    def _atomic_json(path: Path, value: dict[str, Any]) -> None:
        descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                json.dump(value, handle, ensure_ascii=False, sort_keys=True, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
