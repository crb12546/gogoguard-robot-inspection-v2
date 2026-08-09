from __future__ import annotations

import hashlib
import json
import math
import os
import re
import shutil
import tempfile
import threading
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from gogoguard_contracts import (
    DiagnosticMode,
    DiagnosticProfile,
    IncidentBundle,
    IncidentState,
    json_ready,
    utc_now,
)


INCIDENT_ID = re.compile(r"^incident-[0-9]{8}T[0-9]{6}Z-[a-f0-9]{8}$")
PROFILE_PRESETS: dict[DiagnosticMode, dict[str, Any]] = {
    DiagnosticMode.DEVELOPMENT: {
        "pre_trigger_s": 15.0,
        "post_trigger_s": 5.0,
        "point_cloud_hz": 10.0,
        "record_camera": True,
        "record_costmap": True,
        "record_planner_detail": True,
    },
    DiagnosticMode.ACCEPTANCE: {
        "pre_trigger_s": 10.0,
        "post_trigger_s": 5.0,
        "point_cloud_hz": 2.0,
        "record_camera": False,
        "record_costmap": True,
        "record_planner_detail": True,
    },
    DiagnosticMode.PRODUCTION: {
        "pre_trigger_s": 15.0,
        "post_trigger_s": 5.0,
        "point_cloud_hz": 0.0,
        "record_camera": False,
        "record_costmap": False,
        "record_planner_detail": False,
    },
}


def _atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(json_ready(value), handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _number(value: Any, name: str, minimum: float, maximum: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number") from exc
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return result


def preset_profile(
    mode: str | DiagnosticMode,
    *,
    revision: int = 1,
    expires_at: str | None = None,
    remaining_patrols: int | None = None,
) -> DiagnosticProfile:
    selected = DiagnosticMode(mode)
    return DiagnosticProfile(
        revision=revision,
        mode=selected,
        expires_at=expires_at,
        remaining_patrols=remaining_patrols,
        **PROFILE_PRESETS[selected],
    )


def validate_profile(value: dict[str, Any]) -> DiagnosticProfile:
    if not isinstance(value, dict):
        raise ValueError("diagnostic profile must be an object")
    mode = DiagnosticMode(str(value.get("mode") or DiagnosticMode.PRODUCTION.value))
    preset = preset_profile(mode)
    expires_at = value.get("expires_at")
    if expires_at:
        parsed = datetime.fromisoformat(str(expires_at).replace("Z", "+00:00"))
        if parsed.tzinfo is None:
            raise ValueError("expires_at must include a timezone")
        expires_at = parsed.astimezone(timezone.utc).isoformat(timespec="seconds")
    remaining = value.get("remaining_patrols")
    if remaining is not None:
        remaining = int(remaining)
        if not 1 <= remaining <= 100:
            raise ValueError("remaining_patrols must be between 1 and 100")
    profile = DiagnosticProfile(
        revision=max(1, int(value.get("revision") or 1)),
        mode=mode,
        pre_trigger_s=_number(value.get("pre_trigger_s", preset.pre_trigger_s), "pre_trigger_s", 0, 60),
        post_trigger_s=_number(value.get("post_trigger_s", preset.post_trigger_s), "post_trigger_s", 0, 30),
        point_cloud_hz=_number(value.get("point_cloud_hz", preset.point_cloud_hz), "point_cloud_hz", 0, 10),
        record_camera=bool(value.get("record_camera", preset.record_camera)),
        record_costmap=bool(value.get("record_costmap", preset.record_costmap)),
        record_planner_detail=bool(value.get("record_planner_detail", preset.record_planner_detail)),
        expires_at=expires_at,
        remaining_patrols=remaining,
        max_incidents=int(_number(value.get("max_incidents", 20), "max_incidents", 1, 100)),
        max_storage_bytes=int(_number(value.get("max_storage_bytes", 2 * 1024**3), "max_storage_bytes", 128 * 1024**2, 20 * 1024**3)),
        updated_at=str(value.get("updated_at") or utc_now()),
    )
    if mode == DiagnosticMode.PRODUCTION and (
        profile.point_cloud_hz
        or profile.record_camera
        or profile.record_costmap
        or profile.record_planner_detail
    ):
        raise ValueError("production mode cannot enable heavy diagnostic capture")
    return profile


class DiagnosticProfileStore:
    """Versioned, self-expiring recorder profile shared by API and recorder."""

    def __init__(self, data_root: Path) -> None:
        self.root = Path(data_root) / "diagnostics"
        self.path = self.root / "profile.json"
        self.history_root = self.root / "history"
        self._lock = threading.Lock()
        self.history_root.mkdir(parents=True, exist_ok=True)
        if not self.path.exists():
            _atomic_json(self.path, preset_profile(DiagnosticMode.PRODUCTION))

    def get(self) -> DiagnosticProfile:
        try:
            profile = validate_profile(json.loads(self.path.read_text(encoding="utf-8")))
        except (OSError, ValueError, TypeError):
            profile = preset_profile(DiagnosticMode.PRODUCTION)
        if profile.expires_at:
            expiry = datetime.fromisoformat(profile.expires_at.replace("Z", "+00:00"))
            if datetime.now(timezone.utc) >= expiry:
                return self._fall_back(profile, "expired")
        if profile.remaining_patrols is not None and profile.remaining_patrols <= 0:
            return self._fall_back(profile, "patrol_limit_reached")
        return profile

    def update(self, value: dict[str, Any]) -> DiagnosticProfile:
        with self._lock:
            current = self.get()
            candidate = validate_profile(value)
            candidate.revision = current.revision + 1
            candidate.updated_at = utc_now()
            _atomic_json(self.history_root / f"revision-{current.revision:04d}.json", current)
            _atomic_json(self.path, candidate)
            return candidate

    def select_preset(
        self,
        mode: str,
        *,
        expires_at: str | None = None,
        remaining_patrols: int | None = None,
    ) -> DiagnosticProfile:
        current = self.get()
        candidate = preset_profile(
            mode,
            revision=current.revision,
            expires_at=expires_at,
            remaining_patrols=remaining_patrols,
        )
        return self.update(json_ready(candidate))

    def consume_patrol(self) -> DiagnosticProfile:
        with self._lock:
            current = self.get()
            if current.remaining_patrols is None:
                return current
            remaining = current.remaining_patrols - 1
            if remaining <= 0:
                return self._fall_back(current, "patrol_limit_reached")
            updated = replace(current, remaining_patrols=remaining, updated_at=utc_now())
            _atomic_json(self.path, updated)
            return updated

    def _fall_back(self, current: DiagnosticProfile, reason: str) -> DiagnosticProfile:
        fallback = preset_profile(DiagnosticMode.PRODUCTION, revision=current.revision + 1)
        fallback.updated_at = utc_now()
        _atomic_json(self.path, fallback)
        _atomic_json(self.root / "last-fallback.json", {"reason": reason, "from": current, "to": fallback})
        return fallback


class IncidentStore:
    """Immutable incident bundles and narrow export surface."""

    def __init__(self, data_root: Path) -> None:
        self.root = Path(data_root) / "incidents"
        self.requests_root = Path(data_root) / "diagnostics" / "requests"
        self.root.mkdir(parents=True, exist_ok=True)
        self.requests_root.mkdir(parents=True, exist_ok=True)

    def list(self) -> list[dict[str, Any]]:
        values = []
        for path in sorted(self.root.glob("incident-*/manifest.json"), reverse=True):
            try:
                value = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            if value.get("schema") == "gogoguard.incident_bundle.v1":
                values.append(value)
        return values

    def get(self, incident_id: str) -> dict[str, Any]:
        root = self._root_for(incident_id)
        try:
            value = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise KeyError(incident_id) from exc
        if value.get("schema") != "gogoguard.incident_bundle.v1":
            raise KeyError(incident_id)
        return value

    def descriptor(self, incident_id: str) -> dict[str, Any]:
        value = self.get(incident_id)
        return {
            "schema": "gogoguard.incident_export.v1",
            "incident": value,
            "files": list(value.get("files") or []),
            "bytes_total": sum(int(item.get("bytes") or 0) for item in value.get("files") or []),
        }

    def file(self, incident_id: str, relative_name: str) -> Path:
        root = self._root_for(incident_id).resolve()
        relative = Path(relative_name)
        if relative.is_absolute() or ".." in relative.parts:
            raise KeyError(relative_name)
        target = (root / relative).resolve()
        if root not in target.parents or not target.is_file() or target.name == "manifest.json":
            raise KeyError(relative_name)
        allowed = {str(item["path"]) for item in self.get(incident_id).get("files") or []}
        if relative.as_posix() not in allowed:
            raise KeyError(relative_name)
        return target

    def request_capture(self, trigger: str = "manual") -> dict[str, Any]:
        request_id = f"request-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{os.urandom(4).hex()}"
        value = {"schema": "gogoguard.incident_capture_request.v1", "request_id": request_id, "trigger": trigger, "requested_at": utc_now()}
        _atomic_json(self.requests_root / f"{request_id}.json", value)
        return value

    def pop_requests(self) -> list[dict[str, Any]]:
        values = []
        for path in sorted(self.requests_root.glob("request-*.json")):
            processing = path.with_suffix(".processing")
            try:
                path.replace(processing)
                values.append(json.loads(processing.read_text(encoding="utf-8")))
                processing.unlink()
            except (OSError, ValueError):
                continue
        return values

    def seal(
        self,
        bundle: IncidentBundle,
        *,
        evidence_present: Iterable[str],
        evidence_missing: Iterable[str],
        state: IncidentState = IncidentState.SEALED,
    ) -> dict[str, Any]:
        root = self._root_for(bundle.incident_id)
        files = []
        for path in sorted(root.rglob("*")):
            if not path.is_file() or path.name == "manifest.json" or path.name.startswith("."):
                continue
            files.append({"path": path.relative_to(root).as_posix(), "bytes": path.stat().st_size, "sha256": _sha256(path)})
        bundle.state = state
        bundle.ended_at = bundle.ended_at or utc_now()
        bundle.files = files
        bundle.evidence_present = sorted(set(evidence_present))
        bundle.evidence_missing = sorted(set(evidence_missing))
        bundle.root = str(root)
        _atomic_json(root / "manifest.json", bundle)
        self.enforce_retention(bundle.diagnostic_profile_revision)
        return json_ready(bundle)

    def enforce_retention(self, _profile_revision: int | None = None) -> None:
        profile = DiagnosticProfileStore(self.root.parent).get()
        roots = sorted((path.parent for path in self.root.glob("incident-*/manifest.json")), key=lambda path: path.name, reverse=True)
        sizes = {root: sum(item.stat().st_size for item in root.rglob("*") if item.is_file()) for root in roots}
        total = sum(sizes.values())
        # Count retention is deterministic first. Capacity retention then
        # removes only the oldest remaining bundle and always keeps the newest
        # incident even if that single bundle exceeds the configured budget.
        for root in roots[profile.max_incidents:]:
            shutil.rmtree(root)
            total -= sizes[root]
        retained = roots[:profile.max_incidents]
        while total > profile.max_storage_bytes and len(retained) > 1:
            root = retained.pop()
            shutil.rmtree(root)
            total -= sizes[root]

    def _root_for(self, incident_id: str) -> Path:
        if not INCIDENT_ID.fullmatch(incident_id):
            raise KeyError(incident_id)
        return self.root / incident_id


def new_incident_id() -> str:
    return f"incident-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{os.urandom(4).hex()}"
