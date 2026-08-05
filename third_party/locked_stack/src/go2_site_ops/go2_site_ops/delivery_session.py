"""State machine for a field operator's guided mapping session."""

from __future__ import annotations

import json
import os
import tempfile
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

from .coverage import CoverageMap, CoveragePolicy
from .review import ReviewQueue


ALLOWED_TRANSITIONS = {
    "new": {"capturing"},
    "capturing": {"paused", "capture_review"},
    "paused": {"capturing", "capture_review"},
    "capture_review": {"capturing", "ready_to_build"},
    "ready_to_build": set(),
}


class SessionError(ValueError):
    pass


class DeliverySession:
    def __init__(
        self,
        session_id: Optional[str] = None,
        state_path: Optional[Path] = None,
        policy: Optional[CoveragePolicy] = None,
        restore_existing: bool = True,
    ):
        self.session_id = session_id or (
            time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:6]
        )
        self.state = "new"
        self.site_id = ""
        self.operator = ""
        self.started_at: Optional[float] = None
        self.updated_at = time.time()
        self.health_updated_at: Optional[float] = None
        self.coverage = CoverageMap(policy)
        self.review = ReviewQueue()
        self.context: Dict[str, Any] = {}
        self.health: Dict[str, Any] = {
            "lidar": "unknown",
            "imu": "unknown",
            "timeSync": "unknown",
            "storage": "unknown",
            "network": "unknown",
        }
        self.state_path = state_path
        self._lock = threading.RLock()
        self.loaded = False
        if (
            restore_existing
            and self.state_path is not None
            and self.state_path.is_file()
        ):
            self._load()

    def transition(self, target: str) -> None:
        with self._lock:
            if target not in ALLOWED_TRANSITIONS.get(self.state, set()):
                raise SessionError("cannot transition %s -> %s" % (self.state, target))
            if target == "ready_to_build":
                summary = self.coverage.summary()
                if summary["missingCells"] or summary["weakCells"] or summary["reviewCells"]:
                    raise SessionError("coverage is not ready for map building")
            self.state = target
            self.updated_at = time.time()
            self._persist()

    def start(self, site_id: str, operator: str) -> None:
        if not site_id.strip() or not operator.strip():
            raise SessionError("site_id and operator are required")
        with self._lock:
            if self.state != "new":
                raise SessionError("session has already started")
            self.site_id = site_id.strip()
            self.operator = operator.strip()
            self.started_at = time.time()
            self.transition("capturing")

    def update_health(self, values: Mapping[str, Any]) -> None:
        allowed = set(self.health)
        unknown = set(values) - allowed
        if unknown:
            raise SessionError("unknown health keys: %s" % sorted(unknown))
        with self._lock:
            for key, value in values.items():
                if value not in {"unknown", "ready", "warning", "fault"}:
                    raise SessionError("invalid health state for %s" % key)
                self.health[key] = value
            now = time.time()
            self.health_updated_at = now
            self.updated_at = now
            self._persist()

    def touch(self) -> None:
        with self._lock:
            self.updated_at = time.time()
            self._persist()

    def update_context(self, values: Mapping[str, Any]) -> None:
        with self._lock:
            self.context.update(dict(values))
            self.updated_at = time.time()
            self._persist()

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            health_ready = all(value == "ready" for value in self.health.values())
            return {
                "schema": "go2.delivery_session.v1",
                "sessionId": self.session_id,
                "siteId": self.site_id,
                "operator": self.operator,
                "state": self.state,
                "startedAt": self.started_at,
                "updatedAt": self.updated_at,
                "healthUpdatedAt": self.health_updated_at,
                "health": dict(self.health),
                "healthReady": health_ready,
                "context": dict(self.context),
                "coverage": self.coverage.to_dict(),
                "review": self.review.to_dict(),
            }

    def _persist(self) -> None:
        if self.state_path is None:
            return
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        state = self.snapshot()
        state["persistence"] = {
            "schema": "go2.delivery_session_state.v1",
            "coverage": self.coverage.to_state_dict(),
        }
        payload = json.dumps(
            state,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
        )
        descriptor, temporary_path = tempfile.mkstemp(
            prefix=self.state_path.name + ".",
            dir=str(self.state_path.parent),
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                handle.write(payload)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_path, self.state_path)
        finally:
            if os.path.exists(temporary_path):
                os.unlink(temporary_path)

    def _load(self) -> None:
        try:
            payload = json.loads(self.state_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise SessionError("cannot restore delivery session: %s" % exc)
        if not isinstance(payload, dict) or payload.get("schema") != "go2.delivery_session.v1":
            raise SessionError("unsupported delivery session schema")
        state = str(payload.get("state", ""))
        if state not in ALLOWED_TRANSITIONS:
            raise SessionError("invalid persisted delivery state")
        health = payload.get("health")
        if not isinstance(health, dict) or set(health) != set(self.health):
            raise SessionError("persisted health record is invalid")
        if any(value not in {"unknown", "ready", "warning", "fault"} for value in health.values()):
            raise SessionError("persisted health state is invalid")
        review_payload = payload.get("review")
        if not isinstance(review_payload, dict):
            raise SessionError("persisted review queue is missing")
        persistence = payload.get("persistence")
        if (
            not isinstance(persistence, dict)
            or persistence.get("schema") != "go2.delivery_session_state.v1"
            or not isinstance(persistence.get("coverage"), dict)
        ):
            raise SessionError(
                "persisted session lacks exact coverage evidence; start a new session"
            )
        try:
            coverage = CoverageMap.from_state_dict(persistence["coverage"])
            review = ReviewQueue.from_dict(review_payload)
        except (TypeError, ValueError) as exc:
            raise SessionError("persisted session evidence is invalid: %s" % exc)
        context = payload.get("context", {})
        if not isinstance(context, dict):
            raise SessionError("persisted context is invalid")
        self.session_id = str(payload.get("sessionId", "")).strip()
        self.site_id = str(payload.get("siteId", ""))
        self.operator = str(payload.get("operator", ""))
        if not self.session_id:
            raise SessionError("persisted session id is missing")
        self.state = state
        self.started_at = payload.get("startedAt")
        self.updated_at = float(payload.get("updatedAt", time.time()))
        health_updated_at = payload.get("healthUpdatedAt")
        self.health_updated_at = (
            float(health_updated_at) if health_updated_at is not None else None
        )
        self.health = dict(health)
        self.coverage = coverage
        self.review = review
        self.context = dict(context)
        self.loaded = True
