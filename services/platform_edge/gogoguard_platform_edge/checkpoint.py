from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from gogoguard_contracts import (
    EvidenceContractError,
    ReceiptValidationError,
    capture_request,
    extract_capture_response,
    utc_now,
    validate_capture_response_envelope,
    validate_platform_mission_message,
    validate_receipt,
)


PHASES = {
    "checkpoint_reached", "stopped", "pose_ready", "announcing",
    "capture_ready", "captured", "waiting_verdict", "resuming",
    "platform_timeout", "completed",
}


class LocalGimbalClient:
    """Narrow localhost HTTP port; platform_edge never imports device drivers."""

    def __init__(
        self,
        url: str = "http://127.0.0.1:8080/api/v1/gimbal/move",
        *,
        timeout_s: float = 5.0,
    ) -> None:
        self.url = url
        self.timeout_s = float(timeout_s)

    def move_for_inspection(
        self,
        *,
        pan_body_deg: float,
        tilt_euler_deg: float,
        roll_euler_deg: float,
    ) -> dict[str, Any]:
        content = json.dumps(
            {
                "pan": float(pan_body_deg),
                "tilt": float(tilt_euler_deg),
                "roll": float(roll_euler_deg),
            },
            separators=(",", ":"),
            allow_nan=False,
        ).encode()
        request = Request(
            self.url,
            data=content,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urlopen(request, timeout=self.timeout_s) as response:
            value = json.loads(response.read(65537).decode("utf-8"))
        if not isinstance(value, dict):
            raise RuntimeError("local gimbal endpoint returned an invalid response")
        return value


def mission_url(heartbeat_url: str, suffix: str) -> str:
    endpoint = urlsplit(heartbeat_url)
    if not endpoint.path.endswith("/robot/heartbeat"):
        raise ValueError("platform heartbeat URL cannot derive mission URL")
    return endpoint._replace(
        path=endpoint.path[: -len("heartbeat")] + "mission/" + suffix.lstrip("/")
    ).geturl()


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, 0o640)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class CheckpointCoordinator:
    """Connect the stopped Nav2 checkpoint to the frozen platform workflow."""

    GIMBAL_POSE_TOLERANCE_DEG = 2.0

    def __init__(
        self,
        *,
        heartbeat_url: str,
        navigation_status_path: Path,
        control_path: Path,
        inbox_path: Path,
        state_path: Path,
        post_json: Callable[[str, dict[str, Any], float], dict[str, Any]],
        gimbal: Any | None = None,
        timeout_s: float = 10.0,
        evidence_transaction_enabled: bool = False,
        robot_id: str = "LLYJ0001",
        max_capture_reconcile_queries: int = 6,
        navigation_generation_id: str | None = None,
        max_event_age_s: float = 600.0,
        stale_event_audit_path: Path | None = None,
    ) -> None:
        self.event_url = mission_url(heartbeat_url, "event")
        self.ack_url = mission_url(heartbeat_url, "verdict/ack")
        self.navigation_status_path = Path(navigation_status_path)
        self.control_path = Path(control_path)
        self.inbox_path = Path(inbox_path)
        self.state_path = Path(state_path)
        self.post_json = post_json
        self.gimbal = gimbal
        self.timeout_s = float(timeout_s)
        self.evidence_transaction_enabled = bool(evidence_transaction_enabled)
        self.robot_id = str(robot_id)
        self.navigation_generation_id = str(navigation_generation_id or "")
        self.max_event_age_s = max(60.0, float(max_event_age_s))
        self.stale_event_audit_path = Path(
            stale_event_audit_path
            or self.state_path.with_name("checkpoint-stale-events.jsonl")
        )
        self.max_capture_reconcile_queries = max(
            1, int(max_capture_reconcile_queries)
        )
        self.context = _read_json(self.state_path)
        # Monotonic timestamps do not survive a reboot.  A durable outbox is
        # retried once after process start and then returns to bounded backoff.
        for pending in self.context.get("pendingEvents", []):
            if isinstance(pending, dict):
                pending["nextAttemptAt"] = 0.0
        self._inbox_offset = 0
        self._seen_messages: set[str] = {
            str(value)
            for value in self.context.get("appliedMessageIds", [])
            if isinstance(value, str) and value
        }
        self._pending_messages: list[dict[str, Any]] = []

    def _move_camera_for_checkpoint(
        self, checkpoint: dict[str, Any]
    ) -> dict[str, Any] | None:
        camera = (
            checkpoint.get("camera")
            if isinstance(checkpoint.get("camera"), dict)
            else {}
        )
        if self.gimbal is None:
            return None
        targets = {
            "pan": float(camera.get("pan") or 0.0),
            "tilt": float(camera.get("tilt") or 0.0),
            "roll": float(camera.get("roll") or 0.0),
        }
        gimbal_result = self.gimbal.move_for_inspection(
            pan_body_deg=targets["pan"],
            tilt_euler_deg=targets["tilt"],
            roll_euler_deg=targets["roll"],
        )
        actual = (
            gimbal_result.get("angles")
            if isinstance(gimbal_result, dict)
            and isinstance(gimbal_result.get("angles"), dict)
            else None
        )
        if actual is None:
            raise RuntimeError("Z1Pro checkpoint move returned no angle feedback")
        values = {
            name: float(actual.get(name, math.nan))
            for name in ("pan", "tilt", "roll")
        }
        if any(not math.isfinite(value) for value in values.values()):
            raise RuntimeError("Z1Pro checkpoint feedback is invalid")
        errors = {name: abs(values[name] - targets[name]) for name in values}
        if any(
            value > self.GIMBAL_POSE_TOLERANCE_DEG
            for value in errors.values()
        ):
            raise RuntimeError(
                "Z1Pro did not reach checkpoint view: "
                + ", ".join(
                    f"{name} error={value:.1f}deg"
                    for name, value in errors.items()
                )
            )
        pose = {
            "targetDeg": targets,
            "actualDeg": values,
            "absoluteErrorDeg": errors,
            "toleranceDeg": self.GIMBAL_POSE_TOLERANCE_DEG,
            "converged": True,
        }
        # Keep the former flat field for local compatibility, but persist the
        # complete proof used by the durable pose_ready event.
        self.context["camera"] = values
        self.context["cameraPose"] = pose
        return pose

    def run(self, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            try:
                self.tick()
                self.context.pop("lastError", None)
            except Exception as exc:
                self.context["lastError"] = type(exc).__name__
                self._save()
            stop_event.wait(0.2)

    def tick(self, *, now: float | None = None) -> None:
        instant = time.monotonic() if now is None else float(now)
        navigation = _read_json(self.navigation_status_path)
        if (
            self.navigation_generation_id
            and navigation.get("edge_generation_id")
            != self.navigation_generation_id
        ):
            return
        runtime = navigation.get("runtime")
        runtime = runtime if isinstance(runtime, dict) else navigation
        checkpoint = runtime.get("checkpoint")
        checkpoint = checkpoint if isinstance(checkpoint, dict) else {}
        if not (
            self.evidence_transaction_enabled
            and isinstance(checkpoint.get("evidenceTransactionVersion"), int)
            and not isinstance(checkpoint.get("evidenceTransactionVersion"), bool)
            and checkpoint.get("evidenceTransactionVersion") == 1
        ):
            self._legacy_tick(runtime, checkpoint, instant)
            return
        if str(checkpoint.get("decisionMode") or "platform") != "platform":
            self._flush_pending_events(instant)
            self._clear_checkpoint_context()
            return
        mission_id = str(checkpoint.get("missionId") or "")
        checkpoint_id = str(checkpoint.get("activeCheckpointId") or "")
        phase = str(checkpoint.get("phase") or "TRAVELING")

        if not checkpoint_id:
            if self.context.get("checkpointId") and phase == "TRAVELING":
                if self.context.get("stage") != "completion_pending":
                    self._queue_event(runtime, self.context, "resuming")
                    self._queue_event(runtime, self.context, "completed")
                    self.context["stage"] = "completion_pending"
                    self._save()
            self._flush_pending_events(instant)
            if (
                self.context.get("stage") == "completion_pending"
                and not self.context.get("pendingEvents")
            ):
                self._clear_checkpoint_context()
            return
        attempt = int(checkpoint.get("attempt") or 1)
        identity = (mission_id, checkpoint_id, attempt)
        current = (
            self.context.get("missionId"),
            self.context.get("checkpointId"),
            int(self.context.get("attempt") or 0),
        )
        if current != identity:
            pending_events = list(self.context.get("pendingEvents") or [])
            self.context = {
                "missionId": mission_id,
                "checkpointId": checkpoint_id,
                "attempt": attempt,
                "stage": "new",
                "stageAt": instant,
                "mapVersion": str(runtime.get("mapVersion") or ""),
                "routeId": str(runtime.get("routeId") or ""),
                "routeProgressIndex": int(
                    checkpoint.get("activeRouteProgressIndex")
                    if checkpoint.get("activeRouteProgressIndex") is not None
                    else runtime.get("routeProgressIndex") or 0
                ),
                "verdictTimeoutSec": int(
                    checkpoint.get("verdictTimeoutSec") or 15
                ),
            }
            if pending_events:
                self.context["pendingEvents"] = pending_events
            self._seen_messages = set()
            self._save()

        # Inbox terminal messages are always inspected before an event POST.
        # Platform notification failures must never hold a safe local action.
        messages = self._messages()
        if phase == "WAITING_PLATFORM":
            self._waiting_platform(runtime, checkpoint, messages, instant)
        elif phase == "WAITING_VERDICT":
            self._waiting_verdict(runtime, checkpoint, messages, instant)

        responses = self._flush_pending_events(instant)
        for event, response in responses:
            response_messages = self._apply_event_response(event, response, instant)
            if phase == "WAITING_PLATFORM":
                self._apply_announcement_messages(response_messages, instant)
            elif phase == "WAITING_VERDICT":
                self._apply_verdict_messages(response_messages)

    def _legacy_tick(
        self,
        runtime: dict[str, Any],
        checkpoint: dict[str, Any],
        now: float,
    ) -> None:
        """Frozen pre-transaction behavior kept behind the default-off gate."""

        if str(checkpoint.get("decisionMode") or "platform") != "platform":
            if self.context:
                self.context = {}
                self._save()
            return
        mission_id = str(checkpoint.get("missionId") or "")
        checkpoint_id = str(checkpoint.get("activeCheckpointId") or "")
        phase = str(checkpoint.get("phase") or "TRAVELING")
        if not checkpoint_id:
            if self.context.get("checkpointId") and phase == "TRAVELING":
                self._legacy_event(runtime, self.context, "resuming")
                self._legacy_event(runtime, self.context, "completed")
                self.context = {}
                self._save()
            return
        attempt = int(checkpoint.get("attempt") or 1)
        identity = (mission_id, checkpoint_id, attempt)
        current = (
            self.context.get("missionId"),
            self.context.get("checkpointId"),
            int(self.context.get("attempt") or 0),
        )
        if current != identity:
            self.context = {
                "missionId": mission_id,
                "checkpointId": checkpoint_id,
                "attempt": attempt,
                "stage": "new",
                "stageAt": now,
                "mapVersion": str(runtime.get("mapVersion") or ""),
                "routeId": str(runtime.get("routeId") or ""),
                "routeProgressIndex": int(
                    checkpoint.get("activeRouteProgressIndex")
                    if checkpoint.get("activeRouteProgressIndex") is not None
                    else runtime.get("routeProgressIndex") or 0
                ),
            }
            self._seen_messages = set()
            self._save()
        messages = self._messages()
        if phase == "WAITING_PLATFORM":
            self._legacy_waiting_platform(runtime, checkpoint, messages, now)
        elif phase == "WAITING_VERDICT":
            self._legacy_waiting_verdict(runtime, checkpoint, messages, now)

    def _legacy_waiting_platform(
        self,
        runtime: dict[str, Any],
        checkpoint: dict[str, Any],
        messages: list[dict[str, Any]],
        now: float,
    ) -> None:
        stage = str(self.context.get("stage") or "new")
        if stage == "new":
            self._legacy_event(runtime, self.context, "checkpoint_reached")
            self._legacy_event(runtime, self.context, "stopped")
            camera_pose = self._move_camera_for_checkpoint(checkpoint)
            self._legacy_event(
                runtime,
                self.context,
                "pose_ready",
                **({"cameraPose": camera_pose} if camera_pose is not None else {}),
            )
            response = self._legacy_event(
                runtime,
                self.context,
                "announcing",
                announcementTimeoutSec=20,
            )
            announcement = response.get("announcement")
            announcement = announcement if isinstance(announcement, dict) else {}
            self.context["announcementId"] = announcement.get("announcementId")
            self.context["stage"] = (
                "announcing" if announcement.get("proceed") is False else "dwelling"
            )
            self.context["stageAt"] = now
            self._save()
            stage = self.context["stage"]
        if stage == "announcing":
            completed = next(
                (
                    item
                    for item in messages
                    if item.get("schema") == "gogoguard.announcement_completed.v1"
                    and item.get("announcementId") == self.context.get("announcementId")
                    and self._matches(item)
                ),
                None,
            )
            if completed is not None:
                self.context["stage"] = "dwelling"
                self.context["stageAt"] = now
                self._save()
            elif now - float(self.context.get("stageAt") or now) >= 20.0:
                self._legacy_event(runtime, self.context, "platform_timeout")
                self.context["stage"] = "dwelling"
                self.context["stageAt"] = now
                self._save()
        if self.context.get("stage") == "dwelling":
            dwell_s = max(1.0, float(checkpoint.get("dwellSec") or 1.0))
            if now - float(self.context.get("stageAt") or now) >= dwell_s:
                self._legacy_event(runtime, self.context, "capture_ready")
                self.context["stage"] = "capture_requested"
                self.context["stageAt"] = now
                self._write_control("capture")
                self._save()

    def _legacy_waiting_verdict(
        self,
        runtime: dict[str, Any],
        checkpoint: dict[str, Any],
        messages: list[dict[str, Any]],
        now: float,
    ) -> None:
        timeout = int(checkpoint.get("verdictTimeoutSec") or 15)
        if self.context.get("stage") != "waiting_verdict":
            evidence_id = "ev_" + hashlib.sha256(
                f"{self.context['missionId']}:{self.context['checkpointId']}:{self.context['attempt']}".encode()
            ).hexdigest()[:20]
            self._legacy_event(runtime, self.context, "captured", evidenceId=evidence_id)
            response = self._legacy_event(
                runtime,
                self.context,
                "waiting_verdict",
                evidenceId=evidence_id,
                captured=True,
                verdictTimeoutSec=timeout,
            )
            self.context.update(
                {"stage": "waiting_verdict", "stageAt": now, "evidenceId": evidence_id}
            )
            verdict = response.get("verdict")
            if isinstance(verdict, dict):
                messages.append(verdict)
            self._save()
        verdict = next(
            (
                item
                for item in messages
                if item.get("schema") == "gogoguard.checkpoint_verdict.v1"
                and self._matches(item)
                and not self._expired(item.get("expiresAt"))
            ),
            None,
        )
        if verdict is not None:
            verdict_id = str(verdict.get("verdictId") or "")
            if verdict_id and verdict_id not in self._seen_messages:
                action = str(verdict.get("action") or "")
                if action in {"continue", "retake", "skip"}:
                    self._seen_messages.add(verdict_id)
                    self._write_control(action)
                    try:
                        self.post_json(
                            self.ack_url,
                            {
                                "robotId": self.robot_id,
                                "verdictId": verdict_id,
                                "receivedAt": utc_now(),
                            },
                            self.timeout_s,
                        )
                    except Exception:
                        pass
                    self.context["stage"] = f"verdict_{action}"
                    self._save()
                    return
        if now - float(self.context.get("stageAt") or now) >= timeout:
            self._legacy_event(runtime, self.context, "platform_timeout")
            self._write_control("timeout")
            self.context["stage"] = "verdict_timeout"
            self._save()

    def _legacy_event(
        self,
        runtime: dict[str, Any],
        context: dict[str, Any],
        phase: str,
        **extra: Any,
    ) -> dict[str, Any]:
        if phase not in PHASES:
            raise ValueError("mission event phase is invalid")
        seed = ":".join(
            [
                str(context.get("missionId")),
                str(context.get("checkpointId")),
                str(context.get("attempt")),
                phase,
                str(context.get("stage") or ""),
            ]
        )
        payload = {
            "robotId": self.robot_id,
            "eventId": "me_" + hashlib.sha256(seed.encode()).hexdigest()[:28],
            "missionId": context["missionId"],
            "mapVersion": context.get("mapVersion") or runtime.get("mapVersion"),
            "routeId": context.get("routeId") or runtime.get("routeId"),
            "checkpointId": context["checkpointId"],
            "routeProgressIndex": int(context["routeProgressIndex"]),
            "attempt": int(context["attempt"]),
            "phase": phase,
            "observedAt": utc_now(),
            **extra,
        }
        return self.post_json(self.event_url, payload, self.timeout_s)

    def _waiting_platform(
        self,
        runtime: dict[str, Any],
        checkpoint: dict[str, Any],
        messages: list[dict[str, Any]],
        now: float,
    ) -> None:
        stage = str(self.context.get("stage") or "new")
        self._apply_announcement_messages(messages, now)
        stage = str(self.context.get("stage") or stage)
        if stage == "capture_succeeded_pending_control":
            self._ensure_capture_success_control(now)
            return
        if stage == "capture_failed":
            self._ensure_capture_failure_control()
            return
        if stage == "new":
            self._queue_event(runtime, self.context, "checkpoint_reached")
            self._queue_event(runtime, self.context, "stopped")
            camera_pose = self._move_camera_for_checkpoint(checkpoint)
            self._queue_event(
                runtime,
                self.context,
                "pose_ready",
                **({"cameraPose": camera_pose} if camera_pose is not None else {}),
            )
            self._queue_event(
                runtime,
                self.context,
                "announcing",
                announcementTimeoutSec=20,
            )
            self.context["stage"] = (
                "dwelling" if self.context.get("announcementCompleted") else "announcing"
            )
            self.context["stageAt"] = now
            self._save()
            stage = self.context["stage"]
        if stage == "announcing":
            if self.context.get("announcementCompleted"):
                self.context["stage"] = "dwelling"
                self.context["stageAt"] = now
                self._save()
            elif now - float(self.context.get("stageAt") or now) >= 20.0:
                self._queue_event(runtime, self.context, "platform_timeout")
                self.context["stage"] = "dwelling"
                self.context["stageAt"] = now
                self._save()
        if self.context.get("stage") == "dwelling":
            dwell_s = max(1.0, float(checkpoint.get("dwellSec") or 1.0))
            if now - float(self.context.get("stageAt") or now) >= dwell_s:
                request = capture_request(
                    mission_id=str(self.context["missionId"]),
                    map_version=str(self.context.get("mapVersion") or ""),
                    route_id=str(self.context.get("routeId") or ""),
                    checkpoint_id=str(self.context["checkpointId"]),
                    attempt=int(self.context["attempt"]),
                )
                self.context["captureRequestId"] = request["captureRequestId"]
                self.context["capturePhase"] = "CAPTURE_REQUESTED"
                self._queue_event(
                    runtime,
                    self.context,
                    "capture_ready",
                    **request,
                )
                self.context["stage"] = "capture_requested"
                self.context["stageAt"] = now
                self._save()

    def _waiting_verdict(
        self,
        runtime: dict[str, Any],
        checkpoint: dict[str, Any],
        messages: list[dict[str, Any]],
        now: float,
    ) -> None:
        timeout = int(checkpoint.get("verdictTimeoutSec") or 15)
        if str(self.context.get("stage") or "").startswith("verdict_"):
            return
        if self._apply_verdict_messages(messages):
            return
        if self.context.get("stage") in {
            "capture_requested",
            "capture_reconciling",
            "capture_failed",
            "capture_rejected",
        }:
            if self.context.get("stage") == "capture_failed":
                self._ensure_capture_failure_control()
            return
        if self.context.get("stage") != "waiting_verdict":
            # A v1 checkpoint never fabricates an evidence identifier.  It
            # remains stopped until a validated platform receipt is durable.
            return
        if now - float(self.context.get("stageAt") or now) >= timeout:
            self._queue_event(runtime, self.context, "platform_timeout")
            self._write_control("timeout")
            self.context["stage"] = "verdict_timeout"
            self._save()

    def _queue_event(
        self,
        runtime: dict[str, Any],
        context: dict[str, Any],
        phase: str,
        **extra: Any,
    ) -> str:
        if phase not in PHASES:
            raise ValueError("mission event phase is invalid")
        seed = ":".join(
            [
                str(context.get("missionId")), str(context.get("checkpointId")),
                str(context.get("attempt")), phase, str(context.get("stage") or ""),
            ]
        )
        payload = {
            "robotId": self.robot_id,
            "eventId": "me_" + hashlib.sha256(seed.encode()).hexdigest()[:28],
            "missionId": context["missionId"],
            "mapVersion": context.get("mapVersion") or runtime.get("mapVersion"),
            "routeId": context.get("routeId") or runtime.get("routeId"),
            "checkpointId": context["checkpointId"],
            "routeProgressIndex": int(context["routeProgressIndex"]),
            "attempt": int(context["attempt"]),
            "phase": phase,
            "observedAt": utc_now(),
            **extra,
        }
        pending_events = self.context.setdefault("pendingEvents", [])
        if not any(
            isinstance(item, dict)
            and isinstance(item.get("payload"), dict)
            and item["payload"].get("eventId") == payload["eventId"]
            for item in pending_events
        ):
            pending_events.append(
                {
                    "payload": payload,
                    "attempts": 0,
                    "nextAttemptAt": 0.0,
                }
            )
            del pending_events[:-128]
            self._save()
        return str(payload["eventId"])

    def _flush_pending_events(
        self, now: float
    ) -> list[tuple[dict[str, Any], dict[str, Any]]]:
        pending_events = self.context.get("pendingEvents")
        if not isinstance(pending_events, list) or not pending_events:
            return []
        responses: list[tuple[dict[str, Any], dict[str, Any]]] = []
        retained: list[dict[str, Any]] = []
        for pending in pending_events:
            if not isinstance(pending, dict) or not isinstance(pending.get("payload"), dict):
                continue
            payload = pending["payload"]
            stale_reason = self._stale_event_reason(payload)
            if stale_reason is not None:
                self._audit_stale_event(payload, stale_reason)
                continue
            # Durable outbox entries created by pre-identity releases do not
            # have robotId.  Inject the commissioned identity at send time as
            # well as creation time so a restart drains the existing backlog
            # instead of replaying permanent 401s forever.
            if not str(payload.get("robotId") or "").strip():
                payload["robotId"] = self.robot_id
            if float(pending.get("nextAttemptAt") or 0.0) > now:
                retained.append(pending)
                continue
            try:
                response = self.post_json(self.event_url, payload, self.timeout_s)
                response = response if isinstance(response, dict) else {}
            except Exception as exc:
                attempts = int(pending.get("attempts") or 0) + 1
                status = getattr(exc, "code", None)
                status = status if isinstance(status, int) else None
                delay_s = (
                    60.0
                    if status in {401, 403}
                    else min(60.0, float(2 ** min(attempts - 1, 6)))
                )
                is_capture = payload.get("phase") == "capture_ready"
                if is_capture:
                    self.context["stage"] = "capture_reconciling"
                    self.context["capturePhase"] = "CAPTURE_RECONCILING"
                    self.context["captureReconcileQueries"] = attempts
                pending.update(
                    {
                        "attempts": attempts,
                        "nextAttemptAt": now + delay_s,
                        "lastErrorCode": (
                            f"HTTP_{status}" if status is not None else type(exc).__name__
                        ),
                    }
                )
                exhausted = is_capture and attempts >= self.max_capture_reconcile_queries
                if exhausted:
                    if status is not None and status >= 500:
                        failure_code = "CAPTURE_UPSTREAM_ERROR"
                    else:
                        failure_code = "CAPTURE_TRANSPORT_UNAVAILABLE"
                    self._capture_failed(
                        failure_code,
                        detail_code=pending["lastErrorCode"],
                    )
                else:
                    retained.append(pending)
                self.context["lastEventFailure"] = {
                    "phase": str(payload.get("phase") or "unknown"),
                    "code": pending["lastErrorCode"],
                    "httpStatus": status,
                    "attempts": attempts,
                    "retryAt": now + delay_s,
                }
            else:
                responses.append((payload, response))
        self.context["pendingEvents"] = retained
        if not retained:
            self.context.pop("pendingEvents", None)
            self.context.pop("lastEventFailure", None)
        self._save()
        return responses

    def _stale_event_reason(self, payload: dict[str, Any]) -> str | None:
        observed_at = payload.get("observedAt")
        if not isinstance(observed_at, str) or not observed_at:
            return "OBSERVED_AT_MISSING"
        try:
            parsed = datetime.fromisoformat(observed_at.replace("Z", "+00:00"))
        except ValueError:
            return "OBSERVED_AT_INVALID"
        if parsed.tzinfo is None:
            return "OBSERVED_AT_TIMEZONE_MISSING"
        age_s = (datetime.now(timezone.utc) - parsed).total_seconds()
        if age_s > self.max_event_age_s:
            return "EVENT_TOO_OLD"
        if age_s < -60.0:
            return "OBSERVED_AT_IN_FUTURE"
        return None

    def _audit_stale_event(
        self, payload: dict[str, Any], reason: str
    ) -> None:
        audit = {
            "discardedAt": utc_now(),
            "eventId": str(payload.get("eventId") or ""),
            "missionId": str(payload.get("missionId") or ""),
            "checkpointId": str(payload.get("checkpointId") or ""),
            "phase": str(payload.get("phase") or ""),
            "observedAt": payload.get("observedAt"),
            "reason": reason,
        }
        self.stale_event_audit_path.parent.mkdir(parents=True, exist_ok=True)
        with self.stale_event_audit_path.open("a", encoding="utf-8") as handle:
            json.dump(audit, handle, ensure_ascii=False, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(self.stale_event_audit_path, 0o640)
        self.context["staleEventsDiscarded"] = int(
            self.context.get("staleEventsDiscarded") or 0
        ) + 1
        self.context["lastStaleEvent"] = audit

    def _apply_event_response(
        self,
        event: dict[str, Any],
        response: dict[str, Any],
        now: float,
    ) -> list[dict[str, Any]]:
        values: list[dict[str, Any]] = []
        if event.get("phase") == "announcing":
            announcement = response.get("announcement")
            if isinstance(announcement, dict):
                announcement_id = announcement.get("announcementId")
                expected_id = str(self.context.get("announcementId") or "")
                if (
                    isinstance(announcement_id, str)
                    and announcement_id
                    and (not expected_id or expected_id == announcement_id)
                ):
                    self.context["announcementId"] = announcement_id
                if (
                    announcement.get("proceed") is True
                    and self.context.get("stage") == "announcing"
                ):
                    self.context["stage"] = "dwelling"
                    self.context["stageAt"] = now
                    self._save()
        if event.get("phase") == "capture_ready":
            self._apply_capture_response(event, response, now)
        verdict = response.get("verdict")
        if isinstance(verdict, dict):
            values.append(verdict)
        return values

    def _apply_capture_response(
        self,
        event: dict[str, Any],
        value: dict[str, Any],
        now: float,
    ) -> None:
        if str(self.context.get("stage") or "").startswith("verdict_"):
            return
        try:
            response = validate_capture_response_envelope(
                extract_capture_response(value)
            )
        except EvidenceContractError as exc:
            self.context.update(
                {
                    "stage": "capture_rejected",
                    "stageAt": now,
                    "captureRejection": {
                        "class": "contract",
                        "code": exc.code,
                        "retryable": False,
                    },
                    "capturePhase": "CAPTURE_REJECTED",
                }
            )
            self._save()
            return
        expected_request_id = str(self.context.get("captureRequestId") or "")
        if response.get("captureRequestId") != expected_request_id:
            self._capture_failed(
                "RECEIPT_BINDING_MISMATCH",
                detail_code="CAPTURE_REQUEST_ID_MISMATCH",
            )
            return
        if response["accepted"] is False:
            rejection = dict(response["rejection"])
            self.context.update(
                {
                    "stage": "capture_rejected",
                    "stageAt": now,
                    "captureRejection": rejection,
                    "capturePhase": "CAPTURE_REJECTED",
                }
            )
            self._save()
            return
        state = str(response["state"])
        if state == "failed":
            self._capture_failed(str(response["failureCode"]))
            return
        if state in {"selecting", "processing"}:
            queries = int(self.context.get("captureReconcileQueries") or 0) + 1
            if queries >= self.max_capture_reconcile_queries:
                self._capture_failed("CAPTURE_OUTCOME_UNKNOWN")
                return
            retry_after_ms = response.get("retryAfterMs")
            retry_after_ms = retry_after_ms if isinstance(retry_after_ms, int) else 500
            self.context.update(
                {
                    "stage": "capture_reconciling",
                    "stageAt": now,
                    "captureReconcileQueries": queries,
                    "captureBusinessState": state,
                    "capturePhase": "CAPTURE_RECONCILING",
                }
            )
            pending = self.context.setdefault("pendingEvents", [])
            pending.append(
                {
                    "payload": dict(event),
                    "attempts": queries,
                    "nextAttemptAt": now + min(5.0, max(0.2, retry_after_ms / 1000.0)),
                }
            )
            self._save()
            return
        try:
            receipt = validate_receipt(
                response.get("receipt"),
                expected_robot_id=self.robot_id,
                expected_capture_request_id=expected_request_id,
                expected_mission_id=str(self.context.get("missionId") or ""),
                expected_checkpoint_id=str(self.context.get("checkpointId") or ""),
                expected_attempt=int(self.context.get("attempt") or 0),
                expected_map_version=str(self.context.get("mapVersion") or ""),
                expected_route_id=str(self.context.get("routeId") or ""),
                previous_capture_id=str(self.context.get("captureId") or ""),
            )
        except ReceiptValidationError as exc:
            self._capture_failed(exc.code)
            return
        capture_id = str(receipt["captureId"])
        self.context.update(
            {
                "captureId": capture_id,
                "captureReceipt": receipt,
                "stage": "capture_succeeded_pending_control",
                "stageAt": now,
                "capturePhase": "CAPTURED",
            }
        )
        self.context.pop("captureReconcileQueries", None)
        self.context.pop("captureBusinessState", None)
        self._save()
        self._ensure_capture_success_control(now)

    def _ensure_capture_success_control(self, now: float) -> None:
        capture_id = str(self.context.get("captureId") or "")
        if not capture_id or not isinstance(self.context.get("captureReceipt"), dict):
            self._capture_failed("RECEIPT_SCHEMA_INVALID")
            return
        # Receipt durability is the safety boundary: movement-enabling capture
        # control is never written until the real platform receipt is on disk.
        self._write_control("capture")
        self.context.update(
            {
                "captureControlIssued": True,
                "stage": "waiting_verdict",
                "stageAt": now,
            }
        )
        self._queue_event({}, self.context, "captured", captureId=capture_id)
        self._queue_event(
            {},
            self.context,
            "waiting_verdict",
            captureId=capture_id,
            captured=True,
            verdictTimeoutSec=int(self.context.get("verdictTimeoutSec") or 15),
        )
        self._save()

    def _capture_failed(self, failure_code: str, *, detail_code: str = "") -> None:
        self.context.update(
            {
                "stage": "capture_failed",
                "stageAt": time.monotonic(),
                "captureFailureCode": str(failure_code),
                "capturePhase": "CAPTURE_FAILED",
            }
        )
        if detail_code:
            self.context["captureFailureDetailCode"] = str(detail_code)
        self.context.pop("captureBusinessState", None)
        self._save()
        self._ensure_capture_failure_control()

    def _ensure_capture_failure_control(self) -> None:
        if self.context.get("captureFailureControlIssued") is True:
            return
        # This control changes only the checkpoint protocol phase.  It does not
        # authorize spin or velocity, so an explicit retake can increment the
        # attempt while the robot remains stopped.
        self._write_control("capture_failed")
        self.context["captureFailureControlIssued"] = True
        self._save()

    def public_status(self) -> dict[str, Any]:
        fields = (
            "capturePhase",
            "captureRequestId",
            "captureId",
            "captureFailureCode",
            "captureFailureDetailCode",
            "captureBusinessState",
            "captureReconcileQueries",
            "captureRejection",
            "captureControlIssued",
            "captureFailureControlIssued",
        )
        return {
            name: self.context[name]
            for name in fields
            if name in self.context
        }

    def _apply_announcement_messages(
        self, messages: list[dict[str, Any]], now: float
    ) -> bool:
        for item in messages:
            if (
                item.get("schema") != "gogoguard.announcement_completed.v1"
                or not self._matches(item)
            ):
                continue
            announcement_id = str(item.get("announcementId") or "")
            expected_id = str(self.context.get("announcementId") or "")
            if not announcement_id or (expected_id and expected_id != announcement_id):
                continue
            if announcement_id in self._seen_messages:
                return False
            self.context["announcementId"] = announcement_id
            self.context["announcementCompleted"] = True
            self._remember_message(announcement_id)
            if self.context.get("stage") == "announcing":
                self.context["stage"] = "dwelling"
                self.context["stageAt"] = now
            self._save()
            return True
        return False

    def _apply_verdict_messages(self, messages: list[dict[str, Any]]) -> bool:
        for verdict in messages:
            if (
                verdict.get("schema") != "gogoguard.checkpoint_verdict.v1"
                or not self._matches(verdict)
                or self._expired(verdict.get("expiresAt"))
            ):
                continue
            verdict_id = str(verdict.get("verdictId") or "")
            action = str(verdict.get("action") or "")
            if (
                not verdict_id
                or verdict_id in self._seen_messages
                or action not in {"continue", "retake", "skip"}
            ):
                continue
            # The deterministic navigation control is durable before any
            # best-effort platform acknowledgement.
            self._write_control(action)
            self._remember_message(verdict_id)
            self.context["stage"] = f"verdict_{action}"
            self._save()
            try:
                self.post_json(
                    self.ack_url,
                    {
                        "robotId": self.robot_id,
                        "verdictId": verdict_id,
                        "receivedAt": utc_now(),
                    },
                    self.timeout_s,
                )
            except Exception:
                pass
            return True
        return False

    def _remember_message(self, message_id: str) -> None:
        self._seen_messages.add(message_id)
        applied = [
            str(value)
            for value in self.context.get("appliedMessageIds", [])
            if isinstance(value, str) and value
        ]
        if message_id not in applied:
            applied.append(message_id)
        self.context["appliedMessageIds"] = applied[-128:]

    def _clear_checkpoint_context(self) -> None:
        pending_events = list(self.context.get("pendingEvents") or [])
        last_failure = self.context.get("lastEventFailure")
        self.context = {}
        if pending_events:
            self.context["pendingEvents"] = pending_events
        if last_failure is not None:
            self.context["lastEventFailure"] = last_failure
        self._seen_messages = set()
        self._save()

    def _write_control(self, action: str) -> None:
        seed = f"{self.context['missionId']}:{self.context['checkpointId']}:{self.context['attempt']}:{action}"
        _atomic_json(
            self.control_path,
            {
                "schema": "gogoguard.checkpoint_control.v1",
                "controlId": "cc_" + hashlib.sha256(seed.encode()).hexdigest()[:28],
                "missionId": self.context["missionId"],
                "checkpointId": self.context["checkpointId"],
                "attempt": int(self.context["attempt"]),
                "action": action,
                "issuedAt": utc_now(),
            },
        )

    def _messages(self) -> list[dict[str, Any]]:
        if not self.inbox_path.is_file():
            return list(self._pending_messages)
        values: list[dict[str, Any]] = []
        with self.inbox_path.open("r", encoding="utf-8") as handle:
            if self.inbox_path.stat().st_size < self._inbox_offset:
                self._inbox_offset = 0
            handle.seek(self._inbox_offset)
            for line in handle:
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(value, dict):
                    try:
                        values.append(validate_platform_mission_message(value))
                    except ValueError:
                        continue
            self._inbox_offset = handle.tell()
        self._pending_messages.extend(values)
        self._pending_messages = self._pending_messages[-256:]
        return list(self._pending_messages)

    def _matches(self, value: dict[str, Any]) -> bool:
        try:
            attempt = int(value.get("attempt") or 0)
            current_attempt = int(self.context.get("attempt") or 0)
        except (TypeError, ValueError):
            return False
        if (
            value.get("missionId") != self.context.get("missionId")
            or value.get("checkpointId") != self.context.get("checkpointId")
            or attempt != current_attempt
        ):
            return False
        if value.get("schema") == "gogoguard.checkpoint_verdict.v1":
            return (
                value.get("mapVersion") == self.context.get("mapVersion")
                and value.get("routeId") == self.context.get("routeId")
            )
        return True

    @staticmethod
    def _expired(value: Any) -> bool:
        if not isinstance(value, str) or not value:
            return True
        try:
            parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
        except ValueError:
            return True
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        return parsed <= datetime.now(timezone.utc)

    def _save(self) -> None:
        if self.context:
            _atomic_json(self.state_path, self.context)
        else:
            self.state_path.unlink(missing_ok=True)
