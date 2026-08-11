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

from gogoguard_contracts import utc_now, validate_platform_mission_message


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
        timeout_s: float = 3.0,
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
        self.context = _read_json(self.state_path)
        self._inbox_offset = 0
        self._seen_messages: set[str] = set()
        self._pending_messages: list[dict[str, Any]] = []

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
        runtime = navigation.get("runtime")
        runtime = runtime if isinstance(runtime, dict) else navigation
        checkpoint = runtime.get("checkpoint")
        checkpoint = checkpoint if isinstance(checkpoint, dict) else {}
        mission_id = str(checkpoint.get("missionId") or "")
        checkpoint_id = str(checkpoint.get("activeCheckpointId") or "")
        phase = str(checkpoint.get("phase") or "TRAVELING")

        if not checkpoint_id:
            if self.context.get("checkpointId") and phase == "TRAVELING":
                self._event(runtime, self.context, "resuming")
                self._event(runtime, self.context, "completed")
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
                "stageAt": instant,
                "mapVersion": str(runtime.get("mapVersion") or ""),
                "routeId": str(runtime.get("routeId") or ""),
                "routeProgressIndex": int(
                    checkpoint.get("activeRouteProgressIndex")
                    if checkpoint.get("activeRouteProgressIndex") is not None
                    else runtime.get("routeProgressIndex") or 0
                ),
            }
            self._save()

        messages = self._messages()
        if phase == "WAITING_PLATFORM":
            self._waiting_platform(runtime, checkpoint, messages, instant)
        elif phase == "WAITING_VERDICT":
            self._waiting_verdict(runtime, checkpoint, messages, instant)

    def _waiting_platform(
        self,
        runtime: dict[str, Any],
        checkpoint: dict[str, Any],
        messages: list[dict[str, Any]],
        now: float,
    ) -> None:
        stage = str(self.context.get("stage") or "new")
        if stage == "new":
            self._event(runtime, self.context, "checkpoint_reached")
            self._event(runtime, self.context, "stopped")
            camera = checkpoint.get("camera") if isinstance(checkpoint.get("camera"), dict) else {}
            if self.gimbal is not None:
                gimbal_result = self.gimbal.move_for_inspection(
                    pan_body_deg=float(camera.get("pan") or 0.0),
                    tilt_euler_deg=float(camera.get("tilt") or 0.0),
                    roll_euler_deg=float(camera.get("roll") or 0.0),
                )
                actual = (
                    gimbal_result.get("angles")
                    if isinstance(gimbal_result, dict)
                    and isinstance(gimbal_result.get("angles"), dict)
                    else None
                )
                if actual is not None:
                    values = {
                        name: float(actual.get(name) or 0.0)
                        for name in ("pan", "tilt", "roll")
                    }
                    if all(math.isfinite(value) for value in values.values()):
                        self.context["camera"] = values
            self._event(runtime, self.context, "pose_ready")
            response = self._event(
                runtime,
                self.context,
                "announcing",
                announcementTimeoutSec=20,
            )
            announcement = response.get("announcement")
            announcement = announcement if isinstance(announcement, dict) else {}
            self.context["announcementId"] = announcement.get("announcementId")
            if announcement.get("proceed") is False:
                self.context["stage"] = "announcing"
            else:
                self.context["stage"] = "dwelling"
            self.context["stageAt"] = now
            self._save()
            stage = self.context["stage"]
        if stage == "announcing":
            completed = next(
                (
                    item for item in messages
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
                self._event(runtime, self.context, "platform_timeout")
                self.context["stage"] = "dwelling"
                self.context["stageAt"] = now
                self._save()
        if self.context.get("stage") == "dwelling":
            dwell_s = max(1.0, float(checkpoint.get("dwellSec") or 1.0))
            if now - float(self.context.get("stageAt") or now) >= dwell_s:
                self._event(runtime, self.context, "capture_ready")
                self.context["stage"] = "capture_requested"
                self.context["stageAt"] = now
                self._write_control("capture")
                self._save()

    def _waiting_verdict(
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
            self._event(runtime, self.context, "captured", evidenceId=evidence_id)
            response = self._event(
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
                item for item in messages
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
                            {"verdictId": verdict_id, "receivedAt": utc_now()},
                            self.timeout_s,
                        )
                    except Exception:
                        pass
                    self.context["stage"] = f"verdict_{action}"
                    self._save()
                    return
        if now - float(self.context.get("stageAt") or now) >= timeout:
            self._event(runtime, self.context, "platform_timeout")
            self._write_control("timeout")
            self.context["stage"] = "verdict_timeout"
            self._save()

    def _event(
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
                str(context.get("missionId")), str(context.get("checkpointId")),
                str(context.get("attempt")), phase, str(context.get("stage") or ""),
            ]
        )
        payload = {
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
