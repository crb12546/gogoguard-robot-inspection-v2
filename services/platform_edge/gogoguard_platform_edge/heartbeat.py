from __future__ import annotations

import hashlib
import json
import math
import os
import socket
import ssl
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from gogoguard_contracts import utc_now


MAX_HTTP_BYTES = 1024 * 1024
MAX_COMMANDS_PER_HEARTBEAT = 32
ACTIVE_PATROL_STATES = {
    "PATROLLING",
    "RECOVERING",
    "SEARCHING_PATH",
    "BLOCKED",
    "PAUSED",
    "CHECKPOINT_PAUSING",
    "CHECKPOINT_SETTLING",
    "INSPECTING",
}
INTERACTION_STATUS_FIELDS = {
    "schema",
    "desiredLive",
    "state",
    "room",
    "participantId",
    "videoPublished",
    "audioPublished",
    "audioSubscribed",
    "dataConnected",
    "microphoneMuted",
    "tokenExpiresAt",
    "degradedReason",
    "lastErrorCode",
    "observedAt",
}


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def _atomic_json(path: Path, value: dict[str, Any], *, mode: int = 0o640) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _finite_or_none(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def command_result_url(heartbeat_url: str) -> str:
    endpoint = urlsplit(heartbeat_url)
    if not endpoint.path.endswith("/heartbeat"):
        raise ValueError("platform heartbeat URL cannot derive command result URL")
    return endpoint._replace(
        path=endpoint.path[: -len("heartbeat")] + "command/result"
    ).geturl()


class InteractionControlClient:
    """Send one command to the local interaction service; never persist payloads."""

    def __init__(self, socket_path: Path, *, timeout_s: float = 20.0) -> None:
        self.socket_path = Path(socket_path)
        self.timeout_s = float(timeout_s)

    def request(self, payload: dict[str, Any]) -> dict[str, Any]:
        encoded = (
            json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
            + "\n"
        ).encode("utf-8")
        if len(encoded) > 65536:
            raise ValueError("interaction command exceeds 64 KiB")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(self.timeout_s)
            client.connect(str(self.socket_path))
            client.sendall(encoded)
            reader = client.makefile("rb")
            raw = reader.readline(65537)
        if not raw or len(raw) > 65536:
            raise RuntimeError("interaction service returned an invalid response")
        value = json.loads(raw.decode("utf-8"))
        if not isinstance(value, dict) or value.get("ok") is not True:
            raise RuntimeError("interaction service rejected the platform command")
        return value


class NavigationControlClient:
    """Expose only selected-route start and full motion-owner stop to SaaS."""

    def __init__(self, socket_path: Path, *, timeout_s: float = 10.0) -> None:
        self.socket_path = Path(socket_path)
        self.timeout_s = float(timeout_s)

    def request(self, payload: dict[str, Any]) -> dict[str, Any]:
        action = payload.get("action", payload.get("type"))
        params = payload.get("params")
        if params is None:
            params = {}
        if not isinstance(params, dict):
            raise ValueError("platform patrol command params must be an object")
        if action == "start_patrol":
            method = "patrol.start_selected"
            rpc_params = {
                "expected_map_version": params.get("mapVersion"),
                "expected_route_id": params.get("routeId"),
                "mission_plan": params.get("missionPlan"),
            }
        elif action == "stop_patrol":
            method = "patrol.stop"
            rpc_params = {}
        else:
            raise ValueError("platform command is outside the patrol allow-list")
        encoded = (
            json.dumps(
                {"method": method, "params": rpc_params},
                ensure_ascii=False,
                separators=(",", ":"),
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(self.timeout_s)
            client.connect(str(self.socket_path))
            client.sendall(encoded)
            reader = client.makefile("rb")
            raw = reader.readline(1024 * 1024 + 1)
        if not raw or len(raw) > 1024 * 1024:
            raise RuntimeError("navigation supervisor returned an invalid response")
        value = json.loads(raw.decode("utf-8"))
        if not isinstance(value, dict) or value.get("ok") is not True:
            message = value.get("error") if isinstance(value, dict) else None
            raise RuntimeError(str(message or "navigation supervisor rejected command"))
        result = value.get("result")
        if not isinstance(result, dict) or result.get("state") not in {
            "accepted",
            "running",
            "complete",
        }:
            raise RuntimeError("navigation supervisor did not accept operation")
        return value


class CommandLedger:
    """Persistent command-ID deduplication without storing tokens, URLs or rooms."""

    def __init__(self, path: Path, *, max_entries: int = 256) -> None:
        if max_entries <= 0:
            raise ValueError("command ledger capacity must be positive")
        self.path = Path(path)
        self.max_entries = int(max_entries)
        self._lock = threading.Lock()
        value = _read_json(self.path)
        entries = value.get("entries", [])
        self._entries = [item for item in entries if isinstance(item, dict)][-self.max_entries :]
        self._keys = {
            str(item.get("idHash"))
            for item in self._entries
            if isinstance(item.get("idHash"), str)
        }

    @staticmethod
    def key(command_id: int | str) -> str:
        if isinstance(command_id, bool) or not isinstance(command_id, (int, str)):
            raise ValueError("platform command id must be an integer or string")
        canonical = json.dumps(
            [type(command_id).__name__, command_id],
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()

    def contains(self, command_id: int | str) -> bool:
        return self.key(command_id) in self._keys

    def record(self, command_id: int | str, action: str) -> None:
        key = self.key(command_id)
        with self._lock:
            if key in self._keys:
                return
            self._entries.append(
                {"idHash": key, "action": action, "completedAt": utc_now()}
            )
            self._entries = self._entries[-self.max_entries :]
            self._keys = {str(item["idHash"]) for item in self._entries}
            _atomic_json(
                self.path,
                {
                    "schema": "gogoguard.platform_command_ledger.v1",
                    "entries": self._entries,
                },
                mode=0o600,
            )


class UrllibJsonPoster:
    def __init__(
        self,
        *,
        device_token: str = "",
        tls_insecure: bool = False,
        host_header: str = "",
        allow_insecure_http: bool = False,
    ) -> None:
        self.device_token = device_token.strip()
        self.tls_insecure = bool(tls_insecure)
        self.host_header = host_header.strip()
        self.allow_insecure_http = bool(allow_insecure_http)

    def __call__(
        self, url: str, payload: dict[str, Any], timeout_s: float
    ) -> dict[str, Any]:
        endpoint = urlsplit(url)
        allowed = {"https", "http"} if self.allow_insecure_http else {"https"}
        if (
            endpoint.scheme not in allowed
            or not endpoint.hostname
            or endpoint.username is not None
            or endpoint.password is not None
            or endpoint.fragment
        ):
            raise ValueError("platform heartbeat URL is invalid")
        body = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        headers = {"Content-Type": "application/json", "Accept": "application/json"}
        if self.device_token:
            headers["X-Device-Token"] = self.device_token
        if self.host_header:
            headers["Host"] = self.host_header
        request = Request(url, data=body, headers=headers, method="POST")
        context = ssl._create_unverified_context() if self.tls_insecure else None
        try:
            with urlopen(request, timeout=timeout_s, context=context) as response:
                raw = response.read(MAX_HTTP_BYTES + 1)
        except HTTPError as exc:
            raise RuntimeError(f"platform heartbeat returned HTTP {exc.code}") from exc
        except URLError as exc:
            raise ConnectionError("platform heartbeat connection failed") from exc
        if len(raw) > MAX_HTTP_BYTES:
            raise ValueError("platform heartbeat response exceeds 1 MiB")
        value = json.loads(raw.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("platform heartbeat response must be an object")
        return value


class PlatformHeartbeatService:
    """Thin platform adapter around status files and narrow Unix controls.

    It has no ROS imports. Platform commands are restricted to realtime
    interaction plus start/stop of the already selected patrol route; arbitrary
    velocity and navigation mutation are not exposed.
    """

    def __init__(
        self,
        *,
        robot_id: str,
        heartbeat_url: str,
        navigation_status_path: Path,
        battery_status_path: Path,
        interaction_status_path: Path,
        capabilities_path: Path,
        service_status_path: Path,
        ledger: CommandLedger,
        interaction_client: InteractionControlClient,
        navigation_client: NavigationControlClient,
        post_json: Callable[[str, dict[str, Any], float], dict[str, Any]],
        command_result_url: str = "",
        post_result: Callable[[str, dict[str, Any], float], dict[str, Any]] | None = None,
        interval_s: float = 5.0,
        timeout_s: float = 10.0,
    ) -> None:
        if not robot_id or len(robot_id) > 128:
            raise ValueError("robot id is invalid")
        if not 1.0 <= float(interval_s) <= 60.0:
            raise ValueError("heartbeat interval must be between 1 and 60 seconds")
        if not 1.0 <= float(timeout_s) <= 30.0:
            raise ValueError("heartbeat timeout must be between 1 and 30 seconds")
        self.robot_id = robot_id
        self.heartbeat_url = heartbeat_url
        self.navigation_status_path = Path(navigation_status_path)
        self.battery_status_path = Path(battery_status_path)
        self.interaction_status_path = Path(interaction_status_path)
        self.capabilities_path = Path(capabilities_path)
        self.service_status_path = Path(service_status_path)
        self.ledger = ledger
        self.interaction_client = interaction_client
        self.navigation_client = navigation_client
        self.post_json = post_json
        self.command_result_url = command_result_url
        self.post_result = post_result
        self.interval_s = float(interval_s)
        self.timeout_s = float(timeout_s)
        self._sequence = 0
        self._last_success_at: str | None = None
        self._last_error_code: str | None = None
        self._processed_commands = 0
        self._duplicate_commands = 0
        self._command_results_sent = 0
        self._command_result_failures = 0

    def build_payload(self) -> dict[str, Any]:
        navigation = _read_json(self.navigation_status_path)
        battery_status = _read_json(self.battery_status_path)
        interaction_edge = _read_json(self.interaction_status_path)
        capabilities = _read_json(self.capabilities_path)
        runtime = navigation.get("runtime")
        runtime = runtime if isinstance(runtime, dict) else {}
        pose = navigation.get("localization_pose")
        pose = pose if isinstance(pose, dict) else {}
        commands = navigation.get("commands")
        commands = commands if isinstance(commands, dict) else {}
        final_command = commands.get("final")
        final_command = final_command if isinstance(final_command, dict) else {}
        state = str(runtime.get("state") or "IDLE")
        patrol_running = state in ACTIVE_PATROL_STATES
        live = interaction_edge.get("live")
        live = live if isinstance(live, dict) else {}
        public_live = {
            key: value for key, value in live.items() if key in INTERACTION_STATUS_FIELDS
        }
        payload: dict[str, Any] = {
            "schema": "gogoguard.robot_heartbeat.v1",
            "robotId": self.robot_id,
            "time": utc_now(),
            "status": "patrolling" if patrol_running else "idle",
            "motion": {
                "frame": str(pose.get("frame") or "map"),
                "position": {
                    "x": _finite_or_none(pose.get("x")),
                    "y": _finite_or_none(pose.get("y")),
                    "z": _finite_or_none(pose.get("z")),
                },
                "yaw_rad": _finite_or_none(pose.get("yaw")),
                "twist": {
                    "linear": {
                        "x": _finite_or_none(final_command.get("vx")),
                        "y": _finite_or_none(final_command.get("vy")),
                    },
                    "angular": {"z": _finite_or_none(final_command.get("wz"))},
                },
            },
            "battery": self._battery_payload(battery_status),
            "patrol": {
                "running": patrol_running,
                "state": state,
                "reason": runtime.get("reason"),
                "mapVersion": runtime.get("mapVersion"),
                "routeId": runtime.get("routeId"),
                "routeProgressIndex": runtime.get("routeProgressIndex"),
                "routeProgressPercent": runtime.get("routeProgressPercent"),
                "checkpoint": runtime.get("checkpoint"),
            },
            "interaction": public_live,
        }
        if capabilities.get("schema") == "gogoguard.robot_capabilities.v1":
            payload["capabilities"] = capabilities
        return payload

    @staticmethod
    def _battery_payload(status: dict[str, Any]) -> dict[str, Any]:
        percent = _finite_or_none(status.get("percent", status.get("soc")))
        if percent is not None and not 0.0 <= percent <= 100.0:
            percent = None
        voltage = _finite_or_none(status.get("voltage"))
        if voltage is not None and voltage <= 0.0:
            voltage = None
        charging = status.get("charging")
        if not isinstance(charging, bool):
            charging = None
        observed_at = status.get("observedAt")
        if not isinstance(observed_at, str) or not observed_at:
            observed_at = None
        source = status.get("source")
        if not isinstance(source, str) or not source:
            source = "unitree_sdk"
        return {
            "percent": percent,
            "charging": charging,
            "voltage": voltage,
            "observedAt": observed_at,
            "source": source,
        }

    def poll_once(self) -> dict[str, Any]:
        self._sequence += 1
        try:
            response = self.post_json(
                self.heartbeat_url, self.build_payload(), self.timeout_s
            )
        except Exception as exc:
            self._last_error_code = type(exc).__name__
            self._write_status(online=False)
            return {
                "ok": False,
                "errorCode": type(exc).__name__,
                "message": "platform heartbeat failed",
            }
        self._last_success_at = utc_now()
        try:
            if response.get("robotId") not in {None, self.robot_id}:
                raise ValueError("platform heartbeat response targets another robot")
            commands = response.get("commands", [])
            if not isinstance(commands, list) or len(commands) > MAX_COMMANDS_PER_HEARTBEAT:
                raise ValueError("platform heartbeat commands are invalid")
            outcomes = []
            for command in commands:
                try:
                    outcome = self._handle_command(command)
                except Exception as exc:
                    outcome = {
                        "action": (
                            command.get("action", command.get("type"))
                            if isinstance(command, dict)
                            else None
                        ),
                        "ok": False,
                        "status": "failed",
                        "message": str(exc)[:512] or type(exc).__name__,
                    }
                outcomes.append(outcome)
                self._report_command_result(command, outcome)
            self._last_error_code = None
            self._write_status(online=True)
            return {
                "ok": True,
                "commands": outcomes,
                "registered": response.get("registered"),
            }
        except Exception as exc:
            self._last_error_code = type(exc).__name__
            self._write_status(online=True)
            return {
                "ok": False,
                "errorCode": type(exc).__name__,
                "message": "platform heartbeat command processing failed",
            }

    def _handle_command(self, command: Any) -> dict[str, Any]:
        if not isinstance(command, dict):
            raise ValueError("platform command must be an object")
        command_id = command.get("id")
        action = command.get("action", command.get("type"))
        interaction_actions = {"start_live", "stop_live", "wake_transcript"}
        patrol_actions = {"start_patrol", "stop_patrol"}
        if action not in interaction_actions | patrol_actions:
            raise ValueError("platform command is outside the robot allow-list")
        if self.ledger.contains(command_id):
            self._duplicate_commands += 1
            return {
                "idHash": self.ledger.key(command_id),
                "action": action,
                "duplicate": True,
                "ok": True,
                "status": "duplicate",
                "message": "command already processed",
            }
        if action in interaction_actions:
            response = self.interaction_client.request(command)
        else:
            response = self.navigation_client.request(command)
        self.ledger.record(command_id, action)
        self._processed_commands += 1
        result = response.get("result")
        result = result if isinstance(result, dict) else {}
        return {
            "idHash": self.ledger.key(command_id),
            "action": action,
            "duplicate": False,
            "ok": True,
            "status": str(result.get("state") or "accepted"),
            "message": "command accepted by robot service",
            "operationId": result.get("operationId"),
        }

    def _report_command_result(
        self, command: Any, outcome: dict[str, Any]
    ) -> None:
        if not self.command_result_url or self.post_result is None:
            return
        if not isinstance(command, dict):
            return
        command_id = command.get("id")
        if isinstance(command_id, bool) or not isinstance(command_id, (int, str)):
            return
        result = {
            "command_id": command_id,
            "ok": outcome.get("ok") is True,
            "msg": str(outcome.get("message") or outcome.get("status") or "")[:512],
            "status": str(outcome.get("status") or "failed")[:64],
        }
        if outcome.get("operationId"):
            result["operation_id"] = str(outcome["operationId"])[:128]
        try:
            self.post_result(
                self.command_result_url,
                {"robotId": self.robot_id, "time": utc_now(), "result": result},
                self.timeout_s,
            )
        except Exception:
            self._command_result_failures += 1
        else:
            self._command_results_sent += 1

    def run(self, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            started = time.monotonic()
            self.poll_once()
            remaining = max(0.0, self.interval_s - (time.monotonic() - started))
            stop_event.wait(remaining)

    def _write_status(self, *, online: bool) -> None:
        _atomic_json(
            self.service_status_path,
            {
                "schema": "gogoguard.platform_edge_status.v1",
                "robotId": self.robot_id,
                "online": bool(online),
                "heartbeatSequence": self._sequence,
                "lastSuccessAt": self._last_success_at,
                "lastErrorCode": self._last_error_code,
                "processedCommands": self._processed_commands,
                "duplicateCommands": self._duplicate_commands,
                "commandResultsSent": self._command_results_sent,
                "commandResultFailures": self._command_result_failures,
                "observedAt": utc_now(),
                "motionCommandsPermitted": False,
                "selectedPatrolLifecyclePermitted": True,
            },
        )
