from __future__ import annotations

import base64
import json
import re
import threading
import time
from dataclasses import replace
from datetime import datetime, timezone
from typing import Any, Protocol
from urllib.parse import urlsplit

from gogoguard_contracts import (
    InteractionCapabilities,
    InteractionSessionState,
    InteractionSessionStatus,
    MediaConnectionReceipt,
    RealtimeMediaSessionRequest,
    utc_now,
)


SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")


class StartLiveCommand:
    """Validate a platform command without verifying or persisting its JWT secret."""

    @classmethod
    def from_platform_payload(
        cls, payload: dict[str, Any], *, robot_id: str, allow_insecure_ws: bool = False
    ) -> RealtimeMediaSessionRequest:
        if not isinstance(payload, dict):
            raise ValueError("live command must be an object")
        action = payload.get("action", payload.get("type"))
        if action != "start_live":
            raise ValueError("command action must be start_live")
        command_id = payload.get("id")
        if isinstance(command_id, bool) or not isinstance(command_id, (int, str)):
            raise ValueError("command id must be an integer or string")
        if isinstance(command_id, str) and not command_id.strip():
            raise ValueError("command id must not be empty")
        if not SAFE_NAME.fullmatch(robot_id):
            raise ValueError("robot id is invalid")

        params = payload.get("params")
        if not isinstance(params, dict):
            raise ValueError("start_live params must be an object")
        url, room, token = params.get("url"), params.get("room"), params.get("token")
        if not isinstance(url, str):
            raise ValueError("LiveKit URL is required")
        endpoint = urlsplit(url)
        allowed_schemes = {"wss", "ws"} if allow_insecure_ws else {"wss"}
        if (
            endpoint.scheme not in allowed_schemes
            or not endpoint.hostname
            or endpoint.username is not None
            or endpoint.password is not None
            or endpoint.query
            or endpoint.fragment
        ):
            raise ValueError("LiveKit URL must be a credential-free wss endpoint")
        if not isinstance(room, str) or not SAFE_NAME.fullmatch(room):
            raise ValueError("LiveKit room is invalid")
        if not isinstance(token, str) or not token or len(token) > 8192:
            raise ValueError("LiveKit token is missing or invalid")

        requested = {
            "publishVideo": _boolean_param(params, "publishVideo", True),
            "publishAudio": _boolean_param(params, "publishAudio", True),
            "subscribeAudio": _boolean_param(params, "subscribeAudio", True),
            "publishData": _boolean_param(params, "publishData", True),
        }
        claims = _livekit_claims(token)
        now = int(time.time())
        expires_at, not_before = claims.get("exp"), claims.get("nbf", 0)
        grant = claims.get("video")
        if isinstance(expires_at, bool) or not isinstance(expires_at, int):
            raise ValueError("LiveKit token expiration is missing or invalid")
        if expires_at <= now + 30:
            raise ValueError("LiveKit token is expired or too close to expiration")
        if isinstance(not_before, bool) or not isinstance(not_before, int):
            raise ValueError("LiveKit token not-before claim is invalid")
        if not_before > now + 30:
            raise ValueError("LiveKit token is not active yet")
        if claims.get("sub") != f"robot:{robot_id}":
            raise ValueError("LiveKit token identity does not match robot id")
        if not isinstance(grant, dict):
            raise ValueError("LiveKit token video grant is missing")
        if grant.get("room") != room or grant.get("roomJoin") is not True:
            raise ValueError("LiveKit token room grant does not match command")
        if (requested["publishVideo"] or requested["publishAudio"]) and grant.get("canPublish") is not True:
            raise ValueError("LiveKit token does not allow media publishing")
        if requested["subscribeAudio"] and grant.get("canSubscribe") is not True:
            raise ValueError("LiveKit token does not allow audio subscription")
        if requested["publishData"] and grant.get("canPublishData") is not True:
            raise ValueError("LiveKit token does not allow data publishing")

        return RealtimeMediaSessionRequest(
            command_id=command_id,
            robot_id=robot_id,
            url=url,
            room=room,
            token=token,
            token_expires_at=datetime.fromtimestamp(
                expires_at, tz=timezone.utc
            ).isoformat(timespec="seconds"),
            publish_video=requested["publishVideo"],
            publish_audio=requested["publishAudio"],
            subscribe_audio=requested["subscribeAudio"],
            publish_data=requested["publishData"],
        )


def _livekit_claims(token: str) -> dict[str, Any]:
    parts = token.split(".")
    if len(parts) != 3:
        raise ValueError("LiveKit token must be a JWT")
    try:
        encoded = parts[1] + "=" * (-len(parts[1]) % 4)
        claims = json.loads(base64.urlsafe_b64decode(encoded))
    except (ValueError, TypeError, json.JSONDecodeError) as exc:
        raise ValueError("LiveKit token claims are invalid") from exc
    if not isinstance(claims, dict):
        raise ValueError("LiveKit token claims must be an object")
    return claims


def _boolean_param(params: dict[str, Any], name: str, default: bool) -> bool:
    value = params.get(name, default)
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a boolean")
    return value


class MediaTransport(Protocol):
    def connect(self, request: RealtimeMediaSessionRequest) -> MediaConnectionReceipt: ...
    def disconnect(self) -> None: ...
    def set_microphone_muted(self, muted: bool) -> None: ...
    def stop_playback(self) -> None: ...
    def resume_playback(self) -> None: ...


class InteractionManager:
    """Own desired and actual live state without navigation or cloud-model access."""

    def __init__(self, *, robot_id: str, capabilities: InteractionCapabilities, transport: MediaTransport, allow_insecure_ws: bool = False) -> None:
        if not SAFE_NAME.fullmatch(robot_id):
            raise ValueError("robot id is invalid")
        if capabilities.echo_control not in {"half_duplex", "full_duplex_aec"}:
            raise ValueError("unsupported echo control mode")
        self.robot_id = robot_id
        self.capabilities = capabilities
        self.transport = transport
        self.allow_insecure_ws = allow_insecure_ws
        self._lock = threading.Lock()
        self._status = InteractionSessionStatus(robot_id=robot_id)
        self._last_playback_control_seq = -1

    def status(self) -> InteractionSessionStatus:
        with self._lock:
            return replace(self._status)

    def start(self, payload: dict[str, Any]) -> InteractionSessionStatus:
        request = StartLiveCommand.from_platform_payload(
            payload, robot_id=self.robot_id, allow_insecure_ws=self.allow_insecure_ws
        )
        self._validate_requested_capabilities(request)
        with self._lock:
            if self._status.desired_live:
                if self._status.room != request.room:
                    raise RuntimeError("another live room is already desired")
                if self._status.state in {InteractionSessionState.CONNECTING, InteractionSessionState.LIVE, InteractionSessionState.DEGRADED}:
                    refresh = getattr(self.transport, "refresh", None)
                    if refresh is not None:
                        refresh(request)
                    self._status.token_expires_at = request.token_expires_at
                    self._status.observed_at = utc_now()
                    return replace(self._status)
            self._status = InteractionSessionStatus(
                robot_id=self.robot_id,
                room=request.room,
                state=InteractionSessionState.CONNECTING,
                desired_live=True,
                token_expires_at=request.token_expires_at,
            )
        try:
            receipt = self.transport.connect(request)
            if not receipt.participant_id:
                raise RuntimeError("media transport returned no participant id")
        except Exception as exc:
            safe_code = getattr(exc, "safe_code", "MEDIA_CONNECT_FAILED")
            if (
                not isinstance(safe_code, str)
                or re.fullmatch(r"[A-Z][A-Z0-9_]{0,63}", safe_code) is None
            ):
                safe_code = "MEDIA_CONNECT_FAILED"
            with self._lock:
                self._status.state = InteractionSessionState.FAILED
                self._status.last_error_code = safe_code
                self._status.last_error = f"{type(exc).__name__}: media transport failed"
                self._status.observed_at = utc_now()
                return replace(self._status)
        with self._lock:
            self._status.state = InteractionSessionState.LIVE
            self._status.participant_id = receipt.participant_id
            self._status.video_published = receipt.video_published
            self._status.audio_published = receipt.audio_published
            self._status.audio_subscribed = receipt.audio_subscribed
            self._status.data_connected = receipt.data_connected
            self._status.observed_at = utc_now()
            return replace(self._status)

    def _validate_requested_capabilities(self, request: RealtimeMediaSessionRequest) -> None:
        requested = {
            "video uplink": (request.publish_video, self.capabilities.video_uplink),
            "audio uplink": (request.publish_audio, self.capabilities.audio_uplink),
            "audio downlink": (request.subscribe_audio, self.capabilities.audio_downlink),
            "data channel": (request.publish_data, self.capabilities.data_channel),
        }
        unsupported = [name for name, (enabled, available) in requested.items() if enabled and not available]
        if unsupported:
            raise RuntimeError("requested media capability is unavailable: " + ", ".join(unsupported))

    def stop(self) -> InteractionSessionStatus:
        with self._lock:
            if self._status.state == InteractionSessionState.IDLE:
                return replace(self._status)
            self._status.desired_live = False
            self._status.state = InteractionSessionState.STOPPING
            self._status.observed_at = utc_now()
        try:
            self.transport.disconnect()
        except Exception as exc:
            safe_code = getattr(exc, "safe_code", "MEDIA_DISCONNECT_FAILED")
            if (
                not isinstance(safe_code, str)
                or re.fullmatch(r"[A-Z][A-Z0-9_]{0,63}", safe_code) is None
            ):
                safe_code = "MEDIA_DISCONNECT_FAILED"
            with self._lock:
                self._status.state = InteractionSessionState.FAILED
                self._status.last_error_code = safe_code
                self._status.last_error = f"{type(exc).__name__}: media disconnect failed"
                self._status.observed_at = utc_now()
                return replace(self._status)
        with self._lock:
            self._status = InteractionSessionStatus(robot_id=self.robot_id)
            self._last_playback_control_seq = -1
            return replace(self._status)

    def platform_status(self) -> dict[str, Any]:
        status = self.status()
        return {
            "schema": status.schema,
            "desiredLive": status.desired_live,
            "state": status.state.value,
            "room": status.room,
            "participantId": status.participant_id,
            "videoPublished": status.video_published,
            "audioPublished": status.audio_published,
            "audioSubscribed": status.audio_subscribed,
            "dataConnected": status.data_connected,
            "microphoneMuted": status.microphone_muted,
            "tokenExpiresAt": status.token_expires_at,
            "degradedReason": status.degraded_reason,
            "lastErrorCode": status.last_error_code,
            "observedAt": status.observed_at,
        }

    def playback_started(self) -> InteractionSessionStatus:
        status = self.status()
        if status.state not in {InteractionSessionState.LIVE, InteractionSessionState.DEGRADED}:
            raise RuntimeError("playback requires a live session")
        if self.capabilities.echo_control == "half_duplex" and not status.microphone_muted:
            self.transport.set_microphone_muted(True)
            with self._lock:
                self._status.microphone_muted = True
                self._status.observed_at = utc_now()
        return self.status()

    def playback_finished(self) -> InteractionSessionStatus:
        status = self.status()
        if self.capabilities.echo_control == "half_duplex" and status.microphone_muted:
            self.transport.set_microphone_muted(False)
            with self._lock:
                self._status.microphone_muted = False
                self._status.observed_at = utc_now()
        return self.status()

    def handle_playback_control(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict) or payload.get("schema") != "gogoguard.playback_control.v1":
            raise ValueError("unsupported playback control schema")
        action, sequence = payload.get("action"), payload.get("seq")
        if action not in {"stop", "resume"}:
            raise ValueError("unsupported playback control action")
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            raise ValueError("playback control seq must be a non-negative integer")
        reason = payload.get("reason", "")
        if not isinstance(reason, str) or len(reason) > 128:
            raise ValueError("playback control reason is invalid")
        with self._lock:
            if self._status.state not in {InteractionSessionState.LIVE, InteractionSessionState.DEGRADED}:
                raise RuntimeError("playback control requires a live session")
            if sequence <= self._last_playback_control_seq:
                return {"accepted": False, "duplicate": True, "action": action, "seq": sequence}
            self._last_playback_control_seq = sequence
        if action == "stop":
            self.transport.stop_playback()
            self.playback_finished()
        else:
            self.transport.resume_playback()
        return {"accepted": True, "duplicate": False, "action": action, "seq": sequence}

    def mark_degraded(self, reason: str) -> InteractionSessionStatus:
        if not reason.strip():
            raise ValueError("degraded reason must not be empty")
        with self._lock:
            if self._status.state not in {InteractionSessionState.LIVE, InteractionSessionState.DEGRADED}:
                raise RuntimeError("only a live session can be degraded")
            self._status.state = InteractionSessionState.DEGRADED
            self._status.degraded_reason = reason.strip()
            self._status.observed_at = utc_now()
            return replace(self._status)

    def mark_recovered(self) -> InteractionSessionStatus:
        with self._lock:
            if self._status.state != InteractionSessionState.DEGRADED:
                raise RuntimeError("session is not degraded")
            self._status.state = InteractionSessionState.LIVE
            self._status.degraded_reason = None
            self._status.observed_at = utc_now()
            return replace(self._status)

    def microphone_uplink_allowed(self, *, playback_active: bool) -> bool:
        status = self.status()
        if status.state not in {InteractionSessionState.LIVE, InteractionSessionState.DEGRADED}:
            return False
        if playback_active and self.capabilities.echo_control == "half_duplex":
            return False
        return status.audio_published and not status.microphone_muted
