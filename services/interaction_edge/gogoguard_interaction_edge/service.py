from __future__ import annotations

import json
import os
import socket
import threading
from pathlib import Path
from typing import Any

from gogoguard_contracts import InteractionCapabilities, json_ready
from gogoguard_device_io import (
    Go2VolumeController,
    LiveKitGo2Transport,
    load_interaction_hardware_profile,
)
from gogoguard_interaction import (
    InteractionManager,
    WakeConversationGate,
    WakePolicy,
    load_persona,
)


class InteractionEdgeService:
    """Narrow local control boundary used by the existing GoGoGuard heartbeat agent."""

    def __init__(
        self,
        *,
        robot_id: str,
        hardware_config: Path,
        persona_config: Path,
        unitree_aes_128_key: str,
        volume_executable: Path,
        status_path: Path,
        transport=None,
        allow_insecure_ws: bool = False,
    ) -> None:
        self.robot_id = robot_id
        self.persona = load_persona(persona_config)
        self.wake = WakeConversationGate(WakePolicy())
        profile = load_interaction_hardware_profile(hardware_config)
        self.transport = transport or LiveKitGo2Transport(
            profile=profile,
            unitree_aes_128_key=unitree_aes_128_key,
            volume_controller=Go2VolumeController(volume_executable, profile.go2_interface),
        )
        self.manager = InteractionManager(
            robot_id=robot_id,
            capabilities=InteractionCapabilities(
                video_uplink=True,
                audio_uplink=True,
                audio_downlink=True,
                data_channel=True,
                echo_control="half_duplex",
            ),
            transport=self.transport,
            allow_insecure_ws=allow_insecure_ws,
        )
        self.transport.set_playback_control_handler(self.manager.handle_playback_control)
        set_wake_transcript_handler = getattr(
            self.transport, "set_wake_transcript_handler", None
        )
        if set_wake_transcript_handler is not None:
            set_wake_transcript_handler(self._wake_transcript_received)
        self.transport.set_playback_state_handler(self._playback_state_changed)
        set_health_handler = getattr(self.transport, "set_health_handler", None)
        if set_health_handler is not None:
            set_health_handler(self._media_health_changed)
        self.status_path = status_path
        self._write_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._write_status()

    def _wake_transcript_received(self, payload: dict[str, Any]) -> None:
        with self._command_lock:
            self.wake.handle_transcript(str(payload.get("text", "")))
            self._write_status()

    def _playback_state_changed(self, active: bool) -> None:
        try:
            if active:
                self.manager.playback_started()
            else:
                self.manager.playback_finished()
        finally:
            self._write_status()

    def _media_health_changed(self, recovered: bool, reason: str | None) -> None:
        try:
            if recovered:
                self.manager.mark_recovered()
            else:
                self.manager.mark_degraded(reason or "media reconnect pending")
        except RuntimeError:
            pass
        finally:
            self._write_status()

    def handle(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._command_lock:
            return self._handle_locked(payload)

    def _handle_locked(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError("interaction request must be an object")
        action = payload.get("action", payload.get("type"))
        if action == "status":
            result = self.status()
        elif action == "start_live":
            result = {"live": json_ready(self.manager.start(payload)), "wake": json_ready(self.wake.status())}
        elif action == "stop_live":
            self.wake.sleep("media_stopped")
            result = {"live": json_ready(self.manager.stop()), "wake": json_ready(self.wake.status())}
        elif action == "wake_transcript":
            result = self.wake.handle_transcript(str(payload.get("text", "")))
        elif action == "wake_tick":
            transition = self.wake.tick()
            result = transition or {"action": "none", "state": self.wake.status().state.value}
            if transition is None:
                return result
        else:
            raise ValueError("unsupported interaction action")
        self._write_status()
        return result

    def status(self) -> dict[str, Any]:
        return {
            "schema": "gogoguard.interaction_edge_status.v1",
            "robotId": self.robot_id,
            "persona": {
                "id": "xiaojiu-inspection-v1",
                "name": self.persona.name,
                "developer": self.persona.developer,
            },
            "live": self.manager.platform_status(),
            "wake": json_ready(self.wake.status()),
            "media": self.transport.status(),
            "motionCommandsPermitted": False,
        }

    def close(self) -> None:
        self.manager.stop()
        self._write_status()

    def _write_status(self) -> None:
        status = self.status()
        with self._write_lock:
            self.status_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.status_path.with_suffix(".tmp")
            temporary.write_text(
                json.dumps(status, ensure_ascii=False, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            os.chmod(temporary, 0o640)
            temporary.replace(self.status_path)


class InteractionUnixServer:
    def __init__(self, path: Path, service: InteractionEdgeService) -> None:
        self.path = path
        self.service = service
        self._stop = threading.Event()
        self._socket: socket.socket | None = None

    def serve_forever(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.unlink(missing_ok=True)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._socket = server
        server.bind(str(self.path))
        os.chmod(self.path, 0o660)
        server.listen(8)
        server.settimeout(0.5)
        try:
            while not self._stop.is_set():
                try:
                    connection, _ = server.accept()
                except socket.timeout:
                    self.service.handle({"action": "wake_tick"})
                    continue
                threading.Thread(
                    target=self._handle_connection,
                    args=(connection,),
                    daemon=True,
                ).start()
        finally:
            server.close()
            self.path.unlink(missing_ok=True)
            self.service.close()

    def shutdown(self) -> None:
        self._stop.set()

    def _handle_connection(self, connection: socket.socket) -> None:
        with connection:
            try:
                raw = connection.makefile("rb").readline(65537)
                if not raw or len(raw) > 65536:
                    raise ValueError("interaction request exceeds 64 KiB")
                payload = json.loads(raw.decode("utf-8"))
                result = {"ok": True, "result": self.service.handle(payload)}
            except Exception as exc:
                result = {
                    "ok": False,
                    "errorCode": type(exc).__name__,
                    "message": "interaction command failed",
                }
            connection.sendall(
                (json.dumps(result, ensure_ascii=False, sort_keys=True) + "\n").encode("utf-8")
            )
