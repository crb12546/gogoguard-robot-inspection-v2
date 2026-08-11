from __future__ import annotations

import base64
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path

from gogoguard_contracts import MediaConnectionReceipt
from gogoguard_interaction_edge.client import request
from gogoguard_interaction_edge.service import InteractionEdgeService, InteractionUnixServer


ROOT = Path(__file__).resolve().parents[1]


def token() -> str:
    def encode(value):
        return base64.urlsafe_b64encode(
            json.dumps(value, separators=(",", ":")).encode()
        ).decode().rstrip("=")

    now = int(time.time())
    return ".".join([
        encode({"alg": "HS256", "typ": "JWT"}),
        encode({
            "sub": "robot:LLYJ0001", "nbf": now - 1, "exp": now + 3600,
            "video": {
                "room": "patrol-test-001", "roomJoin": True,
                "canPublish": True, "canSubscribe": True, "canPublishData": True,
            },
        }),
        "test",
    ])


class FakeTransport:
    def __init__(self) -> None:
        self.control = None
        self.wake_transcript = None
        self.playback = None
        self.disconnects = 0
        self.muted = []
        self.health = None

    def set_playback_control_handler(self, handler):
        self.control = handler

    def set_wake_transcript_handler(self, handler):
        self.wake_transcript = handler

    def set_playback_state_handler(self, handler):
        self.playback = handler

    def set_health_handler(self, handler):
        self.health = handler

    def connect(self, request):
        return MediaConnectionReceipt(
            participant_id="robot:LLYJ0001", video_published=True,
            audio_published=True, audio_subscribed=True, data_connected=True,
        )

    def disconnect(self):
        self.disconnects += 1

    def set_microphone_muted(self, muted):
        self.muted.append(muted)

    def stop_playback(self):
        pass

    def resume_playback(self):
        pass

    def refresh(self, request):
        pass

    def status(self):
        return {"running": self.disconnects == 0, "speaker": {"underflowFrames": 0}}


class InteractionEdgeServiceTest(unittest.TestCase):
    def create_service(self, root: Path):
        return InteractionEdgeService(
            robot_id="LLYJ0001",
            hardware_config=ROOT / "config/robot/interaction-hardware.json",
            persona_config=ROOT / "config/robot/interaction-persona.json",
            unitree_aes_128_key="unused-by-fake",
            volume_executable=Path("/missing"),
            status_path=root / "status.json",
            transport=FakeTransport(),
        )

    def test_status_wake_and_live_control_do_not_expose_token(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            service = self.create_service(root)
            secret = token()
            started = service.handle({
                "id": 1, "action": "start_live",
                "params": {
                    "url": "wss://livekit.example.invalid",
                    "room": "patrol-test-001", "token": secret,
                },
            })
            self.assertEqual(started["live"]["state"], "live")
            self.assertEqual(service.handle({"action": "wake_transcript", "text": "小九小九"})["action"], "wake")
            status_text = (root / "status.json").read_text(encoding="utf-8")
            self.assertNotIn(secret, status_text)
            self.assertIn("小玖", status_text)
            service.handle({"action": "stop_live"})

    def test_livekit_data_transcript_updates_the_same_wake_gate(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            service = self.create_service(Path(temporary))
            service.transport.wake_transcript(
                {"action": "wake_transcript", "text": "小玖小玖"}
            )
            self.assertEqual(service.status()["wake"]["state"], "awake")

    def test_unix_socket_is_a_narrow_json_control_boundary(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            service = self.create_service(root)
            server = InteractionUnixServer(root / "control.sock", service)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            for _ in range(100):
                if (root / "control.sock").exists():
                    break
                time.sleep(0.01)
            status = request(root / "control.sock", {"action": "status"})
            self.assertEqual(status["persona"]["name"], "小玖")
            self.assertFalse(status["motionCommandsPermitted"])
            server.shutdown()
            thread.join(timeout=2)
            self.assertFalse(thread.is_alive())


if __name__ == "__main__":
    unittest.main()
