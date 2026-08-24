from __future__ import annotations

import base64
import hashlib
import json
import socket
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
        self.mission_message = None
        self.playback = None
        self.disconnects = 0
        self.muted = []
        self.health = None
        self.published_data = []

    def set_playback_control_handler(self, handler):
        self.control = handler

    def set_wake_transcript_handler(self, handler):
        self.wake_transcript = handler

    def set_mission_message_handler(self, handler):
        self.mission_message = handler

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

    def publish_data(self, payload, *, topic, reliable):
        self.published_data.append((payload, topic, reliable))
        return True

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
            mission_inbox_path=root / "checkpoint-inbox.jsonl",
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
            result = service.transport.wake_transcript(
                {
                    "action": "wake_transcript",
                    "eventId": "wake-event-1",
                    "text": "今天不需要唤醒",
                }
            )
            self.assertEqual(service.status()["wake"]["state"], "sleeping")
            self.assertEqual(result["schema"], "gogoguard.wake_gate_result.v1")
            self.assertEqual(result["eventId"], "wake-event-1")
            self.assertEqual(result["action"], "ignore")
            self.assertEqual(result["reason"], "wake_phrase_not_matched")
            self.assertEqual(
                result["transcriptHash"],
                hashlib.sha256("今天不需要唤醒".encode("utf-8")).hexdigest(),
            )
            self.assertNotIn("今天不需要唤醒", json.dumps(result, ensure_ascii=False))
            self.assertEqual(
                service.transport.published_data,
                [(result, "gogoguard.wake_gate_result.v1", True)],
            )

    def test_map_bound_pose_is_forwarded_unreliably_without_motion_access(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            service = self.create_service(Path(temporary))
            pose = {
                "schema": "gogoguard.robot_pose.v1",
                "robotId": "LLYJ0001",
                "sequence": 4,
                "mapVersion": "map-6855ba54ae11",
                "routeId": "route-6855ba54ae11-workspace-r4",
                "frameId": "map",
                "pose": {
                    "position": {"x": 1.2, "y": -0.4, "z": 0.1},
                    "yawRad": 0.3,
                },
                "localization": {"usable": True, "confidence": 0.8},
                "mission": {
                    "missionId": "mission-20260811-RG-狗02",
                    "checkpointId": "cp_01",
                    "phase": "spinning",
                    "spinProgressRad": 3.14,
                    "camera": {"pan": -14.8, "tilt": 22.4},
                },
                "sourceAt": "2026-08-11T08:00:00.000+00:00",
                "observedAt": "2026-08-11T08:00:00+00:00",
            }
            result = service.handle({"action": "publish_pose", "payload": pose})
            self.assertTrue(result["accepted"])
            self.assertEqual(
                service.transport.published_data,
                [(pose, "gogoguard.robot_pose.v1", False)],
            )
            invalid = dict(pose, robotId="OTHER")
            with self.assertRaisesRegex(ValueError, "identity"):
                service.handle({"action": "publish_pose", "payload": invalid})
            invalid_mission = dict(
                pose,
                mission={**pose["mission"], "camera": {"pan": float("nan"), "tilt": 0}},
            )
            with self.assertRaisesRegex(ValueError, "mission camera"):
                service.handle({"action": "publish_pose", "payload": invalid_mission})

    def test_reliable_checkpoint_verdict_is_validated_and_persisted(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            service = self.create_service(root)
            verdict = {
                "schema": "gogoguard.checkpoint_verdict.v1",
                "verdictId": "vd_1",
                "missionId": "mission-20260811-RG-狗02",
                "mapVersion": "map-6855ba54ae11",
                "routeId": "route-6855ba54ae11-workspace-r4",
                "checkpointId": "cp_01",
                "attempt": 1,
                "action": "continue",
                "expiresAt": "2099-01-01T00:00:00+00:00",
            }
            self.assertTrue(service.transport.mission_message(verdict)["accepted"])
            saved = json.loads(
                (root / "checkpoint-inbox.jsonl").read_text(encoding="utf-8")
            )
            self.assertEqual(saved["verdictId"], "vd_1")
            with self.assertRaisesRegex(ValueError, "action"):
                service.transport.mission_message({**verdict, "action": "move"})

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

    def test_unix_socket_reports_safe_wss_reason_without_echoing_secret(self) -> None:
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
            secret = token()
            payload = {
                "id": 2,
                "action": "start_live",
                "params": {
                    "url": "ws://39.96.37.187:7880",
                    "room": "patrol-test-001",
                    "token": secret,
                },
            }
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.connect(str(root / "control.sock"))
                client.sendall((json.dumps(payload) + "\n").encode("utf-8"))
                response = json.loads(client.makefile("rb").readline())
            self.assertFalse(response["ok"])
            self.assertEqual(response["reasonCode"], "LIVEKIT_WSS_REQUIRED")
            self.assertNotIn(secret, json.dumps(response))
            server.shutdown()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
