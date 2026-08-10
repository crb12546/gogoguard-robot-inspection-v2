from __future__ import annotations

import base64
import json
import time
import unittest
from pathlib import Path

from gogoguard_contracts import (
    InteractionCapabilities,
    InteractionSessionState,
    MediaConnectionReceipt,
    json_ready,
)
from gogoguard_interaction import (
    InteractionManager,
    WakeConversationGate,
    WakePolicy,
    load_persona,
)


ROOT = Path(__file__).resolve().parents[1]


def livekit_token(
    *, identity: str = "robot:LLYJ0001", room: str = "patrol-test-001",
    expires_in: int = 3600, can_publish: bool = True,
    can_subscribe: bool = True, can_publish_data: bool = True,
) -> str:
    def encoded(value: dict) -> str:
        raw = json.dumps(value, separators=(",", ":")).encode()
        return base64.urlsafe_b64encode(raw).decode().rstrip("=")

    now = int(time.time())
    return ".".join([
        encoded({"alg": "HS256", "typ": "JWT"}),
        encoded({
            "sub": identity, "nbf": now - 1, "exp": now + expires_in,
            "video": {
                "room": room, "roomJoin": True, "canPublish": can_publish,
                "canSubscribe": can_subscribe, "canPublishData": can_publish_data,
            },
        }),
        "test-signature",
    ])


def start_payload(*, room: str = "patrol-test-001", token: str | None = None) -> dict:
    return {
        "id": 41,
        "type": "start_live",
        "params": {
            "url": "wss://livekit.example.invalid",
            "token": token or livekit_token(room=room),
            "room": room,
        },
    }


class FakeTransport:
    def __init__(self) -> None:
        self.requests = []
        self.disconnects = 0
        self.connect_error: Exception | None = None
        self.microphone_muted: list[bool] = []
        self.playback_stops = 0
        self.playback_resumes = 0
        self.refreshes = []

    def connect(self, request):
        self.requests.append(request)
        if self.connect_error:
            raise self.connect_error
        return MediaConnectionReceipt(
            participant_id="robot:LLYJ0001", video_published=True,
            audio_published=True, audio_subscribed=True, data_connected=True,
        )

    def disconnect(self) -> None:
        self.disconnects += 1

    def set_microphone_muted(self, muted: bool) -> None:
        self.microphone_muted.append(muted)

    def stop_playback(self) -> None:
        self.playback_stops += 1

    def resume_playback(self) -> None:
        self.playback_resumes += 1

    def refresh(self, request) -> None:
        self.refreshes.append(request)


class InteractionManagerTest(unittest.TestCase):
    def create_manager(self, *, echo_control: str = "half_duplex"):
        transport = FakeTransport()
        manager = InteractionManager(
            robot_id="LLYJ0001",
            capabilities=InteractionCapabilities(
                video_uplink=True, audio_uplink=True, audio_downlink=True,
                data_channel=True, echo_control=echo_control,
            ),
            transport=transport,
        )
        return manager, transport

    def test_start_stop_and_repeated_commands_are_idempotent(self) -> None:
        manager, transport = self.create_manager()
        self.assertEqual(manager.start(start_payload()).state, InteractionSessionState.LIVE)
        self.assertEqual(manager.start(start_payload()).state, InteractionSessionState.LIVE)
        self.assertEqual(len(transport.requests), 1)
        self.assertEqual(manager.stop().state, InteractionSessionState.IDLE)
        self.assertEqual(manager.stop().state, InteractionSessionState.IDLE)
        self.assertEqual(transport.disconnects, 1)

    def test_ephemeral_token_is_absent_from_repr_and_status(self) -> None:
        manager, transport = self.create_manager()
        token = livekit_token()
        status = manager.start(start_payload(token=token))
        self.assertNotIn(token, repr(transport.requests[0]))
        self.assertNotIn(token, repr(status))
        self.assertNotIn(token, str(json_ready(status)))
        self.assertNotIn(token, str(manager.platform_status()))

    def test_production_requires_tls_and_development_ws_is_explicit(self) -> None:
        manager, _ = self.create_manager()
        payload = start_payload()
        payload["params"]["url"] = "ws://39.96.37.187:7880"
        with self.assertRaisesRegex(ValueError, "wss"):
            manager.start(payload)
        developer_manager, _ = self.create_manager()
        developer_manager.allow_insecure_ws = True
        self.assertEqual(developer_manager.start(payload).state, InteractionSessionState.LIVE)

    def test_new_token_refreshes_in_memory_without_reconnecting(self) -> None:
        manager, transport = self.create_manager()
        manager.start(start_payload())
        refreshed = manager.start(start_payload(token=livekit_token(expires_in=7200)))
        self.assertEqual(len(transport.requests), 1)
        self.assertEqual(len(transport.refreshes), 1)
        self.assertEqual(refreshed.state, InteractionSessionState.LIVE)

    def test_rejects_bad_token_identity_room_expiry_and_permissions(self) -> None:
        manager, _ = self.create_manager()
        cases = [
            (livekit_token(identity="robot:other"), "identity"),
            (livekit_token(room="another-room"), "room grant"),
            (livekit_token(expires_in=-1), "expired"),
            (livekit_token(can_publish=False), "media publishing"),
            (livekit_token(can_subscribe=False), "audio subscription"),
            (livekit_token(can_publish_data=False), "data publishing"),
        ]
        for token, message in cases:
            with self.subTest(message=message):
                with self.assertRaisesRegex(ValueError, message):
                    manager.start(start_payload(token=token))

    def test_transport_error_redacts_secret(self) -> None:
        manager, transport = self.create_manager()
        token = livekit_token()
        transport.connect_error = RuntimeError("failure " + token)
        status = manager.start(start_payload(token=token))
        self.assertEqual(status.state, InteractionSessionState.FAILED)
        self.assertTrue(status.desired_live)
        self.assertNotIn(token, status.last_error or "")

    def test_half_duplex_and_ordered_interrupt(self) -> None:
        manager, transport = self.create_manager()
        manager.start(start_payload())
        manager.playback_started()
        first = manager.handle_playback_control({
            "schema": "gogoguard.playback_control.v1", "action": "stop",
            "reason": "user_interrupt", "seq": 42,
        })
        duplicate = manager.handle_playback_control({
            "schema": "gogoguard.playback_control.v1", "action": "stop",
            "reason": "user_interrupt", "seq": 42,
        })
        self.assertTrue(first["accepted"])
        self.assertTrue(duplicate["duplicate"])
        self.assertEqual(transport.playback_stops, 1)
        self.assertEqual(transport.microphone_muted, [True, False])


class WakeConversationGateTest(unittest.TestCase):
    def test_only_wake_phrase_opens_the_conversation_gate(self) -> None:
        gate = WakeConversationGate(WakePolicy(idle_timeout_s=30))
        ignored = gate.handle_transcript("今天天气不错", now=10)
        awakened = gate.handle_transcript("小九，小九！", now=11)
        self.assertEqual(ignored["action"], "ignore")
        self.assertEqual(awakened["action"], "wake")
        self.assertEqual(awakened["acknowledgement"], "我在")
        self.assertTrue(gate.can_forward_conversation())

    def test_activity_extends_timeout_and_explicit_end_sleeps(self) -> None:
        gate = WakeConversationGate(WakePolicy(idle_timeout_s=30))
        gate.handle_transcript("小玖小玖", now=10)
        gate.handle_transcript("你看到了什么", now=35)
        self.assertIsNone(gate.tick(now=64.9))
        self.assertEqual(gate.tick(now=65)["reason"], "idle_timeout")
        gate.handle_transcript("小玖小玖", now=80)
        self.assertEqual(gate.handle_transcript("不用了", now=81)["reason"], "explicit_end")
        self.assertFalse(gate.can_forward_conversation())


class PersonaTest(unittest.TestCase):
    def test_xiaojiu_identity_and_safety_are_versioned(self) -> None:
        persona = load_persona(ROOT / "config/robot/interaction-persona.json")
        prompt = persona.platform_prompt_fragment()
        self.assertEqual(persona.name, "小玖")
        self.assertIn("北京零零壹玖科技有限公司", prompt)
        self.assertIn("不得根据历史画面猜测", prompt)
        self.assertIn("没有底盘运动权限", prompt)


if __name__ == "__main__":
    unittest.main()
