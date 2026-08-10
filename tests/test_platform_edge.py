from __future__ import annotations

import base64
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path

from gogoguard_contracts import MediaConnectionReceipt
from gogoguard_interaction_edge.service import InteractionEdgeService, InteractionUnixServer
from gogoguard_platform_edge import (
    CommandLedger,
    InteractionControlClient,
    PlatformHeartbeatService,
    UrllibJsonPoster,
)


class FakeInteractionClient:
    def __init__(self) -> None:
        self.commands: list[dict] = []

    def request(self, payload: dict) -> dict:
        self.commands.append(payload)
        return {"ok": True, "result": {"accepted": True}}


class FakeMediaTransport:
    def __init__(self) -> None:
        self.disconnects = 0

    def set_playback_control_handler(self, handler) -> None:
        self.playback_control = handler

    def set_playback_state_handler(self, handler) -> None:
        self.playback_state = handler

    def set_health_handler(self, handler) -> None:
        self.health = handler

    def connect(self, request):
        return MediaConnectionReceipt(
            participant_id="robot:LLYJ0001",
            video_published=True,
            audio_published=True,
            audio_subscribed=True,
            data_connected=True,
        )

    def disconnect(self) -> None:
        self.disconnects += 1

    def set_microphone_muted(self, muted: bool) -> None:
        return

    def stop_playback(self) -> None:
        return

    def resume_playback(self) -> None:
        return

    def refresh(self, request) -> None:
        return

    def status(self) -> dict:
        return {"running": self.disconnects == 0, "speaker": {}}


def livekit_token() -> str:
    def encode(value: dict) -> str:
        return base64.urlsafe_b64encode(
            json.dumps(value, separators=(",", ":")).encode()
        ).decode().rstrip("=")

    now = int(time.time())
    return ".".join(
        [
            encode({"alg": "HS256", "typ": "JWT"}),
            encode(
                {
                    "sub": "robot:LLYJ0001",
                    "nbf": now - 1,
                    "exp": now + 3600,
                    "video": {
                        "room": "patrol-test-001",
                        "roomJoin": True,
                        "canPublish": True,
                        "canSubscribe": True,
                        "canPublishData": True,
                    },
                }
            ),
            "test-signature",
        ]
    )


class ResponseQueue:
    def __init__(self, *responses: dict) -> None:
        self.responses = list(responses)
        self.payloads: list[dict] = []

    def __call__(self, url: str, payload: dict, timeout_s: float) -> dict:
        self.payloads.append(payload)
        if not self.responses:
            raise ConnectionError("offline")
        return self.responses.pop(0)


class PlatformHeartbeatServiceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.navigation = self.root / "navigation.json"
        self.interaction = self.root / "interaction.json"
        self.capabilities = self.root / "capabilities.json"
        self.status = self.root / "platform-status.json"
        self.ledger_path = self.root / "command-ledger.json"
        self.navigation.write_text(
            json.dumps(
                {
                    "runtime": {
                        "state": "PATROLLING",
                        "reason": "FOLLOWING_ROUTE",
                        "mapVersion": "map-6855ba54ae11",
                        "routeId": "route-r7",
                        "routeProgressIndex": 12,
                        "routeProgressPercent": 25.0,
                    },
                    "localization_pose": {
                        "frame": "map",
                        "x": 12.4,
                        "y": -3.1,
                        "z": 0.31,
                        "yaw": 1.57,
                    },
                    "commands": {"final": {"vx": 0.4, "vy": 0.0, "wz": 0.1}},
                }
            ),
            encoding="utf-8",
        )
        self.interaction.write_text(
            json.dumps(
                {
                    "schema": "gogoguard.interaction_edge_status.v1",
                    "live": {
                        "schema": "gogoguard.interaction_session_status.v1",
                        "desiredLive": False,
                        "state": "idle",
                        "room": "",
                        "token": "must-not-leak",
                        "url": "must-not-leak",
                    },
                }
            ),
            encoding="utf-8",
        )
        self.capabilities.write_text(
            json.dumps(
                {
                    "schema": "gogoguard.robot_capabilities.v1",
                    "revision": 1,
                    "routeExecution": {"supported": True},
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self) -> None:
        self.temp.cleanup()

    def service(self, responses: ResponseQueue, interaction: FakeInteractionClient):
        return PlatformHeartbeatService(
            robot_id="LLYJ0001",
            heartbeat_url="https://gogoguard.cn/api/v1/robot/heartbeat",
            navigation_status_path=self.navigation,
            interaction_status_path=self.interaction,
            capabilities_path=self.capabilities,
            service_status_path=self.status,
            ledger=CommandLedger(self.ledger_path),
            interaction_client=interaction,
            post_json=responses,
        )

    def test_builds_platform_compatible_read_only_heartbeat(self) -> None:
        responses = ResponseQueue({"ok": True, "robotId": "LLYJ0001", "commands": []})
        service = self.service(responses, FakeInteractionClient())
        self.assertTrue(service.poll_once()["ok"])
        payload = responses.payloads[0]
        self.assertEqual(payload["robotId"], "LLYJ0001")
        self.assertEqual(payload["status"], "patrolling")
        self.assertEqual(payload["motion"]["position"]["x"], 12.4)
        self.assertEqual(payload["motion"]["twist"]["linear"]["x"], 0.4)
        self.assertEqual(payload["patrol"]["routeProgressIndex"], 12)
        self.assertEqual(payload["capabilities"]["revision"], 1)
        encoded = json.dumps(payload)
        self.assertNotIn("must-not-leak", encoded)
        self.assertNotIn('"token"', encoded)
        self.assertNotIn('"url"', encoded)

    def test_start_refresh_stop_and_duplicate_commands_are_deterministic(self) -> None:
        first = {
            "id": 282,
            "action": "start_live",
            "params": {
                "url": "wss://gogoguard.cn",
                "room": "patrol-test-001",
                "token": "secret-token-one",
            },
        }
        duplicate = dict(first)
        refresh = {
            "id": 283,
            "action": "start_live",
            "params": {
                "url": "wss://gogoguard.cn",
                "room": "patrol-test-001",
                "token": "secret-token-two",
            },
        }
        stop = {"id": 285, "type": "stop_live", "params": {"room": "patrol-test-001"}}
        responses = ResponseQueue(
            {"ok": True, "robotId": "LLYJ0001", "commands": [first]},
            {"ok": True, "robotId": "LLYJ0001", "commands": [duplicate]},
            {"ok": True, "robotId": "LLYJ0001", "commands": [refresh]},
            {"ok": True, "robotId": "LLYJ0001", "commands": [stop]},
        )
        interaction = FakeInteractionClient()
        service = self.service(responses, interaction)
        results = [service.poll_once() for _ in range(4)]
        self.assertTrue(all(item["ok"] for item in results))
        self.assertEqual([item.get("action", item.get("type")) for item in interaction.commands],
                         ["start_live", "start_live", "stop_live"])
        self.assertTrue(results[1]["commands"][0]["duplicate"])
        persisted = self.ledger_path.read_text(encoding="utf-8")
        status = self.status.read_text(encoding="utf-8")
        for secret in ("secret-token-one", "secret-token-two", "patrol-test-001"):
            self.assertNotIn(secret, persisted)
            self.assertNotIn(secret, status)

        restarted = CommandLedger(self.ledger_path)
        self.assertTrue(restarted.contains(282))
        self.assertTrue(restarted.contains(283))
        self.assertTrue(restarted.contains(285))

    def test_network_failure_is_visible_without_exposing_endpoint(self) -> None:
        responses = ResponseQueue()
        service = self.service(responses, FakeInteractionClient())
        result = service.poll_once()
        self.assertFalse(result["ok"])
        persisted = json.loads(self.status.read_text(encoding="utf-8"))
        self.assertFalse(persisted["online"])
        self.assertEqual(persisted["lastErrorCode"], "ConnectionError")
        self.assertNotIn("gogoguard.cn", self.status.read_text(encoding="utf-8"))
        self.assertFalse(persisted["motionCommandsPermitted"])

    def test_rejects_commands_for_another_robot_and_motion_actions(self) -> None:
        wrong_robot = ResponseQueue(
            {"ok": True, "robotId": "OTHER", "commands": []}
        )
        result = self.service(wrong_robot, FakeInteractionClient()).poll_once()
        self.assertFalse(result["ok"])

        motion = ResponseQueue(
            {"ok": True, "robotId": "LLYJ0001", "commands": [{"id": 1, "action": "move"}]}
        )
        interaction = FakeInteractionClient()
        result = self.service(motion, interaction).poll_once()
        self.assertFalse(result["ok"])
        self.assertEqual(interaction.commands, [])
        self.assertTrue(json.loads(self.status.read_text(encoding="utf-8"))["online"])

    def test_unix_client_uses_one_line_json_framing(self) -> None:
        socket_path = self.root / "control.sock"
        received: list[dict] = []
        ready = threading.Event()

        def server() -> None:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
                listener.bind(str(socket_path))
                listener.listen(1)
                ready.set()
                connection, _ = listener.accept()
                with connection:
                    raw = connection.makefile("rb").readline()
                    received.append(json.loads(raw.decode("utf-8")))
                    connection.sendall(b'{"ok":true,"result":{"state":"live"}}\n')

        thread = threading.Thread(target=server, daemon=True)
        thread.start()
        self.assertTrue(ready.wait(2))
        response = InteractionControlClient(socket_path).request(
            {"id": 9, "action": "stop_live"}
        )
        thread.join(timeout=2)
        self.assertTrue(response["ok"])
        self.assertEqual(received, [{"id": 9, "action": "stop_live"}])

    def test_real_http_adapter_posts_json_and_optional_device_token(self) -> None:
        requests: list[dict] = []

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self) -> None:
                length = int(self.headers["Content-Length"])
                requests.append(
                    {
                        "path": self.path,
                        "authorization": self.headers.get("Authorization"),
                        "body": json.loads(self.rfile.read(length)),
                    }
                )
                body = b'{"ok":true,"robotId":"LLYJ0001","commands":[]}'
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format: str, *args) -> None:
                return

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            poster = UrllibJsonPoster(
                device_token="device-token",
                allow_insecure_http=True,
            )
            response = poster(
                f"http://127.0.0.1:{server.server_port}/api/v1/robot/heartbeat",
                {"robotId": "LLYJ0001"},
                2.0,
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
        self.assertTrue(response["ok"])
        self.assertEqual(requests[0]["path"], "/api/v1/robot/heartbeat")
        self.assertEqual(requests[0]["authorization"], "Bearer device-token")
        self.assertEqual(requests[0]["body"], {"robotId": "LLYJ0001"})

    def test_complete_platform_to_unix_to_interaction_state_flow(self) -> None:
        repository = Path(__file__).resolve().parents[1]
        interaction_status = self.root / "interaction-runtime.json"
        socket_path = self.root / "interaction.sock"
        media = FakeMediaTransport()
        interaction_service = InteractionEdgeService(
            robot_id="LLYJ0001",
            hardware_config=repository / "config/robot/interaction-hardware.json",
            persona_config=repository / "config/robot/interaction-persona.json",
            unitree_aes_128_key="unused-by-fake",
            volume_executable=Path("/missing"),
            status_path=interaction_status,
            transport=media,
        )
        unix_server = InteractionUnixServer(socket_path, interaction_service)
        thread = threading.Thread(target=unix_server.serve_forever, daemon=True)
        thread.start()
        for _ in range(100):
            if socket_path.exists():
                break
            time.sleep(0.01)
        secret = livekit_token()
        responses = ResponseQueue(
            {
                "ok": True,
                "robotId": "LLYJ0001",
                "commands": [
                    {
                        "id": 282,
                        "action": "start_live",
                        "params": {
                            "url": "wss://gogoguard.cn",
                            "room": "patrol-test-001",
                            "token": secret,
                            "publishVideo": True,
                            "publishAudio": True,
                            "subscribeAudio": True,
                            "publishData": True,
                        },
                    }
                ],
            },
            {"ok": True, "robotId": "LLYJ0001", "commands": []},
            {
                "ok": True,
                "robotId": "LLYJ0001",
                "commands": [{"id": 285, "action": "stop_live"}],
            },
        )
        platform = PlatformHeartbeatService(
            robot_id="LLYJ0001",
            heartbeat_url="https://gogoguard.cn/api/v1/robot/heartbeat",
            navigation_status_path=self.navigation,
            interaction_status_path=interaction_status,
            capabilities_path=self.capabilities,
            service_status_path=self.status,
            ledger=CommandLedger(self.ledger_path),
            interaction_client=InteractionControlClient(socket_path),
            post_json=responses,
        )
        try:
            self.assertTrue(platform.poll_once()["ok"])
            self.assertEqual(interaction_service.manager.platform_status()["state"], "live")
            self.assertTrue(platform.poll_once()["ok"])
            self.assertEqual(responses.payloads[1]["interaction"]["state"], "live")
            self.assertTrue(platform.poll_once()["ok"])
            self.assertEqual(interaction_service.manager.platform_status()["state"], "idle")
        finally:
            unix_server.shutdown()
            thread.join(timeout=2)
        for path in (interaction_status, self.status, self.ledger_path):
            self.assertNotIn(secret, path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
