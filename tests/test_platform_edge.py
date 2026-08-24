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
from unittest import mock

from gogoguard_contracts import MediaConnectionReceipt
from gogoguard_interaction_edge.service import InteractionEdgeService, InteractionUnixServer
from gogoguard_platform_edge import (
    CommandLedger,
    InteractionControlClient,
    NavigationControlClient,
    PlatformHeartbeatService,
    UrllibJsonPoster,
    command_result_url,
)


class FakeInteractionClient:
    def __init__(self) -> None:
        self.commands: list[dict] = []

    def request(self, payload: dict) -> dict:
        self.commands.append(payload)
        return {"ok": True, "result": {"accepted": True}}


class FakeNavigationClient(FakeInteractionClient):
    pass


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
    def test_release_pipeline_injects_build_identity_and_keeps_v1_gate_off(self) -> None:
        repository = Path(__file__).resolve().parents[1]
        prepare = (
            repository / "deployment/workstation/prepare-robot-release"
        ).read_text(encoding="utf-8")
        run_edge = (repository / "deployment/robot/run-edge").read_text(
            encoding="utf-8"
        )
        edge_entrypoint = (
            repository / "deployment/container/edge-entrypoint"
        ).read_text(encoding="utf-8")
        validator = (
            repository / "deployment/robot/validate-runtime-env"
        ).read_text(encoding="utf-8")
        service = (repository / "deployment/robot/gogoguard-edge.service").read_text(
            encoding="utf-8"
        )
        installer = (repository / "deployment/robot/install-release").read_text(
            encoding="utf-8"
        )
        runtime_env = (
            repository / "deployment/robot/runtime.env.example"
        ).read_text(encoding="utf-8")
        for name in (
            "GOGOGUARD_SOFTWARE_VERSION",
            "GOGOGUARD_GIT_COMMIT",
            "GOGOGUARD_IMAGE_DIGEST",
            "GOGOGUARD_BUILD_ID",
        ):
            self.assertIn(name, prepare)
        self.assertIn("--env-file /etc/gogoguard/release.env", run_edge)
        self.assertIn("GOGOGUARD_CHECKPOINT_EVIDENCE_TXN_ENABLED=0", runtime_env)
        self.assertIn("GOGOGUARD_ROBOT_ID=LLYJ0001", runtime_env)
        self.assertIn("GOGOGUARD_PLATFORM_HEARTBEAT_CA_FILE=", runtime_env)
        self.assertIn("/etc/gogoguard/platform-ca.crt", run_edge)
        self.assertIn('--ca-file "$PLATFORM_HEARTBEAT_CA_FILE"', edge_entrypoint)
        self.assertIn("commissioned platform CA file is unavailable", validator)
        self.assertIn("validate-runtime-env /etc/gogoguard/runtime.env", service)
        self.assertIn("validate-runtime-env /etc/gogoguard/runtime.env", installer)
        self.assertIn("commission-device-token", installer)

    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.navigation = self.root / "navigation.json"
        self.interaction = self.root / "interaction.json"
        self.battery = self.root / "battery.json"
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
                        "runtimeInstanceId": "runtime-1234",
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
                    "wake": {
                        "schema": "gogoguard.conversation_wake_status.v1",
                        "state": "awake",
                        "wakePhrase": "小玖小玖",
                        "wakeSequence": 7,
                        "observedAt": "2026-08-11T12:00:00Z",
                        "privateDebug": "must-not-leak",
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
        self.battery.write_text(
            json.dumps(
                {
                    "schema": "gogoguard.battery_status.v1",
                    "percent": 87,
                    "charging": False,
                    "voltage": 32.14,
                    "observedAt": "2026-08-11T12:00:00Z",
                    "source": "unitree_sdk",
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self) -> None:
        self.temp.cleanup()

    def service(
        self,
        responses: ResponseQueue,
        interaction: FakeInteractionClient,
        navigation: FakeNavigationClient | None = None,
        *,
        post_result=None,
        **service_kwargs,
    ):
        return PlatformHeartbeatService(
            robot_id="LLYJ0001",
            heartbeat_url="https://gogoguard.cn/api/v1/robot/heartbeat",
            navigation_status_path=self.navigation,
            battery_status_path=self.battery,
            interaction_status_path=self.interaction,
            capabilities_path=self.capabilities,
            service_status_path=self.status,
            ledger=CommandLedger(self.ledger_path),
            interaction_client=interaction,
            navigation_client=navigation or FakeNavigationClient(),
            post_json=responses,
            command_result_url=(
                "https://gogoguard.cn/api/v1/robot/command/result"
                if post_result is not None
                else ""
            ),
            post_result=post_result,
            **service_kwargs,
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
        self.assertTrue(payload["patrol"]["navigationReady"])
        self.assertEqual(payload["battery"]["percent"], 87.0)
        self.assertEqual(payload["battery"]["voltage"], 32.14)
        self.assertEqual(payload["capabilities"]["revision"], 1)
        self.assertEqual(
            payload["capabilities"]["evidenceTransaction"],
            {
                "supported": False,
                "versions": [],
                "manifestSha256": "58230fa7782fb5c4921a5ba730eb359f06b56b57eff57494732caac341281eb0",
            },
        )
        self.assertEqual(payload["runtimeInstanceId"], "runtime-1234")
        self.assertEqual(payload["interaction"]["wake"]["wakeSequence"], 7)
        self.assertNotIn("privateDebug", payload["interaction"]["wake"])
        for field in (
            "bootId", "edgeInstanceId", "edgeStartedAt", "softwareVersion",
            "gitCommit", "imageDigest", "buildId",
        ):
            self.assertIn(field, payload)
        encoded = json.dumps(payload)
        self.assertNotIn("must-not-leak", encoded)
        self.assertNotIn('"token"', encoded)
        self.assertNotIn('"url"', encoded)

    def test_stale_navigation_generation_fails_closed_and_is_visible(self) -> None:
        navigation = json.loads(self.navigation.read_text(encoding="utf-8"))
        navigation["edge_generation_id"] = "previous-generation"
        self.navigation.write_text(json.dumps(navigation), encoding="utf-8")
        responses = ResponseQueue(
            {"ok": True, "robotId": "LLYJ0001", "commands": []}
        )
        service = self.service(
            responses,
            FakeInteractionClient(),
            navigation_generation_id="current-generation",
        )
        self.assertTrue(service.poll_once()["ok"])
        payload = responses.payloads[0]
        self.assertEqual(payload["status"], "idle")
        self.assertIsNone(payload["runtimeInstanceId"])
        self.assertFalse(payload["patrol"]["running"])
        self.assertFalse(payload["patrol"]["navigationReady"])
        self.assertEqual(
            payload["patrol"]["navigationReasonCode"],
            "NAVIGATION_SNAPSHOT_GENERATION_MISMATCH",
        )
        persisted = json.loads(self.status.read_text(encoding="utf-8"))
        self.assertFalse(persisted["navigationReady"])
        self.assertEqual(
            persisted["navigationReasonCode"],
            "NAVIGATION_SNAPSHOT_GENERATION_MISMATCH",
        )
        self.assertEqual(payload["edgeInstanceId"], "current-generation")

    def test_missing_navigation_generation_rejects_queued_start_with_receipt(self) -> None:
        result_calls = []

        def post_result(url, payload, timeout_s):
            result_calls.append((url, payload, timeout_s))
            return {"ok": True}

        responses = ResponseQueue(
            {
                "ok": True,
                "robotId": "LLYJ0001",
                "commands": [{"id": 902, "action": "start_patrol", "params": {}}],
            }
        )
        navigation = FakeNavigationClient()
        service = self.service(
            responses,
            FakeInteractionClient(),
            navigation,
            post_result=post_result,
            navigation_generation_id="current-generation",
        )
        outcome = service.poll_once()["commands"][0]
        self.assertFalse(outcome["ok"])
        self.assertEqual(outcome["status"], "NAVIGATION_NOT_READY")
        self.assertEqual(outcome["reasonCode"], "NAVIGATION_SNAPSHOT_MISSING")
        self.assertEqual(navigation.commands, [])
        self.assertEqual(
            result_calls[0][1]["result"]["reason_code"],
            "NAVIGATION_SNAPSHOT_MISSING",
        )
        self.assertFalse(result_calls[0][1]["result"]["ok"])

    def test_lost_runtime_is_not_ready_and_rejects_queued_start(self) -> None:
        navigation_status = json.loads(self.navigation.read_text(encoding="utf-8"))
        navigation_status["edge_generation_id"] = "current-generation"
        navigation_status["runtime"].update(
            {
                "state": "INTERRUPTED",
                "reason": "NAV_RUNTIME_LOST",
                "motionAuthorized": False,
                "runtimeStatusStale": True,
            }
        )
        self.navigation.write_text(
            json.dumps(navigation_status), encoding="utf-8"
        )
        result_calls = []

        def post_result(url, payload, timeout_s):
            result_calls.append((url, payload, timeout_s))
            return {"ok": True}

        responses = ResponseQueue(
            {
                "ok": True,
                "robotId": "LLYJ0001",
                "commands": [{"id": 903, "action": "start_patrol", "params": {}}],
            }
        )
        navigation = FakeNavigationClient()
        service = self.service(
            responses,
            FakeInteractionClient(),
            navigation,
            post_result=post_result,
            navigation_generation_id="current-generation",
        )

        outcome = service.poll_once()["commands"][0]
        payload = responses.payloads[0]
        self.assertFalse(payload["patrol"]["running"])
        self.assertFalse(payload["patrol"]["navigationReady"])
        self.assertEqual(payload["patrol"]["reason"], "NAV_RUNTIME_LOST")
        self.assertEqual(
            payload["patrol"]["navigationReasonCode"], "NAV_RUNTIME_LOST"
        )
        self.assertFalse(outcome["ok"])
        self.assertEqual(outcome["status"], "NAVIGATION_NOT_READY")
        self.assertEqual(outcome["reasonCode"], "NAV_RUNTIME_LOST")
        self.assertEqual(navigation.commands, [])
        self.assertEqual(
            result_calls[0][1]["result"]["reason_code"], "NAV_RUNTIME_LOST"
        )
        persisted = json.loads(self.status.read_text(encoding="utf-8"))
        self.assertFalse(persisted["navigationReady"])
        self.assertEqual(persisted["navigationReasonCode"], "NAV_RUNTIME_LOST")

    def test_evidence_transaction_capability_requires_explicit_gate(self) -> None:
        payload = self.service(
            ResponseQueue(),
            FakeInteractionClient(),
            evidence_transaction_enabled=True,
        ).build_payload()
        self.assertEqual(
            payload["capabilities"]["evidenceTransaction"],
            {
                "supported": True,
                "versions": [1],
                "manifestSha256": "58230fa7782fb5c4921a5ba730eb359f06b56b57eff57494732caac341281eb0",
            },
        )

    def test_mission_command_ack_distinguishes_queued_from_applied(self) -> None:
        responses = ResponseQueue(
            {
                "ok": True,
                "robotId": "LLYJ0001",
                "commands": [
                    {
                        "id": 901,
                        "action": "announcement_completed",
                        "payload": {
                            "schema": "gogoguard.announcement_completed.v1",
                            "announcementId": "ann_901",
                            "missionId": "mission-1",
                            "checkpointId": "cp_01",
                            "attempt": 1,
                            "status": "completed",
                        },
                    }
                ],
            }
        )
        service = self.service(
            responses,
            FakeInteractionClient(),
            mission_inbox_path=self.root / "checkpoint-inbox.jsonl",
        )
        outcome = service.poll_once()["commands"][0]
        self.assertEqual(outcome["status"], "queued")
        self.assertEqual(outcome["message"], "mission message validated and queued")
        queued = json.loads(service.mission_inbox_path.read_text().strip())
        self.assertEqual(queued["announcementId"], "ann_901")

    def test_four_layer_identity_is_stable_across_edge_restart_boundaries(self) -> None:
        boot_id_path = self.root / "boot_id"
        boot_id_path.write_text("11111111-2222-4333-8444-555555555555\n", encoding="utf-8")
        build_identity = {
            "softwareVersion": "2.4.0",
            "gitCommit": "abc1234",
            "imageDigest": "sha256:" + "a" * 64,
            "buildId": "field-20260823",
        }
        responses = ResponseQueue()
        first = self.service(
            responses,
            FakeInteractionClient(),
            boot_id_path=boot_id_path,
            build_identity=build_identity,
        ).build_payload()
        second = self.service(
            responses,
            FakeInteractionClient(),
            boot_id_path=boot_id_path,
            build_identity=build_identity,
        ).build_payload()
        self.assertEqual(first["bootId"], second["bootId"])
        self.assertNotEqual(first["edgeInstanceId"], second["edgeInstanceId"])
        self.assertEqual(first["runtimeInstanceId"], "runtime-1234")
        for field, value in build_identity.items():
            self.assertEqual(first[field], value)

    def test_unknown_pose_and_battery_values_are_null_not_fake_zero(self) -> None:
        self.navigation.write_text(
            json.dumps({"runtime": {"state": "IDLE"}, "localization_pose": None}),
            encoding="utf-8",
        )
        self.battery.unlink()
        responses = ResponseQueue({"ok": True, "robotId": "LLYJ0001", "commands": []})
        service = self.service(responses, FakeInteractionClient())
        self.assertTrue(service.poll_once()["ok"])
        payload = responses.payloads[0]
        self.assertIsNone(payload["motion"]["position"]["x"])
        self.assertIsNone(payload["motion"]["yaw_rad"])
        self.assertIsNone(payload["motion"]["twist"]["linear"]["x"])
        self.assertIsNone(payload["battery"]["percent"])
        self.assertEqual(payload["battery"]["source"], "unitree_sdk")

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
        self.assertTrue(result["ok"])
        self.assertFalse(result["commands"][0]["ok"])
        self.assertEqual(interaction.commands, [])
        self.assertTrue(json.loads(self.status.read_text(encoding="utf-8"))["online"])

    def test_command_results_report_acceptance_and_failure_without_payload_secrets(self) -> None:
        commands = [
            {"id": 40, "action": "start_patrol", "params": {}},
            {"id": 41, "action": "move", "params": {"token": "never-log-me"}},
        ]
        responses = ResponseQueue(
            {"ok": True, "robotId": "LLYJ0001", "commands": commands}
        )
        result_calls = []

        def post_result(url, payload, timeout_s):
            result_calls.append((url, payload, timeout_s))
            return {"ok": True}

        service = self.service(
            responses,
            FakeInteractionClient(),
            post_result=post_result,
        )
        outcome = service.poll_once()
        self.assertTrue(outcome["ok"])
        self.assertTrue(outcome["commands"][0]["ok"])
        self.assertFalse(outcome["commands"][1]["ok"])
        self.assertEqual(len(result_calls), 2)
        self.assertEqual(result_calls[0][1]["result"]["command_id"], 40)
        self.assertTrue(result_calls[0][1]["result"]["ok"])
        self.assertFalse(result_calls[1][1]["result"]["ok"])
        self.assertNotIn(
            "never-log-me", json.dumps(result_calls, ensure_ascii=False)
        )
        persisted = json.loads(self.status.read_text(encoding="utf-8"))
        self.assertEqual(persisted["commandResultsSent"], 2)

    def test_command_result_endpoint_is_derived_from_frozen_heartbeat_path(self) -> None:
        self.assertEqual(
            command_result_url(
                "http://39.96.37.187/api/v1/robot/heartbeat?source=dog"
            ),
            "http://39.96.37.187/api/v1/robot/command/result?source=dog",
        )

    def test_platform_can_only_start_or_stop_the_selected_patrol(self) -> None:
        start = {
            "id": "patrol-1",
            "action": "start_patrol",
            "params": {
                "mapVersion": "map-v1",
                "routeId": "route-v2",
                "missionPlan": {
                    "missionId": "mission-1",
                    "checkpoints": [
                        {"checkpointId": "point-1", "routeProgressIndex": 42}
                    ],
                },
            },
        }
        stop = {"id": "patrol-2", "action": "stop_patrol", "params": {}}
        responses = ResponseQueue(
            {"ok": True, "robotId": "LLYJ0001", "commands": [start]},
            {"ok": True, "robotId": "LLYJ0001", "commands": [stop]},
        )
        interaction = FakeInteractionClient()
        navigation = FakeNavigationClient()
        service = self.service(responses, interaction, navigation)
        self.assertTrue(service.poll_once()["ok"])
        self.assertTrue(service.poll_once()["ok"])
        self.assertEqual(interaction.commands, [])
        self.assertEqual(navigation.commands, [start, stop])

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

    def test_navigation_client_maps_platform_lifecycle_to_narrow_rpc(self) -> None:
        socket_path = self.root / "navigation.sock"
        received: list[dict] = []
        ready = threading.Event()

        def server() -> None:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
                listener.bind(str(socket_path))
                listener.listen(1)
                ready.set()
                connection, _ = listener.accept()
                with connection:
                    received.append(
                        json.loads(connection.makefile("rb").readline().decode("utf-8"))
                    )
                    connection.sendall(
                        b'{"ok":true,"result":{"state":"accepted","operationId":"op-1"}}\n'
                    )

        thread = threading.Thread(target=server, daemon=True)
        thread.start()
        self.assertTrue(ready.wait(2))
        response = NavigationControlClient(socket_path).request(
            {
                "id": 10,
                "action": "start_patrol",
                "params": {"mapVersion": "map-v1", "routeId": "route-v2"},
            }
        )
        thread.join(timeout=2)
        self.assertTrue(response["ok"])
        self.assertEqual(
            received,
            [
                {
                    "method": "patrol.start_selected",
                    "params": {
                        "expected_map_version": "map-v1",
                        "expected_route_id": "route-v2",
                        "mission_plan": None,
                    },
                }
            ],
        )

    def test_real_http_adapter_posts_json_and_optional_device_token(self) -> None:
        requests: list[dict] = []

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self) -> None:
                length = int(self.headers["Content-Length"])
                requests.append(
                    {
                        "path": self.path,
                        "authorization": self.headers.get("Authorization"),
                        "device_token": self.headers.get("X-Device-Token"),
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
        self.assertIsNone(requests[0]["authorization"])
        self.assertEqual(requests[0]["device_token"], "device-token")
        self.assertEqual(requests[0]["body"], {"robotId": "LLYJ0001"})

    def test_https_adapter_uses_only_the_commissioned_ca_file(self) -> None:
        class Response:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            @staticmethod
            def read(*_args):
                return b'{"ok":true}'

        ca_file = self.root / "platform-ca.crt"
        ca_file.write_text("test CA placeholder", encoding="utf-8")
        context = object()
        with mock.patch(
            "gogoguard_platform_edge.heartbeat.ssl.create_default_context",
            return_value=context,
        ) as create_context, mock.patch(
            "gogoguard_platform_edge.heartbeat.urlopen",
            return_value=Response(),
        ) as opened:
            result = UrllibJsonPoster(ca_file=ca_file)(
                "https://39.96.37.187/api/v1/robot/heartbeat",
                {"robotId": "LLYJ0001"},
                2.0,
            )

        self.assertTrue(result["ok"])
        create_context.assert_called_once_with(cafile=str(ca_file))
        self.assertIs(opened.call_args.kwargs["context"], context)

    def test_private_ca_cannot_disable_tls_verification(self) -> None:
        with self.assertRaisesRegex(ValueError, "cannot be combined"):
            UrllibJsonPoster(ca_file=self.root / "ca.crt", tls_insecure=True)

    def test_real_http_adapter_preserves_safe_http_status_for_backoff(self) -> None:
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self) -> None:
                self.send_response(401)
                self.send_header("Content-Length", "0")
                self.end_headers()

            def log_message(self, format: str, *args) -> None:
                return

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            poster = UrllibJsonPoster(allow_insecure_http=True)
            with self.assertRaisesRegex(RuntimeError, "HTTP 401") as caught:
                poster(
                    f"http://127.0.0.1:{server.server_port}/heartbeat",
                    {"robotId": "LLYJ0001"},
                    2.0,
                )
            self.assertEqual(getattr(caught.exception, "code", None), 401)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

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
            battery_status_path=self.battery,
            interaction_status_path=interaction_status,
            capabilities_path=self.capabilities,
            service_status_path=self.status,
            ledger=CommandLedger(self.ledger_path),
            interaction_client=InteractionControlClient(socket_path),
            navigation_client=FakeNavigationClient(),
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
