from __future__ import annotations

import json
import hashlib
import os
import tempfile
import time
import unittest
import zipfile
from pathlib import Path
from unittest.mock import Mock, patch

from gogoguard_field_workstation.platform_assets import PlatformAssetUploader
from gogoguard_platform_edge.checkpoint import CheckpointCoordinator, mission_url
from gogoguard_contracts import contract_ref, utc_now


class FakeGimbal:
    def __init__(self) -> None:
        self.moves = []

    def move_for_inspection(self, **angles):
        self.moves.append(angles)
        return {
            "angles": {
                "pan": angles["pan_body_deg"] + 0.2,
                "tilt": angles["tilt_euler_deg"] - 0.1,
                "roll": angles["roll_euler_deg"],
            }
        }


class MisalignedGimbal(FakeGimbal):
    def move_for_inspection(self, **angles):
        self.moves.append(angles)
        return {
            "angles": {
                "pan": angles["pan_body_deg"] + 12.0,
                "tilt": angles["tilt_euler_deg"],
                "roll": angles["roll_euler_deg"],
            }
        }


class DeviceTokenError(RuntimeError):
    code = 401


class CheckpointPlatformTest(unittest.TestCase):
    @staticmethod
    def capture_receipt(capture_request_id, **changes):
        value = {
            "captureRequestId": capture_request_id,
            "captureId": "capture-platform-1",
            "frameId": "frame-41",
            "ingressAcceptedAt": "2026-08-23T05:30:00+00:00",
            "frameReceivedAt": "2026-08-23T05:30:01+00:00",
            "streamInstanceId": "stream-boot-1",
            "frameSequence": 41,
            "participant": "robot:LLYJ0001",
            "track": "camera",
            "width": 1280,
            "height": 720,
            "mime": "image/jpeg",
            "bytes": 8192,
            "sha256": "a" * 64,
            "missionId": "mission-1",
            "checkpointId": "cp_01",
            "attempt": 1,
            "mapVersion": "map-v1",
            "routeId": "route-v1",
            "evidenceOrigin": "evidence_ingress",
            "evidenceScope": "checkpoint_verdict",
        }
        value.update(changes)
        return value

    def test_v1_captured_event_requires_real_platform_receipt(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            control = root / "control.json"
            calls = []
            runtime = {
                "mapVersion": "map-v1",
                "routeId": "route-v1",
                "routeProgressIndex": 12,
                "checkpoint": {
                    "missionId": "mission-1",
                    "activeCheckpointId": "cp_01",
                    "activeRouteProgressIndex": 12,
                    "attempt": 1,
                    "phase": "WAITING_PLATFORM",
                    "evidenceTransactionVersion": 1,
                    "dwellSec": 1,
                    "verdictTimeoutSec": 15,
                },
            }
            navigation.write_text(json.dumps({"runtime": runtime}), encoding="utf-8")

            def poster(url, payload, timeout):
                calls.append(dict(payload))
                if payload.get("phase") == "announcing":
                    return {"announcement": {"proceed": True}}
                if payload.get("phase") == "capture_ready":
                    request_id = payload["captureRequestId"]
                    return {
                        "captureResponse": {
                            "accepted": True,
                            "captureRequestId": request_id,
                            "contract": contract_ref(),
                            "state": "succeeded",
                            "receipt": self.capture_receipt(request_id),
                        }
                    }
                return {"ok": True}

            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=control,
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=poster,
                evidence_transaction_enabled=True,
                robot_id="LLYJ0001",
            )
            coordinator.tick(now=100.0)
            coordinator.tick(now=101.1)
            state = json.loads((root / "state.json").read_text())
            self.assertEqual(state["stage"], "waiting_verdict")
            self.assertEqual(state["captureId"], "capture-platform-1")
            self.assertEqual(json.loads(control.read_text())["action"], "capture")
            coordinator.tick(now=101.2)
            captured = [item for item in calls if item.get("phase") == "captured"]
            self.assertEqual(captured[0]["captureId"], "capture-platform-1")
            self.assertNotIn("evidenceId", captured[0])
            self.assertTrue(calls)
            self.assertTrue(all(item.get("robotId") == "LLYJ0001" for item in calls))

    def test_v1_pending_capture_reconciles_with_same_request_and_event_id(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            runtime = {
                "mapVersion": "map-v1",
                "routeId": "route-v1",
                "checkpoint": {
                    "missionId": "mission-1",
                    "activeCheckpointId": "cp_01",
                    "activeRouteProgressIndex": 12,
                    "attempt": 1,
                    "phase": "WAITING_PLATFORM",
                    "evidenceTransactionVersion": 1,
                    "dwellSec": 1,
                },
            }
            navigation.write_text(json.dumps({"runtime": runtime}), encoding="utf-8")
            captures = []

            def poster(url, payload, timeout):
                if payload.get("phase") == "announcing":
                    return {"announcement": {"proceed": True}}
                if payload.get("phase") == "capture_ready":
                    captures.append(dict(payload))
                    response = {
                        "accepted": True,
                        "captureRequestId": payload["captureRequestId"],
                        "contract": contract_ref(),
                    }
                    if len(captures) == 1:
                        return {
                            "captureResponse": {
                                **response,
                                "state": "selecting",
                                "retryAfterMs": 200,
                            }
                        }
                    return {
                        "captureResponse": {
                            **response,
                            "state": "succeeded",
                            "receipt": self.capture_receipt(payload["captureRequestId"]),
                        }
                    }
                return {"ok": True}

            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=root / "control.json",
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=poster,
                evidence_transaction_enabled=True,
            )
            coordinator.tick(now=100.0)
            coordinator.tick(now=101.1)
            self.assertFalse((root / "control.json").exists())
            runtime["checkpoint"]["phase"] = "WAITING_VERDICT"
            navigation.write_text(json.dumps({"runtime": runtime}), encoding="utf-8")
            coordinator.tick(now=101.31)
            self.assertEqual(len(captures), 2)
            self.assertEqual(captures[0]["captureRequestId"], captures[1]["captureRequestId"])
            self.assertEqual(captures[0]["eventId"], captures[1]["eventId"])
            self.assertEqual(coordinator.context["stage"], "waiting_verdict")

    def test_v1_invalid_receipt_fails_closed_and_holds_position(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            runtime = {
                "mapVersion": "map-v1",
                "routeId": "route-v1",
                "checkpoint": {
                    "missionId": "mission-1",
                    "activeCheckpointId": "cp_01",
                    "attempt": 1,
                    "phase": "WAITING_PLATFORM",
                    "evidenceTransactionVersion": 1,
                    "dwellSec": 1,
                },
            }
            navigation.write_text(json.dumps({"runtime": runtime}), encoding="utf-8")

            def poster(url, payload, timeout):
                if payload.get("phase") == "announcing":
                    return {"announcement": {"proceed": True}}
                if payload.get("phase") == "capture_ready":
                    request_id = payload["captureRequestId"]
                    return {
                        "captureResponse": {
                            "accepted": True,
                            "captureRequestId": request_id,
                            "contract": contract_ref(),
                            "state": "succeeded",
                            "receipt": self.capture_receipt(
                                request_id, participant="robot:ANOTHER"
                            ),
                        }
                    }
                return {"ok": True}

            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=root / "control.json",
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=poster,
                evidence_transaction_enabled=True,
            )
            coordinator.tick(now=100.0)
            coordinator.tick(now=101.1)
            self.assertEqual(coordinator.context["stage"], "capture_failed")
            self.assertEqual(
                coordinator.context["captureFailureCode"],
                "RECEIPT_BINDING_MISMATCH",
            )
            failure_control = json.loads((root / "control.json").read_text())
            self.assertEqual(failure_control["action"], "capture_failed")
            self.assertFalse(
                any(
                    item.get("payload", {}).get("phase")
                    in {"captured", "waiting_verdict"}
                    for item in coordinator.context.get("pendingEvents", [])
                    if isinstance(item, dict)
                )
            )

    def test_frozen_mission_urls_are_derived_from_heartbeat(self) -> None:
        heartbeat = "http://39.96.37.187/api/v1/robot/heartbeat"
        self.assertEqual(
            mission_url(heartbeat, "event"),
            "http://39.96.37.187/api/v1/robot/mission/event",
        )

    def test_true_stop_to_capture_to_verdict_writes_navigation_controls(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            control = root / "control.json"
            inbox = root / "inbox.jsonl"
            calls = []

            def poster(url, payload, timeout):
                calls.append((url, dict(payload)))
                if payload.get("phase") == "announcing":
                    return {"ok": True, "announcement": {"proceed": True}}
                if payload.get("phase") == "waiting_verdict":
                    return {
                        "ok": True,
                        "verdict": {
                            "schema": "gogoguard.checkpoint_verdict.v1",
                            "verdictId": "vd_1",
                            "missionId": "mission-1",
                            "mapVersion": "map-v1",
                            "routeId": "route-v1",
                            "checkpointId": "cp_01",
                            "attempt": 1,
                            "action": "continue",
                            "expiresAt": "2099-01-01T00:00:00+00:00",
                        },
                    }
                return {"ok": True}

            gimbal = FakeGimbal()
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=control,
                inbox_path=inbox,
                state_path=root / "state.json",
                post_json=poster,
                gimbal=gimbal,
            )
            runtime = {
                "mapVersion": "map-v1",
                "routeId": "route-v1",
                "routeProgressIndex": 12,
                "checkpoint": {
                    "missionId": "mission-1",
                    "activeCheckpointId": "cp_01",
                    "activeRouteProgressIndex": 12,
                    "attempt": 1,
                    "phase": "WAITING_PLATFORM",
                    "dwellSec": 1,
                    "camera": {"pan": -15, "tilt": 22.5, "roll": 0},
                    "verdictTimeoutSec": 15,
                },
            }
            navigation.write_text(json.dumps({"runtime": runtime}), encoding="utf-8")
            coordinator.tick(now=100.0)
            coordinator.tick(now=101.1)
            self.assertEqual(json.loads(control.read_text())["action"], "capture")
            self.assertEqual(gimbal.moves[0]["pan_body_deg"], -15.0)
            self.assertEqual(
                json.loads((root / "state.json").read_text())["camera"]["pan"],
                -14.8,
            )
            pose_ready = next(
                payload for _, payload in calls if payload.get("phase") == "pose_ready"
            )
            self.assertEqual(
                pose_ready["cameraPose"]["targetDeg"],
                {"pan": -15.0, "tilt": 22.5, "roll": 0.0},
            )
            self.assertEqual(
                pose_ready["cameraPose"]["actualDeg"],
                {"pan": -14.8, "tilt": 22.4, "roll": 0.0},
            )
            self.assertEqual(pose_ready["cameraPose"]["toleranceDeg"], 2.0)
            self.assertTrue(pose_ready["cameraPose"]["converged"])
            runtime["checkpoint"]["phase"] = "WAITING_VERDICT"
            navigation.write_text(json.dumps({"runtime": runtime}), encoding="utf-8")
            coordinator.tick(now=102.0)
            self.assertEqual(json.loads(control.read_text())["action"], "continue")
            phases = [payload["phase"] for _, payload in calls if "phase" in payload]
            self.assertEqual(
                phases[:7],
                [
                    "checkpoint_reached", "stopped", "pose_ready", "announcing",
                    "capture_ready", "captured", "waiting_verdict",
                ],
            )

    def test_platform_does_not_report_pose_ready_when_gimbal_misses_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            calls = []
            navigation.write_text(
                json.dumps(
                    {
                        "runtime": {
                            "mapVersion": "map-v1",
                            "routeId": "route-v1",
                            "checkpoint": {
                                "missionId": "mission-1",
                                "activeCheckpointId": "cp_01",
                                "activeRouteProgressIndex": 12,
                                "attempt": 1,
                                "phase": "WAITING_PLATFORM",
                                "camera": {"pan": 35, "tilt": 5, "roll": 0},
                            },
                        }
                    }
                ),
                encoding="utf-8",
            )
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=root / "control.json",
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=lambda url, payload, timeout: calls.append(payload) or {"ok": True},
                gimbal=MisalignedGimbal(),
            )
            with self.assertRaisesRegex(RuntimeError, "did not reach checkpoint view"):
                coordinator.tick(now=100.0)
            self.assertNotIn("pose_ready", [item.get("phase") for item in calls])

    def test_platform_coordinator_does_not_take_local_operator_checkpoint(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            control = root / "control.json"
            calls = []
            navigation.write_text(
                json.dumps(
                    {
                        "runtime": {
                            "checkpoint": {
                                "missionId": "local:map-v1:1:abcd",
                                "decisionMode": "local_operator",
                                "activeCheckpointId": "cp_01",
                                "attempt": 1,
                                "phase": "WAITING_PLATFORM",
                            }
                        }
                    }
                ),
                encoding="utf-8",
            )
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=control,
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=lambda *args: calls.append(args) or {},
            )
            coordinator.tick(now=100.0)
            self.assertEqual(calls, [])
            self.assertFalse(control.exists())

    def test_announcement_terminal_advances_while_mission_events_return_401(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            control = root / "control.json"
            inbox = root / "inbox.jsonl"
            runtime = {
                "mapVersion": "map-v1",
                "routeId": "route-v1",
                "checkpoint": {
                    "missionId": "mission-1",
                    "activeCheckpointId": "cp_01",
                    "activeRouteProgressIndex": 12,
                    "attempt": 1,
                    "phase": "WAITING_PLATFORM",
                    "evidenceTransactionVersion": 1,
                    "dwellSec": 1,
                    "camera": {"pan": -15, "tilt": 22.5, "roll": 0},
                },
            }
            navigation.write_text(json.dumps({"runtime": runtime}), encoding="utf-8")
            inbox.write_text(
                json.dumps(
                    {
                        "schema": "gogoguard.announcement_completed.v1",
                        "announcementId": "ann_1",
                        "missionId": "mission-1",
                        "checkpointId": "cp_01",
                        "attempt": 1,
                        "status": "completed",
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            calls = []

            def reject_events(url, payload, timeout):
                calls.append((url, payload.get("phase")))
                raise DeviceTokenError("credential redacted")

            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=control,
                inbox_path=inbox,
                state_path=root / "state.json",
                post_json=reject_events,
                gimbal=FakeGimbal(),
                evidence_transaction_enabled=True,
            )
            coordinator.tick(now=100.0)
            state = json.loads((root / "state.json").read_text())
            self.assertEqual(state["stage"], "dwelling")
            self.assertEqual(state["announcementId"], "ann_1")
            self.assertEqual(state["lastEventFailure"]["code"], "HTTP_401")
            durable_pose_ready = next(
                item["payload"]
                for item in state["pendingEvents"]
                if item["payload"].get("phase") == "pose_ready"
            )
            self.assertEqual(
                durable_pose_ready["cameraPose"]["targetDeg"]["pan"], -15.0
            )
            self.assertEqual(
                durable_pose_ready["cameraPose"]["actualDeg"]["pan"], -14.8
            )
            coordinator.tick(now=101.1)
            self.assertFalse(control.exists())
            self.assertEqual(coordinator.context["stage"], "capture_reconciling")
            self.assertIn("capture_ready", [phase for _, phase in calls])

    def test_evidence_transaction_path_is_default_off_and_requires_version_one(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            navigation.write_text(
                json.dumps(
                    {
                        "runtime": {
                            "mapVersion": "map-v1",
                            "routeId": "route-v1",
                            "checkpoint": {
                                "missionId": "mission-1",
                                "activeCheckpointId": "cp_01",
                                "attempt": 1,
                                "phase": "WAITING_PLATFORM",
                                "evidenceTransactionVersion": 1,
                            },
                        }
                    }
                ),
                encoding="utf-8",
            )
            gimbal = FakeGimbal()
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=root / "control.json",
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=lambda *args: (_ for _ in ()).throw(DeviceTokenError()),
                gimbal=gimbal,
            )
            with self.assertRaises(DeviceTokenError):
                coordinator.tick(now=100.0)
            self.assertEqual(gimbal.moves, [])
            self.assertEqual(json.loads((root / "state.json").read_text())["stage"], "new")

    def test_verdict_is_applied_before_failed_event_and_only_once_after_restart(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            inbox = root / "inbox.jsonl"
            state = root / "state.json"
            navigation.write_text(
                json.dumps(
                    {
                        "runtime": {
                            "mapVersion": "map-v1",
                            "routeId": "route-v1",
                            "checkpoint": {
                                "missionId": "mission-1",
                                "activeCheckpointId": "cp_01",
                                "activeRouteProgressIndex": 12,
                                "attempt": 1,
                                "phase": "WAITING_VERDICT",
                                "evidenceTransactionVersion": 1,
                                "verdictTimeoutSec": 15,
                            },
                        }
                    }
                ),
                encoding="utf-8",
            )
            inbox.write_text(
                json.dumps(
                    {
                        "schema": "gogoguard.checkpoint_verdict.v1",
                        "verdictId": "vd_1",
                        "missionId": "mission-1",
                        "mapVersion": "map-v1",
                        "routeId": "route-v1",
                        "checkpointId": "cp_01",
                        "attempt": 1,
                        "action": "continue",
                        "expiresAt": "2099-01-01T00:00:00+00:00",
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            actions = []
            requests = []

            def reject_network(url, payload, timeout):
                requests.append((url, dict(payload)))
                raise DeviceTokenError("credential redacted")

            def build():
                value = CheckpointCoordinator(
                    heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                    navigation_status_path=navigation,
                    control_path=root / "control.json",
                    inbox_path=inbox,
                    state_path=state,
                    post_json=reject_network,
                    evidence_transaction_enabled=True,
                )
                value._write_control = actions.append
                return value

            build().tick(now=100.0)
            build().tick(now=101.0)
            self.assertEqual(actions, ["continue"])
            persisted = json.loads(state.read_text())
            self.assertEqual(persisted["stage"], "verdict_continue")
            self.assertEqual(persisted["appliedMessageIds"], ["vd_1"])
            acknowledgements = [
                payload for url, payload in requests if url.endswith("/verdict/ack")
            ]
            self.assertEqual(acknowledgements[0]["robotId"], "LLYJ0001")

    def test_wrong_announcement_identity_does_not_advance(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            inbox = root / "inbox.jsonl"
            navigation.write_text(
                json.dumps(
                    {
                        "runtime": {
                            "mapVersion": "map-v1",
                            "routeId": "route-v1",
                            "checkpoint": {
                                "missionId": "mission-1",
                                "activeCheckpointId": "cp_01",
                                "attempt": 1,
                                "phase": "WAITING_PLATFORM",
                                "evidenceTransactionVersion": 1,
                            },
                        }
                    }
                ),
                encoding="utf-8",
            )
            inbox.write_text(
                json.dumps(
                    {
                        "schema": "gogoguard.announcement_completed.v1",
                        "announcementId": "ann_wrong",
                        "missionId": "other-mission",
                        "checkpointId": "cp_01",
                        "attempt": 1,
                        "status": "completed",
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=root / "control.json",
                inbox_path=inbox,
                state_path=root / "state.json",
                post_json=lambda *args: (_ for _ in ()).throw(DeviceTokenError()),
                evidence_transaction_enabled=True,
            )
            coordinator.tick(now=100.0)
            persisted = json.loads((root / "state.json").read_text())
            self.assertEqual(persisted["stage"], "announcing")
            self.assertNotIn("announcementCompleted", persisted)

    def test_401_event_retry_uses_low_frequency_backoff(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            navigation.write_text(
                json.dumps(
                    {
                        "runtime": {
                            "mapVersion": "map-v1",
                            "routeId": "route-v1",
                            "checkpoint": {
                                "missionId": "mission-1",
                                "activeCheckpointId": "cp_01",
                                "attempt": 1,
                                "phase": "WAITING_PLATFORM",
                                "evidenceTransactionVersion": 1,
                            },
                        }
                    }
                ),
                encoding="utf-8",
            )
            calls = []

            def reject_events(url, payload, timeout):
                calls.append(payload["eventId"])
                raise DeviceTokenError()

            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=root / "control.json",
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=reject_events,
                evidence_transaction_enabled=True,
            )
            coordinator.tick(now=100.0)
            first_count = len(calls)
            coordinator.tick(now=101.0)
            self.assertEqual(len(calls), first_count)
            coordinator.tick(now=160.0)
            self.assertEqual(calls[first_count:first_count * 2], calls[:first_count])
            self.assertEqual(len(calls), first_count * 2 + 1)

    def test_durable_pre_identity_event_gets_robot_id_before_retry(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            calls = []
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=root / "navigation.json",
                control_path=root / "control.json",
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=lambda url, payload, timeout: calls.append(dict(payload)) or {},
                evidence_transaction_enabled=True,
                robot_id="LLYJ0001",
            )
            coordinator.context["pendingEvents"] = [
                {
                    "payload": {
                        "eventId": "me_pre_identity",
                        "phase": "stopped",
                        "observedAt": utc_now(),
                    },
                    "attempts": 3,
                    "nextAttemptAt": 0.0,
                }
            ]
            coordinator._flush_pending_events(100.0)
            self.assertEqual(calls[0]["robotId"], "LLYJ0001")
            self.assertFalse(coordinator.context.get("pendingEvents"))

    def test_stale_durable_event_is_audited_without_platform_side_effect(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            calls = []
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=root / "navigation.json",
                control_path=root / "control.json",
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=lambda url, payload, timeout: calls.append(dict(payload)) or {},
                evidence_transaction_enabled=True,
                robot_id="LLYJ0001",
            )
            coordinator.context["pendingEvents"] = [
                {
                    "payload": {
                        "eventId": "me_stale",
                        "phase": "announcing",
                        "observedAt": "2020-01-01T00:00:00+00:00",
                    },
                    "attempts": 1,
                    "nextAttemptAt": 0.0,
                }
            ]
            coordinator._flush_pending_events(100.0)
            self.assertEqual(calls, [])
            self.assertEqual(coordinator.context["staleEventsDiscarded"], 1)
            audit = [
                json.loads(line)
                for line in coordinator.stale_event_audit_path.read_text().splitlines()
            ]
            self.assertEqual(audit[0]["reason"], "EVENT_TOO_OLD")

    def test_durable_event_without_timezone_is_never_transmitted(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            calls = []
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=root / "navigation.json",
                control_path=root / "control.json",
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=lambda url, payload, timeout: calls.append(dict(payload)) or {},
                evidence_transaction_enabled=True,
                robot_id="LLYJ0001",
            )
            coordinator.context["pendingEvents"] = [
                {
                    "payload": {
                        "eventId": "me_naive_time",
                        "phase": "stopped",
                        "observedAt": "2026-08-23T08:00:00.000",
                    },
                    "attempts": 0,
                    "nextAttemptAt": 0.0,
                }
            ]
            coordinator._flush_pending_events(100.0)
            self.assertEqual(calls, [])
            audit = json.loads(
                coordinator.stale_event_audit_path.read_text().splitlines()[0]
            )
            self.assertEqual(audit["reason"], "OBSERVED_AT_TIMEZONE_MISSING")

    def test_generation_fence_ignores_pre_restart_navigation_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            navigation.write_text(
                json.dumps(
                    {
                        "edge_generation_id": "old-generation",
                        "runtime": {
                            "checkpoint": {
                                "missionId": "stale-mission",
                                "activeCheckpointId": "cp_old",
                                "attempt": 1,
                                "phase": "WAITING_PLATFORM",
                            }
                        },
                    }
                ),
                encoding="utf-8",
            )
            calls = []
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=root / "control.json",
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=lambda *args: calls.append(args) or {},
                navigation_generation_id="current-generation",
            )
            coordinator.tick(now=100.0)
            self.assertEqual(calls, [])
            self.assertEqual(coordinator.context, {})


    def test_platform_asset_upload_resumes_and_stops_before_activation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            archive = root / "bundle.zip"
            with zipfile.ZipFile(archive, "w") as output:
                output.writestr("manifest.json", "{}")
            manifest = root / "manifest.json"
            manifest.write_text(json.dumps({"files": []}), encoding="utf-8")
            uploader = PlatformAssetUploader(
                root,
                {
                    "base_url": "http://39.96.37.187/api/v1",
                    "allow_insecure_http": True,
                },
            )
            requests = []

            def request(method, path, body=None, raw=None):
                requests.append((method, path))
                if path.endswith("/init"):
                    return {
                        "uploadId": "ab_1", "chunkSize": 8, "totalChunks": 2,
                        "siteId": "site-1", "siteName": "天津大学北洋园",
                        "missing": [0, 1],
                    }
                if "/chunk" in path:
                    index = int(path.rsplit("=", 1)[-1])
                    return {"received": list(range(index + 1)), "missing": []}
                if path.endswith("/complete"):
                    return {"ok": True, "summary": {"checkpoints": 1}}
                raise AssertionError((method, path))

            uploader._request = request
            descriptor = {
                "archive": str(archive),
                "manifest": str(manifest),
                "mapVersion": "map-v1",
                "routeId": "route-v1",
                "archiveBytes": archive.stat().st_size,
                "archiveSha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
            }
            with patch.dict(os.environ, {"GOGOGUARD_DEVICE_TOKEN": "secret"}):
                uploader.start("map-123456789abc", descriptor)
                deadline = time.time() + 2
                while time.time() < deadline and uploader.status("map-123456789abc")["state"] not in {"verified", "failed"}:
                    time.sleep(0.01)
            self.assertEqual(uploader.status("map-123456789abc")["state"], "verified")
            self.assertFalse(any(path.endswith("/activate") for _, path in requests))

    def test_platform_asset_token_falls_back_to_commissioned_macos_keychain(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            uploader = PlatformAssetUploader(
                Path(temporary),
                {
                    "device_token_env": "GOGOGUARD_TEST_DEVICE_TOKEN",
                    "device_token_keychain_service": "gogoguard-test-token",
                    "device_token_keychain_account": "field-operator",
                },
            )
            keychain = Mock(returncode=0, stdout="keychain-token\n", stderr="")
            with (
                patch.dict(os.environ, {}, clear=True),
                patch(
                    "gogoguard_field_workstation.platform_assets.platform.system",
                    return_value="Darwin",
                ),
                patch(
                    "gogoguard_field_workstation.platform_assets.Path.is_file",
                    return_value=True,
                ),
                patch(
                    "gogoguard_field_workstation.platform_assets.subprocess.run",
                    return_value=keychain,
                ) as lookup,
            ):
                self.assertEqual(uploader._device_token(), "keychain-token")
                self.assertEqual(uploader._device_token(), "keychain-token")
            lookup.assert_called_once()
            command = lookup.call_args.args[0]
            self.assertEqual(command[-5:], ["-a", "field-operator", "-s", "gogoguard-test-token", "-w"])

    def test_platform_asset_token_environment_override_skips_keychain(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            uploader = PlatformAssetUploader(
                Path(temporary),
                {"device_token_env": "GOGOGUARD_TEST_DEVICE_TOKEN"},
            )
            with (
                patch.dict(
                    os.environ,
                    {"GOGOGUARD_TEST_DEVICE_TOKEN": "process-token"},
                    clear=True,
                ),
                patch(
                    "gogoguard_field_workstation.platform_assets.subprocess.run"
                ) as lookup,
            ):
                self.assertEqual(uploader._device_token(), "process-token")
            lookup.assert_not_called()


if __name__ == "__main__":
    unittest.main()
