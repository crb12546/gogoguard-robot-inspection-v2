import json
import unittest

from gogoguard_contracts import (
    CameraStreamStatus,
    DeviceStatus,
    MapJob,
    RecordingSession,
    is_safe_external_id,
    json_ready,
    validate_platform_mission_message,
)


class ContractsTest(unittest.TestCase):
    def test_contracts_are_json_ready(self) -> None:
        self.assertEqual(json_ready(RecordingSession())["state"], "idle")
        self.assertEqual(json_ready(MapJob())["state"], "queued")
        self.assertEqual(json_ready(DeviceStatus())["schema"], "gogoguard.device_status.v1")
        self.assertEqual(json_ready(CameraStreamStatus())["protocol"], "webrtc")

    def test_non_finite_numbers_become_json_null(self) -> None:
        value = json_ready(
            {
                "positive": float("inf"),
                "negative": float("-inf"),
                "unknown": float("nan"),
                "finite": 1.25,
            }
        )
        self.assertEqual(
            value,
            {
                "positive": None,
                "negative": None,
                "unknown": None,
                "finite": 1.25,
            },
        )
        self.assertNotIn("Infinity", json.dumps(value, allow_nan=False))
        self.assertNotIn("NaN", json.dumps(value, allow_nan=False))

    def test_frozen_platform_ids_and_verdict_are_validated(self) -> None:
        mission_id = "mission-20260811-143022-RG-狗02"
        self.assertTrue(is_safe_external_id(mission_id))
        verdict = validate_platform_mission_message(
            {
                "schema": "gogoguard.checkpoint_verdict.v1",
                "verdictId": "vd_a1b2c3",
                "missionId": mission_id,
                "mapVersion": "map-6855ba54ae11",
                "routeId": "route-6855ba54ae11-workspace-r4",
                "checkpointId": "cp_01",
                "attempt": 1,
                "action": "continue",
                "expiresAt": "2099-01-01T00:00:00+00:00",
            }
        )
        self.assertEqual(verdict["missionId"], mission_id)
        with self.assertRaisesRegex(ValueError, "action"):
            validate_platform_mission_message({**verdict, "action": "move"})
