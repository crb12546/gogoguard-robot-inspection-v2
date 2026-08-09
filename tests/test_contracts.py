import json
import unittest

from gogoguard_contracts import CameraStreamStatus, DeviceStatus, MapJob, RecordingSession, json_ready


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
