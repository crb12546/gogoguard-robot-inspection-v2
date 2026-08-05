import unittest

from gogoguard_contracts import CameraStreamStatus, DeviceStatus, MapJob, RecordingSession, json_ready


class ContractsTest(unittest.TestCase):
    def test_contracts_are_json_ready(self) -> None:
        self.assertEqual(json_ready(RecordingSession())["state"], "idle")
        self.assertEqual(json_ready(MapJob())["state"], "queued")
        self.assertEqual(json_ready(DeviceStatus())["schema"], "gogoguard.device_status.v1")
        self.assertEqual(json_ready(CameraStreamStatus())["protocol"], "webrtc")
