from __future__ import annotations

import io
import json
import unittest
from unittest.mock import patch

from gogoguard_device_io.camera import CameraGateway


CONFIG = {
    "enabled": True,
    "source": "z1pro",
    "stream_path": "z1pro",
    "webrtc_port": 8889,
    "status_url": "http://127.0.0.1:9997/v3/paths/get/z1pro",
}


class _Response(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class CameraGatewayTest(unittest.TestCase):
    def test_demo_mode_never_connects_physical_camera(self) -> None:
        status = CameraGateway("demo", CONFIG).status()
        self.assertFalse(status.enabled)
        self.assertFalse(status.ready)

    @patch("gogoguard_device_io.camera.urlopen")
    def test_robot_status_translates_mediamtx_contract(self, mocked_urlopen) -> None:
        mocked_urlopen.return_value = _Response(json.dumps({
            "online": True,
            "ready": True,
            "tracks2": [{"codec": "H264", "codecProps": {
                "width": 1920, "height": 1080, "profile": "High", "level": "5.2",
            }}],
        }).encode())

        status = CameraGateway("robot", CONFIG).status()

        self.assertTrue(status.enabled)
        self.assertTrue(status.online)
        self.assertTrue(status.ready)
        self.assertEqual((status.width, status.height), (1920, 1080))
        self.assertEqual(status.profile, "High")

    @patch("gogoguard_device_io.camera.urlopen", side_effect=OSError("offline"))
    def test_gateway_failure_is_reported_without_raising(self, _mocked_urlopen) -> None:
        status = CameraGateway("robot", CONFIG).status()
        self.assertFalse(status.ready)
        self.assertIn("不可用", status.message)
