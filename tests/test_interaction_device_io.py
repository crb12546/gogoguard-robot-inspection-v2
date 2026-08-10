from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from gogoguard_device_io import (
    Go2VolumeController,
    NavigationReadOnlyGuard,
    SpeakerJitterBuffer,
    decode_s24_3le_stereo_to_s16_mono,
    load_interaction_hardware_profile,
)


ROOT = Path(__file__).resolve().parents[1]


def s24(value: int) -> bytes:
    if value < 0:
        value += 1 << 24
    return value.to_bytes(3, "little")


class InteractionDeviceIoTest(unittest.TestCase):
    def test_commissioned_hardware_profile_is_exact(self) -> None:
        profile = load_interaction_hardware_profile(
            ROOT / "config/robot/interaction-hardware.json"
        )
        self.assertEqual(profile.microphone_device, "hw:CARD=B2,DEV=0")
        self.assertEqual(profile.microphone_gain, 4.0)
        self.assertEqual(profile.speaker_default_volume, 10)
        self.assertEqual((profile.video_width, profile.video_height, profile.video_fps), (1920, 1080, 30))

    def test_owned_vui_boundary_and_container_keep_the_zero_to_ten_scale(self) -> None:
        source = (
            ROOT / "modules/device_io/ros/go2_cmd_vel_bridge/src/go2_vui_control.cpp"
        ).read_text(encoding="utf-8")
        cmake = (
            ROOT / "modules/device_io/ros/go2_cmd_vel_bridge/CMakeLists.txt"
        ).read_text(encoding="utf-8")
        dockerfile = (ROOT / "deployment/container/Dockerfile").read_text(encoding="utf-8")
        self.assertIn("unitree/robot/go2/vui/vui_client.hpp", source)
        self.assertIn("requested > 10", source)
        self.assertIn("add_executable(go2_vui_control", cmake)
        self.assertIn("test -x install/lib/go2_cmd_vel_bridge/go2_vui_control", dockerfile)

    def test_boya_s24_stereo_conversion_gain_and_clip(self) -> None:
        converted = decode_s24_3le_stereo_to_s16_mono(
            s24(256) + s24(256) + s24(-512) + s24(-512), gain=4
        )
        self.assertEqual(int.from_bytes(converted[0:2], "little", signed=True), 4)
        self.assertEqual(int.from_bytes(converted[2:4], "little", signed=True), -8)
        clipped = decode_s24_3le_stereo_to_s16_mono(
            s24(0x7FFFFF) + s24(0x7FFFFF), gain=16
        )
        self.assertEqual(int.from_bytes(clipped, "little", signed=True), 32767)

    def test_speaker_prebuffers_and_reprime_after_underflow(self) -> None:
        buffer = SpeakerJitterBuffer(prebuffer_ms=60, max_buffer_ms=100)
        frame = b"\x01\x00" * 960
        buffer.append(frame * 2)
        self.assertEqual(buffer.read_frame(), bytes(len(frame)))
        buffer.append(frame)
        self.assertEqual(buffer.read_frame(), frame)
        self.assertEqual(buffer.read_frame(), frame)
        self.assertEqual(buffer.read_frame(), frame)
        self.assertEqual(buffer.read_frame(), bytes(len(frame)))
        self.assertEqual(buffer.status()["underflowFrames"], 1)

    def test_speaker_queue_is_bounded_and_interrupt_flushes(self) -> None:
        buffer = SpeakerJitterBuffer(prebuffer_ms=20, max_buffer_ms=40)
        frame = b"\x01\x00" * 960
        buffer.append(frame * 4)
        self.assertEqual(buffer.status()["dropEvents"], 1)
        self.assertLessEqual(buffer.status()["queuedBytes"], len(frame) * 2)
        buffer.flush()
        self.assertEqual(buffer.status()["queuedBytes"], 0)
        self.assertEqual(buffer.status()["flushes"], 1)

    def test_navigation_guard_accepts_only_stopped_or_zero_unauthorized(self) -> None:
        stopped = NavigationReadOnlyGuard.evaluate({
            "runtime": None, "runtime_process": {"running": False},
            "motion_bridge": {"running": False}, "commands": {"final": None},
        })
        unauthorized = NavigationReadOnlyGuard.evaluate({
            "runtime": {"motionAuthorized": False},
            "runtime_process": {"running": False}, "motion_bridge": {"running": False},
            "commands": {"final": {"vx": 0, "vy": 0, "wz": 0}},
        })
        moving = NavigationReadOnlyGuard.evaluate({
            "runtime": {"motionAuthorized": True},
            "runtime_process": {"running": True}, "motion_bridge": {"running": True},
            "commands": {"final": {"vx": 0.2, "vy": 0, "wz": 0}},
        })
        self.assertTrue(stopped["safe"])
        self.assertTrue(unauthorized["safe"])
        self.assertFalse(moving["safe"])

    @patch("gogoguard_device_io.interaction_media.subprocess.run")
    def test_volume_ten_is_set_and_verified(self, run) -> None:
        run.return_value.stdout = json.dumps({
            "requested": 10, "setCode": 0, "getCode": 0, "observed": 10,
        })
        result = Go2VolumeController(Path("/opt/go2_vui_control"), "eth0").set_and_verify(10)
        self.assertEqual(result["observed"], 10)
        self.assertEqual(run.call_args.args[0][-2:], ["set-volume", "10"])


if __name__ == "__main__":
    unittest.main()
