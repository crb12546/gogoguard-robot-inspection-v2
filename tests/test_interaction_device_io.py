from __future__ import annotations

import asyncio
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from gogoguard_device_io import (
    Go2VolumeController,
    LiveKitGo2Transport,
    NavigationReadOnlyGuard,
    SpeakerJitterBuffer,
    amplify_s16_pcm,
    decode_s24_3le_stereo_to_s16_mono,
    load_interaction_hardware_profile,
    microphone_capture_command,
)
from gogoguard_contracts import MediaConnectionReceipt, RealtimeMediaSessionRequest


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
        self.assertEqual(profile.speaker_gain, 3.0)
        self.assertEqual((profile.video_width, profile.video_height, profile.video_fps), (1920, 1080, 30))

    def test_owned_vui_boundary_and_container_keep_the_zero_to_ten_scale(self) -> None:
        source = (
            ROOT / "modules/device_io/ros/go2_cmd_vel_bridge/src/go2_vui_control.cpp"
        ).read_text(encoding="utf-8")
        cmake = (
            ROOT / "modules/device_io/ros/go2_cmd_vel_bridge/CMakeLists.txt"
        ).read_text(encoding="utf-8")
        dockerfile = (ROOT / "deployment/container/Dockerfile").read_text(encoding="utf-8")
        combined = (ROOT / "deployment/container/Dockerfile.combined").read_text(
            encoding="utf-8"
        )
        self.assertIn("unitree/robot/go2/vui/vui_client.hpp", source)
        self.assertIn("requested > 10", source)
        self.assertIn("add_executable(go2_vui_control", cmake)
        self.assertIn("test -x install/lib/go2_cmd_vel_bridge/go2_vui_control", dockerfile)
        self.assertIn("src/go2_vui_control.cpp", combined)
        self.assertIn(
            "/opt/gogoguard/ros_ws/install/lib/go2_cmd_vel_bridge/go2_vui_control",
            combined,
        )

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

    def test_boya_capture_reuses_frozen_ffmpeg_instead_of_missing_arecord(self) -> None:
        profile = load_interaction_hardware_profile(
            ROOT / "config/robot/interaction-hardware.json"
        )
        command = microphone_capture_command(profile)
        self.assertEqual(command[0], "ffmpeg")
        self.assertNotIn("arecord", command)
        self.assertIn("alsa", command)
        self.assertEqual(command.count("pcm_s24le"), 2)
        self.assertEqual(command[-2:], ["s24le", "pipe:1"])

    def test_speaker_gain_is_three_x_with_peak_limiting(self) -> None:
        quiet = (1000).to_bytes(2, "little", signed=True) * 4
        amplified = amplify_s16_pcm(quiet, gain=3.0)
        self.assertEqual(
            int.from_bytes(amplified[:2], "little", signed=True), 3000
        )
        loud = (30000).to_bytes(2, "little", signed=True) * 4
        limited = amplify_s16_pcm(loud, gain=3.0)
        peak = int.from_bytes(limited[:2], "little", signed=True)
        self.assertGreater(peak, 30000)
        self.assertLessEqual(peak, 32767)

    def test_unitree_rtc_loads_before_other_native_media_runtime(self) -> None:
        native = (
            ROOT
            / "modules/device_io/gogoguard_device_io/interaction_native.py"
        ).read_text(encoding="utf-8")
        service = (
            ROOT
            / "services/interaction_edge/gogoguard_interaction_edge/service.py"
        ).read_text(encoding="utf-8")
        dockerfile = (
            ROOT / "deployment/container/Dockerfile.combined"
        ).read_text(encoding="utf-8")
        unitree = native.index("from unitree_webrtc_connect import")
        av = native.index("import av")
        livekit = native.index("from livekit import rtc")
        self.assertLess(unitree, av)
        self.assertLess(av, livekit)
        self.assertLess(
            service.index("volume_controller.set_and_verify"),
            service.index("real_transport.preload_dependencies()"),
        )
        self.assertIn("real_transport.preload_dependencies()", service)
        self.assertIn(
            "gogoguard_device_io.interaction_native import NATIVE_DEPENDENCIES",
            dockerfile,
        )

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

    def test_livekit_data_channel_accepts_only_frozen_agent_schemas(self) -> None:
        profile = load_interaction_hardware_profile(
            ROOT / "config/robot/interaction-hardware.json"
        )
        transport = LiveKitGo2Transport(
            profile=profile, unitree_aes_128_key="test-only"
        )
        self.assertFalse(
            transport.publish_data(
                {"schema": "gogoguard.robot_pose.v1", "x": 1.0},
                topic="gogoguard.robot_pose.v1",
                reliable=False,
            )
        )
        received = []
        transport.set_wake_transcript_handler(received.append)
        message = {
            "schema": "gogoguard.wake_transcript.v1",
            "action": "wake_transcript",
            "eventId": "wake-event-1",
            "text": "小九小九",
            "state": "awake",
            "wake": True,
        }
        self.assertFalse(
            transport.handle_data_message(
                message, participant_identity="robot:LLYJ0001"
            )
        )
        self.assertTrue(
            transport.handle_data_message(
                message, participant_identity="agent:patrol-test-001"
            )
        )
        self.assertEqual(
            received,
            [
                {
                    "action": "wake_transcript",
                    "eventId": "wake-event-1",
                    "text": "小九小九",
                }
            ],
        )
        mission_messages = []
        transport.set_mission_message_handler(mission_messages.append)
        verdict = {
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
        self.assertTrue(
            transport.handle_data_message(
                verdict, participant_identity="agent:patrol-test-001"
            )
        )
        self.assertEqual(mission_messages, [verdict])
        self.assertFalse(
            transport.handle_data_message(
                {"schema": "gogoguard.motion.v1", "action": "goto"},
                participant_identity="agent:patrol-test-001",
            )
        )

    def test_audio_timing_separates_network_gaps_from_response_latency(self) -> None:
        profile = load_interaction_hardware_profile(
            ROOT / "config/robot/interaction-hardware.json"
        )
        transport = LiveKitGo2Transport(
            profile=profile, unitree_aes_128_key="test-only"
        )
        transport._accepted_transcript(now=10.0)
        transport._observe_agent_audio_frame(active=False, now=10.02)
        transport._observe_agent_audio_frame(active=True, now=10.42)
        timing = transport.status()["audioTiming"]
        self.assertEqual(timing["agentFrames"], 2)
        self.assertEqual(timing["agentFrameGapOver40ms"], 1)
        self.assertEqual(timing["agentFrameGapMaxMs"], 400.0)
        self.assertEqual(timing["transcriptToAudioSamples"], 1)
        self.assertEqual(timing["lastTranscriptToAudioMs"], 420.0)

    def test_timed_out_media_generation_is_cancelled_and_next_start_succeeds(self) -> None:
        profile = load_interaction_hardware_profile(
            ROOT / "config/robot/interaction-hardware.json"
        )

        class RetryTransport(LiveKitGo2Transport):
            def __init__(self):
                super().__init__(
                    profile=profile,
                    unitree_aes_128_key="test-only",
                    startup_timeout_s=0.05,
                    cleanup_timeout_s=1.0,
                )
                self.attempt = 0

            async def _run(self, request):
                self.attempt += 1
                self._loop = asyncio.get_running_loop()
                self._session_stop = asyncio.Event()
                if self.attempt == 1:
                    self._startup_stage = "LIVEKIT_CONNECT"
                    await asyncio.Event().wait()
                self._ever_connected = True
                self._startup_stage = "MEDIA_READY"
                self._ready.put(
                    MediaConnectionReceipt(
                        participant_id="robot:LLYJ0001",
                        video_published=True,
                        audio_published=True,
                        audio_subscribed=True,
                        data_connected=True,
                    )
                )
                await self._session_stop.wait()

        request = RealtimeMediaSessionRequest(
            command_id=1,
            robot_id="LLYJ0001",
            url="ws://39.96.37.187:7880",
            room="patrol-test-001",
            token="not-inspected-by-transport-test",
        )
        transport = RetryTransport()
        with self.assertRaisesRegex(RuntimeError, "realtime media startup failed") as failure:
            transport.connect(request)
        self.assertEqual(failure.exception.safe_code, "LIVEKIT_CONNECT_TIMEOUT")
        self.assertFalse(transport.status()["running"])
        self.assertFalse(transport.status()["cleanupStuck"])

        receipt = transport.connect(request)
        self.assertEqual(receipt.participant_id, "robot:LLYJ0001")
        self.assertEqual(transport.status()["startupStage"], "MEDIA_READY")
        transport.disconnect()
        self.assertFalse(transport.status()["running"])

    @patch("gogoguard_device_io.interaction_media.subprocess.run")
    def test_volume_ten_is_set_and_verified(self, run) -> None:
        run.return_value.stdout = json.dumps({
            "requested": 10, "setCode": 0, "getCode": 0, "observed": 10,
        })
        result = Go2VolumeController(Path("/opt/go2_vui_control"), "eth0").set_and_verify(10)
        self.assertEqual(result["observed"], 10)
        self.assertEqual(run.call_args.args[0][-2:], ["set-volume", "10"])
        self.assertTrue(
            run.call_args.kwargs["env"]["LD_LIBRARY_PATH"].startswith(
                "/opt/gogoguard/deps/lib"
            )
        )

    @patch("gogoguard_device_io.interaction_media.time.sleep")
    @patch("gogoguard_device_io.interaction_media.subprocess.run")
    def test_volume_initialization_retries_a_transient_command_failure(
        self, run, sleep
    ) -> None:
        success = type("Completed", (), {
            "stdout": json.dumps({
                "requested": 10, "setCode": 0, "getCode": 0, "observed": 10,
            })
        })()
        run.side_effect = [
            subprocess.TimeoutExpired(["go2_vui_control"], 6),
            success,
        ]
        result = Go2VolumeController(
            Path("/opt/go2_vui_control"), "eth0"
        ).set_and_verify(10)
        self.assertEqual(result["observed"], 10)
        self.assertEqual(run.call_count, 2)
        sleep.assert_called_once_with(0.25)


if __name__ == "__main__":
    unittest.main()
