from __future__ import annotations

import struct
import unittest
from unittest.mock import patch

from gogoguard_device_io import (
    Z1ProGimbal,
    build_gcu_packet,
    crc16_ccitt_nibble,
    parse_gcu_reply,
)


OFFICIAL_REPLY = bytes.fromhex(
    "8A 5E 49 00 02 12 01 80 0C FE F4 01 DD FC 20 00 4A 18 FF FF A5 03 "
    "47 18 FF FF 01 00 FE FF 00 00 00 00 00 00 00 01 1F 32 29 00 00 "
    "06 17 00 00 24 F2 DF 65 16 EE AA 16 A3 A0 00 00 2B 01 14 00 00 "
    "00 00 08 00 00 20 00 EC 85"
)


def reply_with_angles(
    roll: float,
    tilt: float,
    pan: float,
    *,
    absolute_roll: float | None = None,
    absolute_pitch: float | None = None,
    absolute_yaw: float = 0.0,
) -> bytes:
    frame = bytearray(OFFICIAL_REPLY)
    struct.pack_into(
        "<hhh",
        frame,
        12,
        int(round(roll * 100)),
        int(round(tilt * 100)),
        int(round(pan * 100)),
    )
    struct.pack_into(
        "<hhH",
        frame,
        18,
        int(round((roll if absolute_roll is None else absolute_roll) * 100)),
        int(round((tilt if absolute_pitch is None else absolute_pitch) * 100)),
        int(round(absolute_yaw * 100)) % 36000,
    )
    crc = crc16_ccitt_nibble(bytes(frame[:-2]))
    frame[-2:] = bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    return bytes(frame)


class FakeGcuSocket:
    def __init__(self, responses: list[bytes], *, max_chunk: int | None = None) -> None:
        self.responses = list(responses)
        self.last = responses[-1]
        self.sent: list[bytes] = []
        self.buffer = b""
        self.max_chunk = max_chunk

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return None

    def settimeout(self, _timeout: float) -> None:
        return None

    def sendall(self, packet: bytes) -> None:
        self.sent.append(packet)

    def recv(self, size: int) -> bytes:
        if not self.buffer:
            self.buffer = self.responses.pop(0) if self.responses else self.last
        count = min(size, len(self.buffer))
        if self.max_chunk is not None:
            count = min(count, self.max_chunk)
        value, self.buffer = self.buffer[:count], self.buffer[count:]
        return value


class Z1ProGimbalProtocolTest(unittest.TestCase):
    def test_packet_codec_matches_official_examples(self) -> None:
        self.assertEqual(build_gcu_packet(0x00)[-2:], bytes.fromhex("FD 13"))
        self.assertEqual(
            build_gcu_packet(0x00, pitch=100, control_valid=True)[-2:],
            bytes.fromhex("E7 9F"),
        )
        self.assertEqual(
            build_gcu_packet(0x00, yaw=1000, control_valid=True)[-2:],
            bytes.fromhex("DC 69"),
        )

    def test_parser_reads_official_reply_and_rejects_crc_damage(self) -> None:
        reply = parse_gcu_reply(OFFICIAL_REPLY)
        self.assertEqual(reply.command, 0x20)
        self.assertEqual(reply.status, 0)
        self.assertAlmostEqual(reply.relative_roll_deg, -8.03)
        self.assertAlmostEqual(reply.relative_tilt_deg, 0.32)
        self.assertAlmostEqual(reply.relative_pan_deg, 62.18)
        self.assertAlmostEqual(reply.absolute_roll_deg, -0.01)
        self.assertAlmostEqual(reply.absolute_pitch_deg, 9.33)
        self.assertAlmostEqual(reply.absolute_yaw_deg, 62.15)
        self.assertEqual(
            reply.angle_control_feedback(),
            {"roll": -0.01, "tilt": 9.33, "pan": 62.18},
        )
        damaged = bytearray(OFFICIAL_REPLY)
        damaged[12] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC"):
            parse_gcu_reply(bytes(damaged))

    def test_probe_reads_one_tcp_frame_across_fragmented_packets(self) -> None:
        fake = FakeGcuSocket([OFFICIAL_REPLY], max_chunk=3)
        with patch(
            "gogoguard_device_io.z1pro_gimbal.socket.create_connection",
            return_value=fake,
        ):
            reply = Z1ProGimbal().probe()
        self.assertAlmostEqual(reply.relative_pan_deg, 62.18)
        self.assertEqual(len(fake.sent), 1)

    def test_motion_and_photo_fail_closed_until_static_commissioning(self) -> None:
        gimbal = Z1ProGimbal(commissioned=False)
        self.assertFalse(gimbal.capability()["supported"])
        self.assertEqual(
            gimbal.capability()["commissioningStatus"],
            "disabled_field_acceptance_pending",
        )
        self.assertEqual(gimbal.capability()["commandHz"], 40.0)
        with self.assertRaisesRegex(RuntimeError, "not field commissioned"):
            gimbal.move_for_inspection(pan_body_deg=20.0, tilt_euler_deg=-5.0)
        with self.assertRaisesRegex(RuntimeError, "not field commissioned"):
            gimbal.trigger_native_photo()

    def test_enabled_capability_remains_explicitly_pending_field_acceptance(self) -> None:
        capability = Z1ProGimbal(commissioned=True).capability()
        self.assertTrue(capability["supported"])
        self.assertEqual(
            capability["commissioningStatus"],
            "enabled_field_acceptance_pending",
        )
        self.assertEqual(capability["moveTimeoutSec"], 3.0)
        self.assertEqual(capability["angleToleranceDeg"], 2.0)
        self.assertEqual(capability["confirmationSamples"], 3)

    def test_rejects_frequency_outside_official_range(self) -> None:
        with self.assertRaisesRegex(ValueError, "30-50 Hz"):
            Z1ProGimbal(command_hz=20.0)

    def test_angle_move_repeats_official_rate_until_feedback_converges(self) -> None:
        fake = FakeGcuSocket(
            [
                reply_with_angles(0.0, 0.0, 0.0),
                reply_with_angles(0.0, -5.0, 20.0),
                reply_with_angles(0.0, -5.0, 20.0),
                reply_with_angles(0.0, -5.0, 20.0),
            ]
        )
        gimbal = Z1ProGimbal(
            commissioned=True,
            move_timeout_s=0.5,
            confirmation_samples=3,
        )
        with patch(
            "gogoguard_device_io.z1pro_gimbal.socket.create_connection",
            return_value=fake,
        ):
            reply = gimbal.move_for_inspection(
                pan_body_deg=20.0, tilt_euler_deg=-5.0
            )
        self.assertAlmostEqual(reply.relative_pan_deg, 20.0)
        self.assertGreaterEqual(len(fake.sent), 4)
        self.assertTrue(all(packet[69] == 0x10 for packet in fake.sent))

    def test_angle_move_compares_mixed_protocol_feedback_frames(self) -> None:
        fake = FakeGcuSocket(
            [
                reply_with_angles(
                    -3.0,
                    -10.0,
                    82.0,
                    absolute_roll=-3.0,
                    absolute_pitch=0.58,
                    absolute_yaw=104.0,
                )
            ]
        )
        gimbal = Z1ProGimbal(
            commissioned=True,
            move_timeout_s=0.25,
            confirmation_samples=1,
        )
        with patch(
            "gogoguard_device_io.z1pro_gimbal.socket.create_connection",
            return_value=fake,
        ):
            reply = gimbal.move_for_inspection(
                pan_body_deg=82.0,
                tilt_euler_deg=0.58,
                roll_euler_deg=-3.0,
            )
        self.assertAlmostEqual(reply.relative_tilt_deg, -10.0)
        self.assertAlmostEqual(reply.absolute_pitch_deg, 0.58)

    def test_fine_move_does_not_accept_one_degree_without_actuation(self) -> None:
        fake = FakeGcuSocket(
            [
                reply_with_angles(0.0, 0.0, 0.0),
                reply_with_angles(0.0, 0.0, 1.0),
            ]
        )
        gimbal = Z1ProGimbal(
            commissioned=True,
            move_timeout_s=0.25,
            confirmation_samples=1,
        )
        with patch(
            "gogoguard_device_io.z1pro_gimbal.socket.create_connection",
            return_value=fake,
        ):
            reply = gimbal.move_for_inspection(
                pan_body_deg=1.0,
                tilt_euler_deg=0.0,
                tolerance_deg=0.5,
            )
        self.assertAlmostEqual(reply.relative_pan_deg, 1.0)
        self.assertGreaterEqual(len(fake.sent), 2)

    def test_move_rejects_precision_looser_than_inspection_contract(self) -> None:
        gimbal = Z1ProGimbal(commissioned=True)
        with self.assertRaisesRegex(ValueError, "move tolerance"):
            gimbal.move_for_inspection(
                pan_body_deg=0.0,
                tilt_euler_deg=0.0,
                tolerance_deg=3.0,
            )

    def test_angle_move_reports_axis_errors_when_feedback_never_converges(self) -> None:
        fake = FakeGcuSocket([reply_with_angles(0.0, 0.0, 0.0)])
        gimbal = Z1ProGimbal(
            commissioned=True,
            move_timeout_s=0.25,
            confirmation_samples=2,
        )
        with patch(
            "gogoguard_device_io.z1pro_gimbal.socket.create_connection",
            return_value=fake,
        ):
            with self.assertRaisesRegex(
                TimeoutError, "tilt error 5.00deg, pan error 20.00deg"
            ):
                gimbal.move_for_inspection(
                    pan_body_deg=20.0, tilt_euler_deg=-5.0
                )
        self.assertGreaterEqual(len(fake.sent), 2)


if __name__ == "__main__":
    unittest.main()
