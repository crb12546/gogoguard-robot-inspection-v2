from __future__ import annotations

import unittest

from gogoguard_device_io import (
    Z1ProGimbal,
    build_gcu_packet,
    parse_gcu_reply,
)


OFFICIAL_REPLY = bytes.fromhex(
    "8A 5E 49 00 02 12 01 80 0C FE F4 01 DD FC 20 00 4A 18 FF FF A5 03 "
    "47 18 FF FF 01 00 FE FF 00 00 00 00 00 00 00 01 1F 32 29 00 00 "
    "06 17 00 00 24 F2 DF 65 16 EE AA 16 A3 A0 00 00 2B 01 14 00 00 "
    "00 00 08 00 00 20 00 EC 85"
)


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
        damaged = bytearray(OFFICIAL_REPLY)
        damaged[12] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC"):
            parse_gcu_reply(bytes(damaged))

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

    def test_rejects_frequency_outside_official_range(self) -> None:
        with self.assertRaisesRegex(ValueError, "30-50 Hz"):
            Z1ProGimbal(command_hz=20.0)


if __name__ == "__main__":
    unittest.main()
