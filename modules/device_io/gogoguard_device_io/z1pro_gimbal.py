from __future__ import annotations

import math
import socket
import struct
from dataclasses import dataclass


TX_HEADER = b"\xA8\xE5"
RX_HEADER = b"\x8A\x5E"


def crc16_ccitt_nibble(data: bytes) -> int:
    table = (
        0x0000, 0x1021, 0x2042, 0x3063,
        0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B,
        0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    )
    crc = 0
    for byte in data:
        crc = ((crc << 4) & 0xFFFF) ^ table[((crc >> 12) ^ (byte >> 4)) & 0x0F]
        crc = ((crc << 4) & 0xFFFF) ^ table[((crc >> 12) ^ byte) & 0x0F]
    return crc


def build_gcu_packet(
    command: int,
    params: bytes = b"",
    *,
    roll: int = 0,
    pitch: int = 0,
    yaw: int = 0,
    control_valid: bool = False,
) -> bytes:
    main = bytearray(32)
    struct.pack_into("<hhh", main, 0, roll, pitch, yaw)
    if control_valid:
        main[6] |= 0x04
    main[25] = 0x01
    payload = bytearray(TX_HEADER)
    payload += struct.pack("<H", 72 + len(params))
    payload += b"\x02"
    payload += main
    payload += bytes(32)
    payload += bytes([command & 0xFF])
    payload += params
    crc = crc16_ccitt_nibble(bytes(payload))
    payload += bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    return bytes(payload)


@dataclass(frozen=True)
class Z1ProGimbalReply:
    command: int | None
    status: int | None
    relative_roll_deg: float | None
    relative_tilt_deg: float | None
    relative_pan_deg: float | None


def parse_gcu_reply(data: bytes) -> Z1ProGimbalReply:
    if len(data) < 5 or data[:2] != RX_HEADER:
        raise ValueError("Z1Pro returned an invalid GCU response")
    declared = struct.unpack_from("<H", data, 2)[0]
    if declared < 72 or declared > len(data):
        raise ValueError("Z1Pro GCU response is truncated")
    frame = data[:declared]
    received_crc = (frame[-2] << 8) | frame[-1]
    if crc16_ccitt_nibble(frame[:-2]) != received_crc:
        raise ValueError("Z1Pro GCU response CRC is invalid")
    angles = (None, None, None)
    if len(data) >= 18:
        raw = struct.unpack_from("<hhh", data, 12)
        angles = tuple(value / 100.0 for value in raw)
    return Z1ProGimbalReply(
        command=data[69] if len(data) >= 70 else None,
        status=data[70] if len(data) >= 71 else None,
        relative_roll_deg=angles[0],
        relative_tilt_deg=angles[1],
        relative_pan_deg=angles[2],
    )


class Z1ProGimbal:
    """Commissioning-gated GCU adapter; it never moves the dog.

    In official angle-control mode 0x10, yaw is relative to the carrier while
    roll and pitch are Euler attitudes. World-absolute control needs valid
    carrier INS in every GCU frame and is deliberately not claimed here.
    """

    PAN_RANGE_DEG = (-140.0, 140.0)
    TILT_RANGE_DEG = (-110.0, 120.0)
    ROLL_RANGE_DEG = (-45.0, 45.0)

    def __init__(
        self,
        *,
        host: str = "192.168.144.108",
        port: int = 2332,
        timeout_s: float = 1.5,
        command_hz: float = 40.0,
        commissioned: bool = False,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        if not 30.0 <= float(command_hz) <= 50.0:
            raise ValueError("Z1Pro command frequency must stay within the official 30-50 Hz range")
        self.command_hz = float(command_hz)
        self.commissioned = bool(commissioned)

    def capability(self) -> dict[str, object]:
        return {
            "supported": self.commissioned,
            "hardwareSupported": True,
            "commissioningStatus": "field_verified" if self.commissioned else "protocol_candidate",
            "controlMode": "gcu_angle_control_0x10",
            "panFrame": "carrier_relative",
            "tiltFrame": "euler_attitude",
            "absoluteWorld": False,
            "commandHz": self.command_hz,
            "panDeg": list(self.PAN_RANGE_DEG),
            "tiltDeg": list(self.TILT_RANGE_DEG),
            "rollDeg": list(self.ROLL_RANGE_DEG),
        }

    def probe(self) -> Z1ProGimbalReply:
        return parse_gcu_reply(self._exchange(build_gcu_packet(0x00)))

    def move_for_inspection(
        self,
        *,
        pan_body_deg: float,
        tilt_euler_deg: float,
        roll_euler_deg: float = 0.0,
    ) -> Z1ProGimbalReply:
        if not self.commissioned:
            raise RuntimeError("Z1Pro angle control is not field commissioned")
        roll = self._scaled_angle(roll_euler_deg, self.ROLL_RANGE_DEG, "roll")
        tilt = self._scaled_angle(tilt_euler_deg, self.TILT_RANGE_DEG, "tilt")
        pan = self._scaled_angle(pan_body_deg, self.PAN_RANGE_DEG, "pan")
        reply = parse_gcu_reply(
            self._exchange(
                build_gcu_packet(
                    0x10,
                    roll=roll,
                    pitch=tilt,
                    yaw=pan,
                    control_valid=True,
                )
            )
        )
        if reply.status not in {None, 0}:
            raise RuntimeError(f"Z1Pro rejected angle command with status {reply.status}")
        return reply

    def trigger_native_photo(self) -> Z1ProGimbalReply:
        if not self.commissioned:
            raise RuntimeError("Z1Pro native photo command is not field commissioned")
        return parse_gcu_reply(self._exchange(build_gcu_packet(0x20, b"\x01")))

    def _exchange(self, packet: bytes) -> bytes:
        with socket.create_connection((self.host, self.port), timeout=self.timeout_s) as client:
            client.settimeout(self.timeout_s)
            client.sendall(packet)
            response = client.recv(4096)
        if not response:
            raise TimeoutError("Z1Pro returned no GCU acknowledgement")
        return response

    @staticmethod
    def _scaled_angle(value: float, bounds: tuple[float, float], name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} angle must be numeric")
        number = float(value)
        if not math.isfinite(number) or not bounds[0] <= number <= bounds[1]:
            raise ValueError(f"{name} angle is outside the candidate Z1Pro range")
        return int(round(number * 100.0))
