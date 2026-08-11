from __future__ import annotations

import math
import socket
import struct
import time
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
        move_timeout_s: float = 3.0,
        angle_tolerance_deg: float = 2.0,
        confirmation_samples: int = 3,
        commissioned: bool = False,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        if not 30.0 <= float(command_hz) <= 50.0:
            raise ValueError("Z1Pro command frequency must stay within the official 30-50 Hz range")
        self.command_hz = float(command_hz)
        if not 0.25 <= float(move_timeout_s) <= 10.0:
            raise ValueError("Z1Pro move timeout must stay within 0.25-10 seconds")
        if not 0.1 <= float(angle_tolerance_deg) <= 10.0:
            raise ValueError("Z1Pro angle tolerance must stay within 0.1-10 degrees")
        if not 1 <= int(confirmation_samples) <= 10:
            raise ValueError("Z1Pro confirmation samples must stay within 1-10")
        self.move_timeout_s = float(move_timeout_s)
        self.angle_tolerance_deg = float(angle_tolerance_deg)
        self.confirmation_samples = int(confirmation_samples)
        self.commissioned = bool(commissioned)

    def capability(self) -> dict[str, object]:
        return {
            "supported": self.commissioned,
            "hardwareSupported": True,
            "commissioningStatus": (
                "enabled_field_acceptance_pending"
                if self.commissioned
                else "disabled_field_acceptance_pending"
            ),
            "controlMode": "gcu_angle_control_0x10",
            "panFrame": "carrier_relative",
            "tiltFrame": "euler_attitude",
            "absoluteWorld": False,
            "commandHz": self.command_hz,
            "moveTimeoutSec": self.move_timeout_s,
            "angleToleranceDeg": self.angle_tolerance_deg,
            "confirmationSamples": self.confirmation_samples,
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
        packet = build_gcu_packet(
            0x10,
            roll=roll,
            pitch=tilt,
            yaw=pan,
            control_valid=True,
        )
        return self._move_until_converged(
            packet,
            roll_deg=float(roll_euler_deg),
            tilt_deg=float(tilt_euler_deg),
            pan_deg=float(pan_body_deg),
        )

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

    def _move_until_converged(
        self,
        packet: bytes,
        *,
        roll_deg: float,
        tilt_deg: float,
        pan_deg: float,
    ) -> Z1ProGimbalReply:
        deadline = time.monotonic() + self.move_timeout_s
        interval = 1.0 / self.command_hz
        next_send = time.monotonic()
        confirmed = 0
        last_reply: Z1ProGimbalReply | None = None
        with socket.create_connection((self.host, self.port), timeout=self.timeout_s) as client:
            while time.monotonic() < deadline:
                delay = next_send - time.monotonic()
                if delay > 0:
                    time.sleep(min(delay, max(0.0, deadline - time.monotonic())))
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                client.settimeout(min(self.timeout_s, max(0.05, remaining)))
                client.sendall(packet)
                try:
                    response = client.recv(4096)
                except socket.timeout:
                    next_send = max(next_send + interval, time.monotonic())
                    continue
                if not response:
                    raise TimeoutError("Z1Pro closed GCU control connection before convergence")
                reply = parse_gcu_reply(response)
                if reply.status not in {None, 0}:
                    raise RuntimeError(
                        f"Z1Pro rejected angle command with status {reply.status}"
                    )
                last_reply = reply
                errors = self._angle_errors(
                    reply,
                    roll_deg=roll_deg,
                    tilt_deg=tilt_deg,
                    pan_deg=pan_deg,
                )
                if all(error <= self.angle_tolerance_deg for error in errors.values()):
                    confirmed += 1
                    if confirmed >= self.confirmation_samples:
                        return reply
                else:
                    confirmed = 0
                next_send = max(next_send + interval, time.monotonic())
        if last_reply is None:
            raise TimeoutError("Z1Pro returned no GCU acknowledgement before move timeout")
        errors = self._angle_errors(
            last_reply,
            roll_deg=roll_deg,
            tilt_deg=tilt_deg,
            pan_deg=pan_deg,
        )
        raise TimeoutError(
            "Z1Pro angle convergence timed out: "
            + ", ".join(f"{name} error {error:.2f}deg" for name, error in errors.items())
        )

    @staticmethod
    def _angle_errors(
        reply: Z1ProGimbalReply,
        *,
        roll_deg: float,
        tilt_deg: float,
        pan_deg: float,
    ) -> dict[str, float]:
        actual = {
            "roll": reply.relative_roll_deg,
            "tilt": reply.relative_tilt_deg,
            "pan": reply.relative_pan_deg,
        }
        target = {"roll": roll_deg, "tilt": tilt_deg, "pan": pan_deg}
        if any(value is None for value in actual.values()):
            raise ValueError("Z1Pro GCU response contains no relative angle feedback")
        return {
            name: abs(float(actual[name]) - target[name])
            for name in ("roll", "tilt", "pan")
        }

    @staticmethod
    def _scaled_angle(value: float, bounds: tuple[float, float], name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} angle must be numeric")
        number = float(value)
        if not math.isfinite(number) or not bounds[0] <= number <= bounds[1]:
            raise ValueError(f"{name} angle is outside the candidate Z1Pro range")
        return int(round(number * 100.0))
