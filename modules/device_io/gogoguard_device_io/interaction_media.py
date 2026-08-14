from __future__ import annotations

import audioop
import json
import os
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.request import urlopen


@dataclass(frozen=True)
class InteractionHardwareProfile:
    schema: str
    microphone_device: str
    microphone_rate_hz: int
    microphone_channels: int
    microphone_format: str
    microphone_gain: float
    speaker_rate_hz: int
    speaker_frame_ms: int
    speaker_prebuffer_ms: int
    speaker_max_buffer_ms: int
    speaker_default_volume: int
    speaker_gain: float
    camera_rtsp: str
    video_width: int
    video_height: int
    video_fps: int
    video_bitrate: int
    go2_ip: str
    go2_interface: str
    navigation_status_url: str


def load_interaction_hardware_profile(path: Path) -> InteractionHardwareProfile:
    value = json.loads(path.read_text(encoding="utf-8"))
    if value.get("schema") != "gogoguard.interaction_hardware.v1":
        raise ValueError("unsupported interaction hardware schema")
    microphone = _object(value, "microphone")
    speaker = _object(value, "speaker")
    camera = _object(value, "camera")
    go2 = _object(value, "go2")
    safety = _object(value, "safety")
    profile = InteractionHardwareProfile(
        schema=value["schema"],
        microphone_device=_text(microphone, "device"),
        microphone_rate_hz=_integer(microphone, "sample_rate_hz"),
        microphone_channels=_integer(microphone, "channels"),
        microphone_format=_text(microphone, "format"),
        microphone_gain=_number(microphone, "gain"),
        speaker_rate_hz=_integer(speaker, "sample_rate_hz"),
        speaker_frame_ms=_integer(speaker, "frame_ms"),
        speaker_prebuffer_ms=_integer(speaker, "prebuffer_ms"),
        speaker_max_buffer_ms=_integer(speaker, "max_buffer_ms"),
        speaker_default_volume=_integer(speaker, "default_volume"),
        speaker_gain=_number(speaker, "gain"),
        camera_rtsp=_text(camera, "rtsp"),
        video_width=_integer(camera, "width"),
        video_height=_integer(camera, "height"),
        video_fps=_integer(camera, "fps"),
        video_bitrate=_integer(camera, "bitrate"),
        go2_ip=_text(go2, "ip"),
        go2_interface=_text(go2, "interface"),
        navigation_status_url=_text(safety, "navigation_status_url"),
    )
    if (profile.microphone_rate_hz, profile.microphone_channels, profile.microphone_format) != (48000, 2, "S24_3LE"):
        raise ValueError("commissioned BOYA boundary must be S24_3LE stereo at 48 kHz")
    if profile.speaker_rate_hz != 48000 or profile.speaker_frame_ms != 20:
        raise ValueError("Go2 downlink boundary must be 48 kHz with 20 ms frames")
    if profile.speaker_prebuffer_ms < profile.speaker_frame_ms:
        raise ValueError("speaker prebuffer must cover at least one frame")
    if profile.speaker_max_buffer_ms < profile.speaker_prebuffer_ms:
        raise ValueError("speaker max buffer must cover the prebuffer")
    if profile.speaker_default_volume != 10:
        raise ValueError("commissioned Go2 speaker default must remain 10")
    if not 1.0 <= profile.speaker_gain <= 8.0:
        raise ValueError("speaker PCM gain must be in [1, 8]")
    if (profile.video_width, profile.video_height) != (1920, 1080):
        raise ValueError("commissioned Z1Pro profile must remain 1920x1080")
    if not 1 <= profile.video_fps <= 30:
        raise ValueError("video fps must be in 1..30")
    return profile


def _object(value: dict[str, Any], key: str) -> dict[str, Any]:
    result = value.get(key)
    if not isinstance(result, dict):
        raise ValueError(f"{key} must be an object")
    return result


def _text(value: dict[str, Any], key: str) -> str:
    result = value.get(key)
    if not isinstance(result, str) or not result.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return result


def _integer(value: dict[str, Any], key: str) -> int:
    result = value.get(key)
    if isinstance(result, bool) or not isinstance(result, int):
        raise ValueError(f"{key} must be an integer")
    return result


def _number(value: dict[str, Any], key: str) -> float:
    result = value.get(key)
    if isinstance(result, bool) or not isinstance(result, (int, float)):
        raise ValueError(f"{key} must be a number")
    return float(result)


def decode_s24_3le_stereo_to_s16_mono(data: bytes, *, gain: float) -> bytes:
    """Convert the exact BOYA hardware format to LiveKit 48 kHz mono PCM."""

    if len(data) % 6:
        raise ValueError("S24_3LE stereo input must contain complete 6-byte frames")
    if not 0.0 < gain <= 16.0:
        raise ValueError("microphone gain must be in (0, 16]")
    output = bytearray(len(data) // 3)
    target = 0
    for offset in range(0, len(data), 6):
        left = int.from_bytes(data[offset : offset + 3], "little", signed=False)
        right = int.from_bytes(data[offset + 3 : offset + 6], "little", signed=False)
        if left & 0x800000:
            left -= 1 << 24
        if right & 0x800000:
            right -= 1 << 24
        sample = round((((left + right) // 2) / 256.0) * gain)
        sample = max(-32768, min(32767, sample))
        output[target : target + 2] = int(sample).to_bytes(2, "little", signed=True)
        target += 2
    return bytes(output)


def amplify_s16_pcm(data: bytes, *, gain: float) -> bytes:
    """Apply fixed playback gain with a per-frame no-overflow peak limiter."""

    if len(data) % 2:
        raise ValueError("speaker PCM must contain complete s16 samples")
    if not 1.0 <= gain <= 8.0:
        raise ValueError("speaker PCM gain must be in [1, 8]")
    if not data or gain == 1.0:
        return data
    peak = audioop.max(data, 2)
    if peak <= 0:
        return data
    # Keep a small headroom below positive s16 full-scale. This preserves the
    # requested gain for quiet TTS and reduces it only for a frame that would
    # otherwise hard-clip or wrap.
    applied_gain = min(gain, (32767.0 * 0.98) / peak)
    return audioop.mul(data, 2, applied_gain)


def microphone_capture_command(profile: InteractionHardwareProfile) -> list[str]:
    """Use the already-commissioned FFmpeg ALSA boundary in the frozen image."""

    if (
        profile.microphone_format != "S24_3LE"
        or profile.microphone_rate_hz != 48000
        or profile.microphone_channels != 2
    ):
        raise ValueError("unsupported realtime microphone capture profile")
    return [
        "ffmpeg",
        "-nostdin",
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "alsa",
        "-acodec",
        "pcm_s24le",
        "-ac",
        str(profile.microphone_channels),
        "-ar",
        str(profile.microphone_rate_hz),
        "-i",
        profile.microphone_device,
        "-acodec",
        "pcm_s24le",
        "-f",
        "s24le",
        "pipe:1",
    ]


class SpeakerJitterBuffer:
    """Bounded 20 ms PCM queue with prebuffering and observable underflow."""

    def __init__(self, *, sample_rate_hz: int = 48000, frame_ms: int = 20, prebuffer_ms: int = 120, max_buffer_ms: int = 3000) -> None:
        if sample_rate_hz <= 0 or frame_ms <= 0:
            raise ValueError("sample rate and frame duration must be positive")
        if prebuffer_ms < frame_ms or max_buffer_ms < prebuffer_ms:
            raise ValueError("invalid speaker buffering window")
        self.frame_bytes = sample_rate_hz * 2 * frame_ms // 1000
        self.prebuffer_bytes = sample_rate_hz * 2 * prebuffer_ms // 1000
        self.max_buffer_bytes = sample_rate_hz * 2 * max_buffer_ms // 1000
        self._buffer = bytearray()
        self._lock = threading.Lock()
        self._primed = False
        self._enabled = True
        self.frames_read = 0
        self.underflow_frames = 0
        self.prebuffer_silence_frames = 0
        self.drop_events = 0
        self.dropped_bytes = 0
        self.flushes = 0
        self.maximum_queued_bytes = 0

    def append(self, pcm: bytes) -> None:
        if len(pcm) % 2:
            raise ValueError("speaker PCM must contain complete s16 samples")
        with self._lock:
            self._buffer.extend(pcm)
            overflow = len(self._buffer) - self.max_buffer_bytes
            if overflow > 0:
                overflow += overflow % 2
                del self._buffer[:overflow]
                self.drop_events += 1
                self.dropped_bytes += overflow
                self._primed = len(self._buffer) >= self.prebuffer_bytes
            self.maximum_queued_bytes = max(self.maximum_queued_bytes, len(self._buffer))

    def read_frame(self) -> bytes:
        silence = bytes(self.frame_bytes)
        with self._lock:
            self.frames_read += 1
            if not self._enabled:
                return silence
            if not self._primed:
                if len(self._buffer) < self.prebuffer_bytes:
                    self.prebuffer_silence_frames += 1
                    return silence
                self._primed = True
            if len(self._buffer) < self.frame_bytes:
                self.underflow_frames += 1
                self._primed = False
                return silence
            frame = bytes(self._buffer[: self.frame_bytes])
            del self._buffer[: self.frame_bytes]
            return frame

    def flush(self) -> None:
        with self._lock:
            self._buffer.clear()
            self._primed = False
            self.flushes += 1

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = enabled
            if not enabled:
                self._buffer.clear()
                self._primed = False

    def status(self) -> dict[str, int | bool]:
        with self._lock:
            return {
                "enabled": self._enabled,
                "primed": self._primed,
                "queuedBytes": len(self._buffer),
                "framesRead": self.frames_read,
                "underflowFrames": self.underflow_frames,
                "prebufferSilenceFrames": self.prebuffer_silence_frames,
                "dropEvents": self.drop_events,
                "droppedBytes": self.dropped_bytes,
                "flushes": self.flushes,
                "maximumQueuedBytes": self.maximum_queued_bytes,
            }


class NavigationReadOnlyGuard:
    """Fail closed unless navigation is stopped or explicitly unauthorized at zero."""

    def __init__(self, status_url: str, *, timeout_s: float = 1.0) -> None:
        self.status_url = status_url
        self.timeout_s = timeout_s

    def inspect(self) -> dict[str, Any]:
        with urlopen(self.status_url, timeout=self.timeout_s) as response:
            return self.evaluate(json.load(response))

    @staticmethod
    def evaluate(payload: dict[str, Any]) -> dict[str, Any]:
        runtime = payload.get("runtime")
        final = (payload.get("commands") or {}).get("final")
        runtime_running = bool((payload.get("runtime_process") or {}).get("running"))
        motion_bridge_running = bool((payload.get("motion_bridge") or {}).get("running"))
        zero = isinstance(final, dict) and all(
            abs(float(final.get(axis, 0.0))) < 1e-9 for axis in ("vx", "vy", "wz")
        )
        explicitly_blocked = isinstance(runtime, dict) and runtime.get("motionAuthorized") is False and zero
        fully_stopped = runtime is None and final is None and not runtime_running and not motion_bridge_running
        return {
            "safe": explicitly_blocked or fully_stopped,
            "motionAuthorized": runtime.get("motionAuthorized") if isinstance(runtime, dict) else False,
            "runtimeProcessRunning": runtime_running,
            "motionBridgeRunning": motion_bridge_running,
            "finalCommand": final,
        }


class Go2VolumeController:
    """Invoke the commissioned VUI boundary and verify the requested 0..10 value."""

    def __init__(
        self,
        executable: Path,
        interface: str,
        *,
        library_path: Path = Path("/opt/gogoguard/deps/lib"),
    ) -> None:
        self.executable = executable
        self.interface = interface
        self.library_path = library_path

    def set_and_verify(self, volume: int) -> dict[str, Any]:
        if not 0 <= volume <= 10:
            raise ValueError("Go2 VUI volume must be in 0..10")
        last_error: Exception | None = None
        environment = os.environ.copy()
        inherited_library_path = environment.get("LD_LIBRARY_PATH", "")
        environment["LD_LIBRARY_PATH"] = str(self.library_path) + (
            f":{inherited_library_path}" if inherited_library_path else ""
        )
        for attempt in range(3):
            try:
                completed = subprocess.run(
                    [str(self.executable), self.interface, "set-volume", str(volume)],
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=6,
                    env=environment,
                )
                payload = json.loads(completed.stdout.strip())
                if (
                    payload.get("observed") != volume
                    or payload.get("setCode") != 0
                    or payload.get("getCode") != 0
                ):
                    raise RuntimeError("Go2 VUI volume verification failed")
                return payload
            except (
                OSError,
                subprocess.SubprocessError,
                json.JSONDecodeError,
                RuntimeError,
            ) as exc:
                last_error = exc
                if attempt < 2:
                    time.sleep(0.25 * (attempt + 1))
        raise RuntimeError("Go2 VUI volume verification failed") from last_error
