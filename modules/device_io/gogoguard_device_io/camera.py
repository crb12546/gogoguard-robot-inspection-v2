from __future__ import annotations

import json
import subprocess
from urllib.request import urlopen

from gogoguard_contracts import CameraStreamStatus


class CameraGateway:
    """Translate the local media gateway status into a product contract."""

    def __init__(self, mode: str, config: dict | None) -> None:
        self.mode = mode
        self.config = config or {}

    def status(self) -> CameraStreamStatus:
        source = str(self.config.get("source", "z1pro"))
        stream_path = str(self.config.get("stream_path", ""))
        port = int(self.config.get("webrtc_port", 0))
        enabled = self.mode == "robot" and bool(self.config.get("enabled", False))
        base = CameraStreamStatus(
            source=source,
            enabled=enabled,
            protocol="webrtc",
            stream_path=stream_path,
            port=port,
            message="演示模式不连接实体相机" if self.mode == "demo" else "相机直播未配置",
        )
        if not enabled:
            return base

        status_url = str(self.config.get("status_url", ""))
        if not status_url:
            base.message = "相机网关状态地址未配置"
            return base

        try:
            with urlopen(status_url, timeout=0.6) as response:
                payload = json.load(response)
        except Exception as exc:
            base.message = f"相机网关不可用: {exc}"
            return base

        track = next(
            (item for item in payload.get("tracks2", []) if item.get("codec") == "H264"),
            {},
        )
        props = track.get("codecProps", {})
        base.online = bool(payload.get("online", False))
        base.ready = bool(payload.get("ready", False)) and bool(track)
        base.width = _optional_int(props.get("width"))
        base.height = _optional_int(props.get("height"))
        base.profile = _optional_text(props.get("profile"))
        base.level = _optional_text(props.get("level"))
        base.message = "Z1Pro 实时画面已连接" if base.ready else "等待浏览器连接 Z1Pro"
        return base

    def capture_jpeg(self) -> bytes:
        """Capture one configuration sample without involving the platform."""
        if self.mode == "demo":
            # Small deterministic JPEG used only by the offline UI/contract harness.
            return bytes.fromhex(
                "ffd8ffe000104a46494600010100000100010000ffdb004300"
                + "08" * 64
                + "ffc0000b080001000101011100ffc40014000100000000000000000000000000000000"
                + "ffda0008010100003f00ffd9"
            )
        if not bool(self.config.get("enabled", False)):
            raise RuntimeError("Z1Pro camera is not enabled")
        source = str(
            self.config.get("capture_url")
            or self.config.get("rtsp_url")
            or "rtsp://192.168.144.108/"
        )
        timeout_s = float(self.config.get("capture_timeout_s") or 8.0)
        command = [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-rtsp_transport", "tcp", "-i", source,
            "-frames:v", "1", "-f", "image2pipe", "-vcodec", "mjpeg", "pipe:1",
        ]
        try:
            result = subprocess.run(
                command,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout_s,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            raise RuntimeError("Z1Pro sample capture failed") from exc
        payload = bytes(result.stdout)
        if len(payload) < 4 or not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9"):
            raise RuntimeError("Z1Pro returned an invalid JPEG sample")
        return payload


def _optional_int(value) -> int | None:
    return int(value) if value is not None else None


def _optional_text(value) -> str | None:
    return str(value) if value is not None else None


def create_camera_gateway(mode: str, config: dict | None) -> CameraGateway:
    return CameraGateway(mode, config)
