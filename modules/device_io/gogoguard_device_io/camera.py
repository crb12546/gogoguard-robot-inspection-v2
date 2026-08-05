from __future__ import annotations

import json
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


def _optional_int(value) -> int | None:
    return int(value) if value is not None else None


def _optional_text(value) -> str | None:
    return str(value) if value is not None else None


def create_camera_gateway(mode: str, config: dict | None) -> CameraGateway:
    return CameraGateway(mode, config)
