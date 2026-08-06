from __future__ import annotations

from dataclasses import asdict, dataclass, field, is_dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class RecordingState(str, Enum):
    IDLE = "idle"
    RECORDING = "recording"
    SEALING = "sealing"
    SEALED = "sealed"
    FAILED = "failed"


class MapJobState(str, Enum):
    QUEUED = "queued"
    UPLOADING = "uploading"
    PROCESSING = "processing"
    ARTIFACTS = "artifacts"
    COMPLETE = "complete"
    FAILED = "failed"


@dataclass
class DeviceStatus:
    schema: str = "gogoguard.device_status.v1"
    robot_id: str = "go2-unconfigured"
    mode: str = "demo"
    online: bool = False
    lidar_hz: float = 0.0
    imu_hz: float = 0.0
    odometry_hz: float = 0.0
    lidar_age_s: float | None = None
    imu_age_s: float | None = None
    disk_free_bytes: int = 0
    message: str = "waiting for sensor data"
    observed_at: str = field(default_factory=utc_now)


@dataclass
class CameraStreamStatus:
    schema: str = "gogoguard.camera_stream_status.v1"
    source: str = "unconfigured"
    enabled: bool = False
    online: bool = False
    ready: bool = False
    protocol: str = "webrtc"
    stream_path: str = ""
    port: int = 0
    width: int | None = None
    height: int | None = None
    profile: str | None = None
    level: str | None = None
    message: str = "camera stream is not configured"
    observed_at: str = field(default_factory=utc_now)


@dataclass
class SensorSnapshot:
    schema: str = "gogoguard.sensor_snapshot.v1"
    sequence: int = 0
    captured_at: str = field(default_factory=utc_now)
    pose: dict[str, float] = field(
        default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    )
    trajectory: list[list[float]] = field(default_factory=list)
    points: list[list[float]] = field(default_factory=list)
    device: DeviceStatus = field(default_factory=DeviceStatus)


@dataclass
class RecordingSession:
    schema: str = "gogoguard.recording_session.v1"
    session_id: str = ""
    site_id: str = ""
    robot_id: str = ""
    state: RecordingState = RecordingState.IDLE
    started_at: str | None = None
    stopped_at: str | None = None
    root: str = ""
    sample_count: int = 0
    error: str | None = None
    bundle_manifest: str | None = None
    map_job_id: str | None = None


@dataclass
class MapJob:
    schema: str = "gogoguard.map_job.v1"
    job_id: str = ""
    session_id: str = ""
    state: MapJobState = MapJobState.QUEUED
    progress: int = 0
    stage: str = "queued"
    message: str = "queued"
    created_at: str = field(default_factory=utc_now)
    updated_at: str = field(default_factory=utc_now)
    artifact_root: str | None = None
    overview_url: str | None = None
    point_cloud_url: str | None = None
    metrics: dict[str, Any] = field(default_factory=dict)
    bytes_transferred: int = 0
    bytes_total: int = 0
    transfer_rate_bps: float = 0.0
    error: str | None = None


def json_ready(value: Any) -> Any:
    if is_dataclass(value):
        return json_ready(asdict(value))
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(item) for item in value]
    return value
