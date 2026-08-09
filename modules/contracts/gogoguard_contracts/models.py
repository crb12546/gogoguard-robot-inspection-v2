from __future__ import annotations

from dataclasses import asdict, dataclass, field, is_dataclass
from datetime import datetime, timezone
from enum import Enum
import math
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


class DiagnosticMode(str, Enum):
    """Resource envelope for the temporary field-development recorder."""

    DEVELOPMENT = "development"
    ACCEPTANCE = "acceptance"
    PRODUCTION = "production"


class IncidentState(str, Enum):
    CAPTURING = "capturing"
    SEALING = "sealing"
    SEALED = "sealed"
    PARTIAL = "partial"
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


@dataclass
class DiagnosticProfile:
    schema: str = "gogoguard.diagnostic_profile.v1"
    revision: int = 1
    mode: DiagnosticMode = DiagnosticMode.PRODUCTION
    pre_trigger_s: float = 15.0
    post_trigger_s: float = 5.0
    point_cloud_hz: float = 0.0
    record_camera: bool = False
    record_costmap: bool = False
    record_planner_detail: bool = False
    expires_at: str | None = None
    remaining_patrols: int | None = None
    max_incidents: int = 20
    max_storage_bytes: int = 2 * 1024 * 1024 * 1024
    updated_at: str = field(default_factory=utc_now)


@dataclass
class IncidentBundle:
    schema: str = "gogoguard.incident_bundle.v1"
    incident_id: str = ""
    state: IncidentState = IncidentState.CAPTURING
    trigger: str = "manual"
    triggered_at: str = field(default_factory=utc_now)
    started_at: str | None = None
    ended_at: str | None = None
    site_id: str = ""
    robot_id: str = ""
    sensor_id: str = ""
    map_version: str | None = None
    route_id: str | None = None
    navigation_profile_revision: int | None = None
    diagnostic_profile_revision: int = 1
    diagnostic_mode: DiagnosticMode = DiagnosticMode.PRODUCTION
    root: str = ""
    files: list[dict[str, Any]] = field(default_factory=list)
    evidence_present: list[str] = field(default_factory=list)
    evidence_missing: list[str] = field(default_factory=list)
    summary: dict[str, Any] = field(default_factory=dict)
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
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value
