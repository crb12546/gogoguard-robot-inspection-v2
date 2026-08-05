"""ROS-free online coverage evidence helpers for guided mapping capture."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Iterable, Optional, Sequence, Set, Tuple


Point3 = Tuple[float, float, float]
Voxel = Tuple[int, int, int]


def angle_distance(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


@dataclass(frozen=True)
class ScanQuality:
    overlap: float
    information: float
    voxel_count: int
    azimuth_bin_count: int


class ScanQualityEstimator:
    """Estimate only online guidance quality; never claim dynamic-object truth."""

    def __init__(
        self,
        *,
        voxel_size_m: float = 0.5,
        history_frames: int = 8,
        information_voxels: int = 180,
        azimuth_bins: int = 16,
    ):
        if voxel_size_m <= 0.0 or history_frames < 1 or information_voxels < 1:
            raise ValueError("invalid scan quality configuration")
        if azimuth_bins < 4:
            raise ValueError("azimuth_bins must be at least four")
        self.voxel_size_m = float(voxel_size_m)
        self.information_voxels = int(information_voxels)
        self.azimuth_bins = int(azimuth_bins)
        self.history: Deque[Set[Voxel]] = deque(maxlen=int(history_frames))

    def _voxel(self, point: Point3) -> Voxel:
        return tuple(
            int(math.floor(float(value) / self.voxel_size_m)) for value in point
        )  # type: ignore[return-value]

    def estimate(
        self,
        points: Iterable[Point3],
        *,
        sensor_xy: Sequence[float],
    ) -> ScanQuality:
        sensor_x, sensor_y = float(sensor_xy[0]), float(sensor_xy[1])
        current: Set[Voxel] = set()
        directions = set()
        width = 2.0 * math.pi / self.azimuth_bins
        for point in points:
            x, y, z = (float(value) for value in point)
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            current.add(self._voxel((x, y, z)))
            angle = math.atan2(y - sensor_y, x - sensor_x) % (2.0 * math.pi)
            directions.add(int(angle / width) % self.azimuth_bins)
        history_union: Set[Voxel] = set()
        for frame in self.history:
            history_union.update(frame)
        overlap = (
            len(current & history_union) / len(current)
            if current and history_union
            else 0.0
        )
        density_score = min(1.0, len(current) / self.information_voxels)
        direction_score = min(1.0, len(directions) / self.azimuth_bins)
        information = density_score * direction_score
        if current:
            self.history.append(current)
        return ScanQuality(
            overlap=round(overlap, 4),
            information=round(information, 4),
            voxel_count=len(current),
            azimuth_bin_count=len(directions),
        )


class PassTracker:
    """Create a new pass only after a real stop and a substantial turn.

    A curved route alone must not fabricate multiple acquisitions. Operators can
    walk continuously within a pass; after reaching an end they stop, turn, and
    the return traversal becomes the next pass.
    """

    def __init__(
        self,
        session_prefix: str,
        *,
        stopped_speed_mps: float = 0.08,
        minimum_stop_s: float = 1.5,
        minimum_turn_rad: float = math.radians(70.0),
    ):
        if not str(session_prefix).strip():
            raise ValueError("session_prefix is required")
        if stopped_speed_mps < 0.0 or minimum_stop_s <= 0.0:
            raise ValueError("invalid pass thresholds")
        self.prefix = str(session_prefix).strip()
        self.stopped_speed_mps = float(stopped_speed_mps)
        self.minimum_stop_s = float(minimum_stop_s)
        self.minimum_turn_rad = float(minimum_turn_rad)
        self.index = 1
        self.stop_started_at: Optional[float] = None
        self.heading_before_stop: Optional[float] = None

    @property
    def pass_id(self) -> str:
        return "%s-pass-%02d" % (self.prefix, self.index)

    def update(self, *, timestamp: float, heading_rad: float, speed_mps: float) -> str:
        timestamp = float(timestamp)
        heading = float(heading_rad)
        speed = max(0.0, float(speed_mps))
        if not all(math.isfinite(value) for value in (timestamp, heading, speed)):
            raise ValueError("pass sample must be finite")
        if speed <= self.stopped_speed_mps:
            if self.stop_started_at is None:
                self.stop_started_at = timestamp
                self.heading_before_stop = heading
            return self.pass_id
        if self.stop_started_at is not None:
            stopped_for = max(0.0, timestamp - self.stop_started_at)
            turned = (
                self.heading_before_stop is not None
                and angle_distance(heading, self.heading_before_stop)
                >= self.minimum_turn_rad
            )
            if stopped_for >= self.minimum_stop_s and turned:
                self.index += 1
            self.stop_started_at = None
            self.heading_before_stop = None
        return self.pass_id


class PoseSpeedEstimator:
    """Estimate planar speed from pose history when odometry twist is absent.

    FAST-LIO historically left ``Odometry.twist`` at its zero default. Using
    that field would turn every moving capture into a fake stop and fabricate
    pass IDs, so capture guidance derives speed from canonical body poses.
    """

    def __init__(self, *, window_s: float = 0.6):
        if not math.isfinite(window_s) or window_s <= 0.0:
            raise ValueError("speed window must be positive")
        self.window_s = float(window_s)
        self.samples: Deque[Tuple[float, float, float]] = deque()

    def update(self, *, timestamp: float, x: float, y: float) -> float:
        sample = (float(timestamp), float(x), float(y))
        if not all(math.isfinite(value) for value in sample):
            raise ValueError("pose speed sample must be finite")
        if self.samples and sample[0] <= self.samples[-1][0]:
            self.samples.clear()
        self.samples.append(sample)
        while (
            len(self.samples) > 2
            and sample[0] - self.samples[1][0] >= self.window_s
        ):
            self.samples.popleft()
        if len(self.samples) < 2:
            return 0.0
        first = self.samples[0]
        elapsed = sample[0] - first[0]
        if elapsed <= 1.0e-6:
            return 0.0
        return math.hypot(sample[1] - first[1], sample[2] - first[2]) / elapsed


def health_state(age_s: Optional[float], ready_s: float, fault_s: float) -> str:
    if age_s is None or not math.isfinite(age_s) or age_s < 0.0:
        return "unknown"
    if age_s <= ready_s:
        return "ready"
    if age_s <= fault_s:
        return "warning"
    return "fault"


def storage_health(free_bytes: int, warning_bytes: int, fault_bytes: int) -> str:
    if free_bytes < 0 or warning_bytes <= fault_bytes or fault_bytes < 0:
        raise ValueError("invalid storage health thresholds")
    if free_bytes <= fault_bytes:
        return "fault"
    if free_bytes <= warning_bytes:
        return "warning"
    return "ready"
