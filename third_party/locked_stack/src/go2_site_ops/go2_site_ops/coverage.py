"""Coverage quality model for operator-guided LiDAR mapping sessions."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple


CellKey = Tuple[int, int]
Point2 = Tuple[float, float]


def _finite(value: Any, label: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("%s must be finite" % label)
    return parsed


def _distance_to_segment(point: Point2, start: Point2, end: Point2) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx = bx - ax
    dy = by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= 1.0e-12:
        return math.hypot(px - ax, py - ay)
    ratio = ((px - ax) * dx + (py - ay) * dy) / length_squared
    ratio = min(1.0, max(0.0, ratio))
    nearest = (ax + ratio * dx, ay + ratio * dy)
    return math.hypot(px - nearest[0], py - nearest[1])


def _circular_bin_distance(left: int, right: int, bin_count: int) -> int:
    raw = abs(left - right)
    return min(raw, bin_count - raw)


@dataclass
class CoverageCell:
    key: CellKey
    center: Point2
    observation_frames: int = 0
    point_count: int = 0
    ray_count: int = 0
    pass_ids: Set[str] = field(default_factory=set)
    view_bins: Set[int] = field(default_factory=set)
    heading_bins: Set[int] = field(default_factory=set)
    overlap_sum: float = 0.0
    information_sum: float = 0.0
    dynamic_sum: float = 0.0
    dynamic_observation_frames: int = 0
    last_seen_at: Optional[float] = None
    review_label: Optional[str] = None
    review_note: str = ""

    @property
    def average_overlap(self) -> float:
        if not self.observation_frames:
            return 0.0
        return self.overlap_sum / self.observation_frames

    @property
    def average_information(self) -> float:
        if not self.observation_frames:
            return 0.0
        return self.information_sum / self.observation_frames

    @property
    def average_dynamic_fraction(self) -> Optional[float]:
        if not self.dynamic_observation_frames:
            return None
        return self.dynamic_sum / self.dynamic_observation_frames

    def has_direction_diversity(self, bin_count: int = 8) -> bool:
        for left in self.view_bins:
            for right in self.view_bins:
                if _circular_bin_distance(left, right, bin_count) >= 2:
                    return True
        return False

    def grade(self, policy: "CoveragePolicy") -> str:
        if self.review_label in {"dynamic_remove", "needs_review"}:
            return "review"
        if self.observation_frames == 0:
            return "missing"
        dynamic_fraction = self.average_dynamic_fraction
        if (
            dynamic_fraction is not None
            and dynamic_fraction >= policy.maximum_dynamic_fraction
        ):
            return "review"
        if (
            len(self.pass_ids) >= policy.minimum_passes
            and self.has_direction_diversity(policy.direction_bin_count)
            and self.average_overlap >= policy.minimum_overlap
            and self.average_information >= policy.minimum_information
        ):
            return "good"
        return "weak"

    def reason(self, policy: "CoveragePolicy") -> str:
        grade = self.grade(policy)
        if grade == "missing":
            return "尚未有效观察"
        if grade == "review":
            return "疑似动态物体或已标记为需要复核"
        reasons: List[str] = []
        if len(self.pass_ids) < policy.minimum_passes:
            reasons.append("通过次数还不够")
        if not self.has_direction_diversity(policy.direction_bin_count):
            reasons.append("还需要从不同方向再看一次")
        if self.average_overlap < policy.minimum_overlap:
            reasons.append("几次看到的环境还没有稳定对齐")
        if self.average_information < policy.minimum_information:
            reasons.append("这一段可用的固定环境太少")
        return "；".join(reasons) if reasons else "这里已经采够"

    def to_dict(self, policy: "CoveragePolicy") -> Dict[str, Any]:
        return {
            "key": [self.key[0], self.key[1]],
            "center": [self.center[0], self.center[1]],
            "grade": self.grade(policy),
            "reason": self.reason(policy),
            "observationFrames": self.observation_frames,
            "pointCount": self.point_count,
            "rayCount": self.ray_count,
            "passCount": len(self.pass_ids),
            "directionCount": len(self.view_bins),
            "averageOverlap": round(self.average_overlap, 4),
            "averageInformation": round(self.average_information, 4),
            "averageDynamicFraction": (
                round(self.average_dynamic_fraction, 4)
                if self.average_dynamic_fraction is not None
                else None
            ),
            "reviewLabel": self.review_label,
            "reviewNote": self.review_note,
        }


@dataclass(frozen=True)
class CoveragePolicy:
    resolution_m: float = 2.0
    minimum_passes: int = 2
    direction_bin_count: int = 8
    minimum_overlap: float = 0.25
    minimum_information: float = 0.25
    maximum_dynamic_fraction: float = 0.35

    def __post_init__(self) -> None:
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if self.minimum_passes < 1:
            raise ValueError("minimum_passes must be positive")
        if self.direction_bin_count < 4:
            raise ValueError("direction_bin_count must be at least four")
        for name in (
            "minimum_overlap",
            "minimum_information",
            "maximum_dynamic_fraction",
        ):
            value = getattr(self, name)
            if value < 0.0 or value > 1.0:
                raise ValueError("%s must be within [0, 1]" % name)


class CoverageMap:
    def __init__(self, policy: Optional[CoveragePolicy] = None):
        self.policy = policy or CoveragePolicy()
        self.cells: Dict[CellKey, CoverageCell] = {}
        self.robot_pose: Optional[Tuple[float, float, float]] = None
        self.last_speed_mps = 0.0
        self.last_scan_at: Optional[float] = None
        self.target_route: List[Point2] = []
        self.target_margin_m = 0.0

    def point_to_key(self, point: Point2) -> CellKey:
        return (
            math.floor(point[0] / self.policy.resolution_m),
            math.floor(point[1] / self.policy.resolution_m),
        )

    def key_center(self, key: CellKey) -> Point2:
        half = self.policy.resolution_m * 0.5
        return (
            key[0] * self.policy.resolution_m + half,
            key[1] * self.policy.resolution_m + half,
        )

    def set_route_target(self, route: Sequence[Point2], margin_m: float) -> None:
        if len(route) < 2:
            raise ValueError("route target needs at least two points")
        margin = _finite(margin_m, "margin_m")
        if margin <= 0.0:
            raise ValueError("margin_m must be positive")
        points = [(_finite(x, "route x"), _finite(y, "route y")) for x, y in route]
        self.target_route = points
        self.target_margin_m = margin
        minimum_x = min(point[0] for point in points) - margin
        maximum_x = max(point[0] for point in points) + margin
        minimum_y = min(point[1] for point in points) - margin
        maximum_y = max(point[1] for point in points) + margin
        minimum_key = self.point_to_key((minimum_x, minimum_y))
        maximum_key = self.point_to_key((maximum_x, maximum_y))
        cells: Dict[CellKey, CoverageCell] = {}
        segments = list(zip(points[:-1], points[1:]))
        for x_index in range(minimum_key[0], maximum_key[0] + 1):
            for y_index in range(minimum_key[1], maximum_key[1] + 1):
                key = (x_index, y_index)
                center = self.key_center(key)
                if min(
                    _distance_to_segment(center, start, end)
                    for start, end in segments
                ) <= margin:
                    previous = self.cells.get(key)
                    cells[key] = previous or CoverageCell(key=key, center=center)
        self.cells = cells

    def record_scan(
        self,
        sensor_x: float,
        sensor_y: float,
        heading_rad: float,
        observed_points: Iterable[Point2],
        pass_id: str,
        timestamp: float,
        overlap: float,
        information: float,
        dynamic_fraction: Optional[float],
        speed_mps: float,
    ) -> int:
        if not pass_id:
            raise ValueError("pass_id is required")
        sensor = (_finite(sensor_x, "sensor_x"), _finite(sensor_y, "sensor_y"))
        heading = _finite(heading_rad, "heading_rad")
        stamp = _finite(timestamp, "timestamp")
        overlap_value = min(1.0, max(0.0, _finite(overlap, "overlap")))
        information_value = min(1.0, max(0.0, _finite(information, "information")))
        dynamic_value = None
        if dynamic_fraction is not None:
            dynamic_value = min(
                1.0,
                max(0.0, _finite(dynamic_fraction, "dynamic_fraction")),
            )
        speed = max(0.0, _finite(speed_mps, "speed_mps"))
        hit_counts: Dict[CellKey, int] = {}
        visible_keys: Set[CellKey] = set()
        for point in observed_points:
            parsed_point = (
                _finite(point[0], "point x"),
                _finite(point[1], "point y"),
            )
            key = self.point_to_key(parsed_point)
            if key in self.cells:
                hit_counts[key] = hit_counts.get(key, 0) + 1
            visible_keys.update(self._ray_keys(sensor, parsed_point))

        heading_bin = self._angle_bin(heading)
        for key in visible_keys:
            if key not in self.cells:
                continue
            cell = self.cells[key]
            view_angle = math.atan2(
                sensor[1] - cell.center[1],
                sensor[0] - cell.center[0],
            )
            cell.observation_frames += 1
            cell.point_count += hit_counts.get(key, 0)
            cell.ray_count += 1
            cell.pass_ids.add(pass_id)
            cell.view_bins.add(self._angle_bin(view_angle))
            cell.heading_bins.add(heading_bin)
            cell.overlap_sum += overlap_value
            cell.information_sum += information_value
            if dynamic_value is not None:
                cell.dynamic_sum += dynamic_value
                cell.dynamic_observation_frames += 1
            cell.last_seen_at = stamp

        self.robot_pose = (sensor[0], sensor[1], heading)
        self.last_speed_mps = speed
        self.last_scan_at = stamp
        return len(visible_keys & set(self.cells))

    def _ray_keys(self, start: Point2, end: Point2) -> Set[CellKey]:
        """Return grid cells traversed by a 2D observation ray.

        Mapping coverage is about observed space, not only cells containing a
        LiDAR return.  Without ray traversal, a clear roadway would look
        unobserved forever because its useful evidence lies on the curbs and
        walls beside it.
        """
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        step = max(self.policy.resolution_m * 0.45, 0.1)
        count = max(1, int(math.ceil(distance / step)))
        return {
            self.point_to_key(
                (
                    start[0] + (end[0] - start[0]) * index / count,
                    start[1] + (end[1] - start[1]) * index / count,
                )
            )
            for index in range(count + 1)
        }

    def _angle_bin(self, angle: float) -> int:
        normalized = angle % (2.0 * math.pi)
        width = 2.0 * math.pi / self.policy.direction_bin_count
        return int(normalized / width) % self.policy.direction_bin_count

    def mark_review(self, key: CellKey, label: Optional[str], note: str = "") -> None:
        if key not in self.cells:
            raise KeyError(key)
        allowed = {None, "stable_anchor", "dynamic_remove", "needs_review", "no_go"}
        if label not in allowed:
            raise ValueError("unsupported review label")
        self.cells[key].review_label = label
        self.cells[key].review_note = str(note)

    def summary(self) -> Dict[str, Any]:
        grades = {"missing": 0, "weak": 0, "good": 0, "review": 0}
        for cell in self.cells.values():
            grades[cell.grade(self.policy)] += 1
        total = len(self.cells)
        accepted = grades["good"]
        return {
            "totalCells": total,
            "missingCells": grades["missing"],
            "weakCells": grades["weak"],
            "goodCells": grades["good"],
            "reviewCells": grades["review"],
            "completion": round(accepted / total, 4) if total else 0.0,
            "robotPose": list(self.robot_pose) if self.robot_pose else None,
            "lastSpeedMps": self.last_speed_mps,
            "lastScanAt": self.last_scan_at,
        }

    def guidance(self) -> Dict[str, Any]:
        if not self.cells:
            return {
                "level": "blocked",
                "stage": "blocked",
                "code": "TARGET_NOT_SET",
                "message": "还没有选定本次要采集的区域，请先让负责人设置",
            }
        if self.last_speed_mps > 0.6:
            return {
                "level": "warning",
                "stage": "slow",
                "code": "SPEED_TOO_HIGH",
                "message": "请走慢一点，保持普通步行速度",
            }
        candidates = [
            cell
            for cell in self.cells.values()
            if cell.grade(self.policy) != "good"
        ]
        if not candidates:
            return {
                "level": "success",
                "stage": "complete",
                "code": "COVERAGE_COMPLETE",
                "message": "这片区域已经采够，可以结束并让系统生成地图",
            }
        target = self._nearest_candidate(candidates)
        action_path = self._action_path(target)
        recommended_heading = self._recommended_heading(target, action_path)
        if len(action_path) >= 2:
            path_heading = math.atan2(
                action_path[-1][1] - action_path[0][1],
                action_path[-1][0] - action_path[0][0],
            )
            if math.cos(path_heading - recommended_heading) < 0.0:
                action_path.reverse()
                path_heading = (path_heading + math.pi) % (2.0 * math.pi)
            recommended_heading = path_heading
        grade = target.grade(self.policy)
        if grade == "review":
            action = "这里可能有人车或树叶在动，请换个时间再沿箭头走一遍"
        elif target.observation_frames and not target.has_direction_diversity(
            self.policy.direction_bin_count
        ):
            action = "请沿箭头反向通过橙色路段"
        else:
            action = "请沿箭头走完橙色路段"
        entry_distance: Optional[float] = None
        if self.robot_pose and action_path:
            entry_distance = math.hypot(
                action_path[0][0] - self.robot_pose[0],
                action_path[0][1] - self.robot_pose[1],
            )
        approaching = (
            entry_distance is not None
            and entry_distance > max(3.0, self.policy.resolution_m * 1.5)
        )
        return {
            "level": "action",
            "stage": "approach" if approaching else "traverse",
            "code": (
                "COVERAGE_APPROACH_ENTRY"
                if approaching
                else "COVERAGE_ACTION_REQUIRED"
            ),
            "message": (
                "先走到地图上的“起点”标记"
                if approaching
                else action
            ),
            "targetCell": [target.key[0], target.key[1]],
            "targetCenter": [target.center[0], target.center[1]],
            "highlightPath": [[point[0], point[1]] for point in action_path],
            "entryPoint": list(action_path[0]) if action_path else list(target.center),
            "exitPoint": list(action_path[-1]) if action_path else list(target.center),
            "recommendedHeadingRad": recommended_heading,
            "entryDistanceM": (
                round(entry_distance, 2) if entry_distance is not None else None
            ),
            "reason": target.reason(self.policy),
        }

    def _nearest_candidate(self, candidates: Sequence[CoverageCell]) -> CoverageCell:
        if self.robot_pose:
            return min(
                candidates,
                key=lambda cell: math.hypot(
                    cell.center[0] - self.robot_pose[0],
                    cell.center[1] - self.robot_pose[1],
                ),
            )
        return min(candidates, key=lambda cell: cell.key)

    def _action_path(self, target: CoverageCell) -> List[Point2]:
        """Return one short, continuous route section instead of scattered cells."""
        if len(self.target_route) < 2:
            return [target.center]
        samples = self._sample_route(self.policy.resolution_m)
        nearest_index = min(
            range(len(samples)),
            key=lambda index: math.hypot(
                samples[index][0] - target.center[0],
                samples[index][1] - target.center[1],
            ),
        )
        incomplete = [self._route_sample_incomplete(point) for point in samples]
        start = nearest_index
        end = nearest_index
        maximum_samples = max(2, int(math.ceil(20.0 / self.policy.resolution_m)))
        while start > 0 and incomplete[start - 1] and end - start + 1 < maximum_samples:
            start -= 1
        while end + 1 < len(samples) and incomplete[end + 1] and end - start + 1 < maximum_samples:
            end += 1
        if start == end:
            if end + 1 < len(samples):
                end += 1
            elif start > 0:
                start -= 1
        return samples[start : end + 1]

    def _route_sample_incomplete(self, point: Point2) -> bool:
        radius = max(self.policy.resolution_m, self.target_margin_m * 0.55)
        nearby = [
            cell
            for cell in self.cells.values()
            if math.hypot(cell.center[0] - point[0], cell.center[1] - point[1]) <= radius
        ]
        if not nearby:
            return True
        good_fraction = sum(
            cell.grade(self.policy) == "good" for cell in nearby
        ) / len(nearby)
        return good_fraction < 0.75

    def _sample_route(self, spacing: float) -> List[Point2]:
        result = [self.target_route[0]]
        for start, end in zip(self.target_route[:-1], self.target_route[1:]):
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            count = max(1, int(math.ceil(length / spacing)))
            for index in range(1, count + 1):
                ratio = index / count
                result.append(
                    (
                        start[0] + (end[0] - start[0]) * ratio,
                        start[1] + (end[1] - start[1]) * ratio,
                    )
                )
        return result

    def _recommended_heading(
        self,
        target: CoverageCell,
        action_path: Sequence[Point2],
    ) -> float:
        if target.heading_bins and not target.has_direction_diversity(
            self.policy.direction_bin_count
        ):
            first_bin = next(iter(target.heading_bins))
            width = 2.0 * math.pi / self.policy.direction_bin_count
            return (first_bin + self.policy.direction_bin_count / 2.0) * width
        if self.robot_pose and action_path:
            nearer_first = math.hypot(
                action_path[0][0] - self.robot_pose[0],
                action_path[0][1] - self.robot_pose[1],
            ) <= math.hypot(
                action_path[-1][0] - self.robot_pose[0],
                action_path[-1][1] - self.robot_pose[1],
            )
            start, end = (
                (action_path[0], action_path[-1])
                if nearer_first
                else (action_path[-1], action_path[0])
            )
            return math.atan2(end[1] - start[1], end[0] - start[0])
        if len(action_path) >= 2:
            return math.atan2(
                action_path[-1][1] - action_path[0][1],
                action_path[-1][0] - action_path[0][0],
            )
        return 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "policy": {
                "resolutionM": self.policy.resolution_m,
                "minimumPasses": self.policy.minimum_passes,
                "directionBinCount": self.policy.direction_bin_count,
                "minimumOverlap": self.policy.minimum_overlap,
                "minimumInformation": self.policy.minimum_information,
                "maximumDynamicFraction": self.policy.maximum_dynamic_fraction,
            },
            "summary": self.summary(),
            "guidance": self.guidance(),
            "target": {
                "type": "route_corridor" if self.target_route else None,
                "route": [[point[0], point[1]] for point in self.target_route],
                "marginM": self.target_margin_m,
            },
            "cells": [
                self.cells[key].to_dict(self.policy)
                for key in sorted(self.cells)
            ],
        }

    def to_state_dict(self) -> Dict[str, Any]:
        """Serialize every accumulator needed to resume guidance exactly."""
        return {
            "schema": "go2.coverage_state.v2",
            "policy": {
                "resolutionM": self.policy.resolution_m,
                "minimumPasses": self.policy.minimum_passes,
                "directionBinCount": self.policy.direction_bin_count,
                "minimumOverlap": self.policy.minimum_overlap,
                "minimumInformation": self.policy.minimum_information,
                "maximumDynamicFraction": self.policy.maximum_dynamic_fraction,
            },
            "robotPose": list(self.robot_pose) if self.robot_pose else None,
            "lastSpeedMps": self.last_speed_mps,
            "lastScanAt": self.last_scan_at,
            "targetRoute": [[point[0], point[1]] for point in self.target_route],
            "targetMarginM": self.target_margin_m,
            "cells": [
                {
                    "key": list(cell.key),
                    "center": list(cell.center),
                    "observationFrames": cell.observation_frames,
                    "pointCount": cell.point_count,
                    "rayCount": cell.ray_count,
                    "passIds": sorted(cell.pass_ids),
                    "viewBins": sorted(cell.view_bins),
                    "headingBins": sorted(cell.heading_bins),
                    "overlapSum": cell.overlap_sum,
                    "informationSum": cell.information_sum,
                    "dynamicSum": cell.dynamic_sum,
                    "dynamicObservationFrames": cell.dynamic_observation_frames,
                    "lastSeenAt": cell.last_seen_at,
                    "reviewLabel": cell.review_label,
                    "reviewNote": cell.review_note,
                }
                for cell in (self.cells[key] for key in sorted(self.cells))
            ],
        }

    @classmethod
    def from_state_dict(cls, payload: Mapping[str, Any]) -> "CoverageMap":
        schema = payload.get("schema")
        if schema not in {"go2.coverage_state.v1", "go2.coverage_state.v2"}:
            raise ValueError("unsupported coverage state schema")
        policy_payload = payload.get("policy")
        if not isinstance(policy_payload, Mapping):
            raise ValueError("coverage state has no policy")
        policy = CoveragePolicy(
            resolution_m=_finite(policy_payload.get("resolutionM"), "resolutionM"),
            minimum_passes=int(policy_payload.get("minimumPasses")),
            direction_bin_count=int(policy_payload.get("directionBinCount")),
            minimum_overlap=_finite(policy_payload.get("minimumOverlap"), "minimumOverlap"),
            minimum_information=_finite(
                policy_payload.get("minimumInformation"), "minimumInformation"
            ),
            maximum_dynamic_fraction=_finite(
                policy_payload.get("maximumDynamicFraction"),
                "maximumDynamicFraction",
            ),
        )
        result = cls(policy)

        def point(value: Any, size: int, label: str) -> Tuple[float, ...]:
            if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
                raise ValueError("%s must be an array" % label)
            parsed = tuple(_finite(item, label) for item in value)
            if len(parsed) != size:
                raise ValueError("%s must contain %d values" % (label, size))
            return parsed

        route = payload.get("targetRoute", [])
        if not isinstance(route, list):
            raise ValueError("targetRoute must be an array")
        result.target_route = [point(item, 2, "targetRoute") for item in route]  # type: ignore[list-item]
        result.target_margin_m = _finite(payload.get("targetMarginM", 0.0), "targetMarginM")
        robot_pose = payload.get("robotPose")
        if robot_pose is not None:
            result.robot_pose = point(robot_pose, 3, "robotPose")  # type: ignore[assignment]
        result.last_speed_mps = _finite(payload.get("lastSpeedMps", 0.0), "lastSpeedMps")
        last_scan = payload.get("lastScanAt")
        result.last_scan_at = None if last_scan is None else _finite(last_scan, "lastScanAt")
        records = payload.get("cells")
        if not isinstance(records, list):
            raise ValueError("coverage cells must be an array")
        for record in records:
            if not isinstance(record, Mapping):
                raise ValueError("coverage cell must be an object")
            key_values = point(record.get("key"), 2, "cell.key")
            key = (int(key_values[0]), int(key_values[1]))
            if key_values != key:
                raise ValueError("coverage cell key must contain integers")
            if key in result.cells:
                raise ValueError("duplicate coverage cell")
            center = point(record.get("center"), 2, "cell.center")
            cell = CoverageCell(key=key, center=center)  # type: ignore[arg-type]
            cell.observation_frames = int(record.get("observationFrames", 0))
            cell.point_count = int(record.get("pointCount", 0))
            cell.ray_count = int(record.get("rayCount", 0))
            if min(cell.observation_frames, cell.point_count, cell.ray_count) < 0:
                raise ValueError("coverage counters cannot be negative")
            cell.pass_ids = {str(value) for value in record.get("passIds", [])}
            cell.view_bins = {int(value) for value in record.get("viewBins", [])}
            cell.heading_bins = {int(value) for value in record.get("headingBins", [])}
            allowed_bins = set(range(policy.direction_bin_count))
            if not cell.view_bins <= allowed_bins or not cell.heading_bins <= allowed_bins:
                raise ValueError("coverage direction bin is out of range")
            cell.overlap_sum = _finite(record.get("overlapSum", 0.0), "overlapSum")
            cell.information_sum = _finite(
                record.get("informationSum", 0.0), "informationSum"
            )
            cell.dynamic_sum = _finite(record.get("dynamicSum", 0.0), "dynamicSum")
            cell.dynamic_observation_frames = int(
                record.get(
                    "dynamicObservationFrames",
                    cell.observation_frames if schema == "go2.coverage_state.v1" else 0,
                )
            )
            if (
                cell.dynamic_observation_frames < 0
                or cell.dynamic_observation_frames > cell.observation_frames
            ):
                raise ValueError("dynamic observation count is invalid")
            last_seen = record.get("lastSeenAt")
            cell.last_seen_at = None if last_seen is None else _finite(last_seen, "lastSeenAt")
            cell.review_label = record.get("reviewLabel")
            cell.review_note = str(record.get("reviewNote", ""))
            result.cells[key] = cell
        return result
