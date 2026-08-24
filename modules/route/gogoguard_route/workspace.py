from __future__ import annotations

import hashlib
import json
import math
import os
import re
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

from gogoguard_calibration import (
    MountCalibration,
    load_commissioned_mount_calibration,
)

from .pose_frames import (
    BASE_POSE_FRAME,
    LIDAR_POSE_FRAME,
    POSE_FRAMES,
    artifact_pose_frame,
    planar_trajectory_to_base,
)


SAFE_MAP_ID = re.compile(r"^map-[A-Za-z0-9]{12}$")
WORKSPACE_SCHEMA = "gogoguard.navigation_workspace.v2"
LEGACY_WORKSPACE_SCHEMA = "gogoguard.navigation_workspace.v1"
NAVIGATION_SURFACE_SCHEMA = "gogoguard.navigation_surface_edit.v1"
DEFAULT_ROBOT_RADIUS_M = 0.48
# Must remain identical to the padded hard envelope in go2_nav2_patrol.yaml.
PADDED_BASE_FOOTPRINT_M = (
    (0.50, 0.30),
    (0.50, -0.30),
    (-0.43, -0.30),
    (-0.43, 0.30),
)
PADDED_FOOTPRINT_RADIUS_M = max(math.hypot(x, y) for x, y in PADDED_BASE_FOOTPRINT_M)
DEFAULT_SURFACE_RESOLUTION_M = 0.10
DEFAULT_OBSTACLE_MIN_Z = 0.05
DEFAULT_OBSTACLE_MAX_Z = 1.80
DEFAULT_GROUND_RESOLUTION_M = 0.20
MAX_GROUND_PREVIEW_CELLS = 50000
GROUND_REFERENCE_BIN_M = 0.08
GROUND_REFERENCE_BAND_M = 0.32
GROUND_INTERPOLATION_RADIUS_M = 1.20
MAX_ROUTE_POINTS = 5000
MAX_BOUNDARY_POINTS = 1000
MAX_MANUAL_SURFACE_CELLS = 50000
DEFAULT_SENSOR_ID = "ARMCP6B0035634"


class NavigationWorkspaceError(ValueError):
    pass


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _finite(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise NavigationWorkspaceError(f"{label} must be numeric") from exc
    if not math.isfinite(number) or abs(number) > 100000.0:
        raise NavigationWorkspaceError(f"{label} must be finite and map-sized")
    return number


def _point(value: Any, label: str) -> tuple[float, float]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) < 2
    ):
        raise NavigationWorkspaceError(f"{label} must contain x/y")
    return _finite(value[0], f"{label}.x"), _finite(value[1], f"{label}.y")


def _canonical_hash(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, sort_keys=True, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _route_points(trajectory: Iterable[Any], spacing_m: float = 0.40) -> list[list[float]]:
    raw = [_point(value, "trajectory point") for value in trajectory]
    if len(raw) < 2:
        raise NavigationWorkspaceError("map trajectory needs at least two points")
    sampled = [raw[0]]
    for value in raw[1:]:
        if math.hypot(value[0] - sampled[-1][0], value[1] - sampled[-1][1]) >= spacing_m:
            sampled.append(value)
    if math.hypot(raw[-1][0] - sampled[-1][0], raw[-1][1] - sampled[-1][1]) >= 0.10:
        sampled.append(raw[-1])
    if len(sampled) < 2:
        raise NavigationWorkspaceError("map trajectory has no traversable planar route")
    return [[x, y] for x, y in sampled]


def _cross(first: tuple[float, float], second: tuple[float, float], third: tuple[float, float]) -> float:
    return (
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )


def _point_on_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> bool:
    if abs(_cross(start, end, point)) > 1.0e-8:
        return False
    return (
        min(start[0], end[0]) - 1.0e-8 <= point[0] <= max(start[0], end[0]) + 1.0e-8
        and min(start[1], end[1]) - 1.0e-8 <= point[1] <= max(start[1], end[1]) + 1.0e-8
    )


def _segments_intersect(
    first_start: tuple[float, float],
    first_end: tuple[float, float],
    second_start: tuple[float, float],
    second_end: tuple[float, float],
) -> bool:
    first_side = _cross(first_start, first_end, second_start)
    second_side = _cross(first_start, first_end, second_end)
    third_side = _cross(second_start, second_end, first_start)
    fourth_side = _cross(second_start, second_end, first_end)
    if (
        first_side * second_side < 0.0
        and third_side * fourth_side < 0.0
    ):
        return True
    return any(
        _point_on_segment(point, start, end)
        for side, point, start, end in (
            (first_side, second_start, first_start, first_end),
            (second_side, second_end, first_start, first_end),
            (third_side, first_start, second_start, second_end),
            (fourth_side, first_end, second_start, second_end),
        )
        if abs(side) <= 1.0e-8
    )


def _polygon_area(points: Sequence[tuple[float, float]]) -> float:
    return 0.5 * sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(points, points[1:] + points[:1])
    )


def _point_in_polygon(
    point: tuple[float, float], polygon: Sequence[tuple[float, float]]
) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if _point_on_segment(point, previous, current):
            return True
        crosses = (current[1] > point[1]) != (previous[1] > point[1])
        if crosses:
            intersection_x = (
                (previous[0] - current[0])
                * (point[1] - current[1])
                / (previous[1] - current[1])
                + current[0]
            )
            if point[0] < intersection_x:
                inside = not inside
        previous = current
    return inside


def _point_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    squared = dx * dx + dy * dy
    if squared <= 1.0e-18:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = max(
        0.0,
        min(
            1.0,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / squared,
        ),
    )
    return math.hypot(
        point[0] - (start[0] + ratio * dx),
        point[1] - (start[1] + ratio * dy),
    )


def _distance_to_boundary(
    point: tuple[float, float], polygon: Sequence[tuple[float, float]]
) -> float:
    return min(
        _point_segment_distance(point, start, end)
        for start, end in zip(polygon, polygon[1:] + polygon[:1])
    )


def _validate_route(values: Any) -> list[list[float]]:
    if not isinstance(values, list) or not 2 <= len(values) <= MAX_ROUTE_POINTS:
        raise NavigationWorkspaceError("route needs 2 to 5000 points")
    points = [_point(value, f"route[{index}]") for index, value in enumerate(values)]
    for index, (first, second) in enumerate(zip(points, points[1:])):
        if math.hypot(second[0] - first[0], second[1] - first[1]) < 0.02:
            raise NavigationWorkspaceError(
                f"route points {index}/{index + 1} are too close"
            )
    return [[x, y] for x, y in points]


def _validate_polygon(values: Any) -> list[list[float]]:
    if not isinstance(values, list) or not 3 <= len(values) <= MAX_BOUNDARY_POINTS:
        raise NavigationWorkspaceError("allowed area needs 3 to 1000 boundary points")
    points = [_point(value, f"allowedArea[{index}]") for index, value in enumerate(values)]
    if len(points) > 3 and math.hypot(
        points[0][0] - points[-1][0], points[0][1] - points[-1][1]
    ) < 0.02:
        points.pop()
    if len(points) < 3:
        raise NavigationWorkspaceError("allowed area has no usable area")
    edges = list(zip(points, points[1:] + points[:1]))
    for first_index, (first_start, first_end) in enumerate(edges):
        for second_index, (second_start, second_end) in enumerate(edges):
            if second_index <= first_index:
                continue
            if second_index in {first_index + 1, (first_index - 1) % len(edges)}:
                continue
            if first_index == 0 and second_index == len(edges) - 1:
                continue
            if _segments_intersect(first_start, first_end, second_start, second_end):
                raise NavigationWorkspaceError("allowed area boundary crosses itself")
    if abs(_polygon_area(points)) < 0.25:
        raise NavigationWorkspaceError("allowed area has no usable area")
    return [[x, y] for x, y in points]


def _validate_surface_cells(values: Any, label: str) -> list[list[int]]:
    if values in (None, []):
        return []
    if not isinstance(values, list) or len(values) > MAX_MANUAL_SURFACE_CELLS:
        raise NavigationWorkspaceError(
            f"{label} must contain at most {MAX_MANUAL_SURFACE_CELLS} grid cells"
        )
    result: list[list[int]] = []
    seen: set[tuple[int, int]] = set()
    for index, value in enumerate(values):
        if (
            not isinstance(value, Sequence)
            or isinstance(value, (str, bytes))
            or len(value) != 2
            or any(isinstance(item, bool) or not isinstance(item, int) for item in value)
        ):
            raise NavigationWorkspaceError(f"{label}[{index}] must contain integer x/y cells")
        cell = (int(value[0]), int(value[1]))
        if max(abs(cell[0]), abs(cell[1])) > 1000000:
            raise NavigationWorkspaceError(f"{label}[{index}] is outside the map")
        if cell not in seen:
            seen.add(cell)
            result.append([cell[0], cell[1]])
    result.sort()
    return result


def _validate_navigation_surface(
    value: Any,
    *,
    legacy_reviewed: bool,
) -> dict[str, Any]:
    raw = value if isinstance(value, dict) else {}
    resolution = _finite(
        raw.get("resolutionM", DEFAULT_SURFACE_RESOLUTION_M),
        "navigationSurface.resolutionM",
    )
    if not 0.05 <= resolution <= 0.25:
        raise NavigationWorkspaceError(
            "navigationSurface.resolutionM must be between 0.05m and 0.25m"
        )
    minimum_z = _finite(
        raw.get("obstacleMinZ", DEFAULT_OBSTACLE_MIN_Z),
        "navigationSurface.obstacleMinZ",
    )
    maximum_z = _finite(
        raw.get("obstacleMaxZ", DEFAULT_OBSTACLE_MAX_Z),
        "navigationSurface.obstacleMaxZ",
    )
    if not minimum_z < maximum_z or maximum_z - minimum_z > 5.0:
        raise NavigationWorkspaceError(
            "navigationSurface obstacle height range is invalid"
        )
    manual_blocked = _validate_surface_cells(
        raw.get("manualBlockedCells"), "navigationSurface.manualBlockedCells"
    )
    manual_clear = _validate_surface_cells(
        raw.get("manualClearCells"), "navigationSurface.manualClearCells"
    )
    clear_set = {tuple(cell) for cell in manual_clear}
    manual_blocked = [cell for cell in manual_blocked if tuple(cell) not in clear_set]
    reviewed = raw.get("reviewed", legacy_reviewed)
    if not isinstance(reviewed, bool):
        raise NavigationWorkspaceError("navigationSurface.reviewed must be boolean")
    return {
        "schema": NAVIGATION_SURFACE_SCHEMA,
        "resolutionM": resolution,
        "obstacleMinZ": minimum_z,
        "obstacleMaxZ": maximum_z,
        "manualBlockedCells": manual_blocked,
        "manualClearCells": manual_clear,
        "reviewed": reviewed,
    }


def _sample_segment(
    start: tuple[float, float], end: tuple[float, float], spacing_m: float = 0.10
) -> Iterable[tuple[float, float]]:
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, int(math.ceil(distance / spacing_m)))
    for index in range(steps + 1):
        ratio = index / steps
        yield (
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        )


def _route_pose_samples(
    route: Sequence[tuple[float, float]], spacing_m: float
) -> Iterable[tuple[float, float, float]]:
    for start, end in zip(route, route[1:]):
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        for x, y in _sample_segment(start, end, spacing_m):
            yield x, y, yaw


def _footprint_polygon(
    x: float, y: float, yaw: float
) -> list[tuple[float, float]]:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        (
            x + cosine * local_x - sine * local_y,
            y + sine * local_x + cosine * local_y,
        )
        for local_x, local_y in PADDED_BASE_FOOTPRINT_M
    ]


def _route_footprint_cells(
    route: Sequence[tuple[float, float]], resolution: float
) -> set[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    for x, y, yaw in _route_pose_samples(route, resolution / 2.0):
        cells.update(_footprint_cells(x, y, yaw, resolution))
    return cells


def _footprint_cells(
    x: float, y: float, yaw: float, resolution: float
) -> set[tuple[int, int]]:
    footprint = _footprint_polygon(x, y, yaw)
    minimum_x = math.floor(min(point[0] for point in footprint) / resolution)
    maximum_x = math.floor(max(point[0] for point in footprint) / resolution)
    minimum_y = math.floor(min(point[1] for point in footprint) / resolution)
    maximum_y = math.floor(max(point[1] for point in footprint) / resolution)
    cells: set[tuple[int, int]] = set()
    for cell_x in range(minimum_x, maximum_x + 1):
        for cell_y in range(minimum_y, maximum_y + 1):
            center = (
                (cell_x + 0.5) * resolution,
                (cell_y + 0.5) * resolution,
            )
            if _point_in_polygon(center, footprint):
                cells.add((cell_x, cell_y))
    return cells


def validate_workspace(payload: dict[str, Any], *, require_ready: bool = False) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise NavigationWorkspaceError("navigation workspace must be an object")
    map_job_id = str(payload.get("mapJobId") or "")
    if not SAFE_MAP_ID.fullmatch(map_job_id):
        raise NavigationWorkspaceError("navigation workspace has an invalid mapJobId")
    radius = _finite(payload.get("robotRadiusM", DEFAULT_ROBOT_RADIUS_M), "robotRadiusM")
    if not 0.30 <= radius <= 0.80:
        raise NavigationWorkspaceError("robotRadiusM must be between 0.30m and 0.80m")
    route = _validate_route(payload.get("route") or [])
    raw_area = payload.get("allowedArea")
    allowed_area = None if raw_area in (None, []) else _validate_polygon(raw_area)
    surface = _validate_navigation_surface(
        payload.get("navigationSurface"),
        # Existing V1 green-area workspaces remain publishable. New workspaces
        # and every edited V2 surface require the operator's explicit save.
        legacy_reviewed=(
            allowed_area is not None and not isinstance(payload.get("navigationSurface"), dict)
        ),
    )
    if require_ready and allowed_area is None:
        raise NavigationWorkspaceError("draw and save the green allowed area before publishing")
    if require_ready and not surface["reviewed"]:
        raise NavigationWorkspaceError(
            "review and save the static navigation map before publishing"
        )
    route_pose_frame = str(payload.get("routePoseFrame") or BASE_POSE_FRAME)
    if route_pose_frame != BASE_POSE_FRAME:
        raise NavigationWorkspaceError(
            "navigation workspace route must be expressed in base_link poses"
        )
    source_pose_frame = str(
        payload.get("sourceTrajectoryPoseFrame") or BASE_POSE_FRAME
    )
    if source_pose_frame not in POSE_FRAMES:
        raise NavigationWorkspaceError("source trajectory pose frame is invalid")
    if allowed_area is not None:
        polygon = [(value[0], value[1]) for value in allowed_area]
        route_values = [(value[0], value[1]) for value in route]
        sampled_route = [
            point
            for start, end in zip(route_values, route_values[1:])
            for point in _sample_segment(start, end)
        ]
        for point in sampled_route:
            if not _point_in_polygon(point, polygon):
                raise NavigationWorkspaceError("the blue route leaves the green allowed area")
        for point in sampled_route:
            if _distance_to_boundary(point, polygon) + 1.0e-6 < radius:
                raise NavigationWorkspaceError(
                    "the blue route is closer to the green boundary than the robot radius"
                )
        for x, y, yaw in _route_pose_samples(route_values, 0.10):
            if any(
                not _point_in_polygon(corner, polygon)
                for corner in _footprint_polygon(x, y, yaw)
            ):
                raise NavigationWorkspaceError(
                    "the padded rectangular robot footprint leaves the green allowed area"
                )
    normalized = {
        "schema": WORKSPACE_SCHEMA,
        "mapJobId": map_job_id,
        "mapVersion": str(payload.get("mapVersion") or map_job_id),
        "revision": max(0, int(payload.get("revision") or 0)),
        "routeSource": (
            "recorded" if payload.get("routeSource") == "recorded" else "edited"
        ),
        "routePoseFrame": route_pose_frame,
        "sourceTrajectoryPoseFrame": source_pose_frame,
        "mountCalibrationId": str(payload.get("mountCalibrationId") or ""),
        "mountCalibrationSha256": str(payload.get("mountCalibrationSha256") or ""),
        "route": route,
        "allowedArea": allowed_area,
        "robotRadiusM": radius,
        "navigationSurface": surface,
        "sourceMapSha256": str(payload.get("sourceMapSha256") or ""),
        "updatedAt": str(payload.get("updatedAt") or _utc_now()),
    }
    normalized["ready"] = allowed_area is not None and surface["reviewed"]
    normalized["workspaceHash"] = _canonical_hash(
        {key: value for key, value in normalized.items() if key != "workspaceHash"}
    )
    return normalized


class NavigationWorkspaceStore:
    """Version editable route data beside, never inside, immutable GLIM artifacts."""

    def __init__(
        self,
        data_root: Path,
        *,
        sensor_id: str = DEFAULT_SENSOR_ID,
        mount_calibration: MountCalibration | None = None,
    ) -> None:
        self.data_root = Path(data_root)
        self.sensor_id = sensor_id
        self.mount_calibration = mount_calibration or load_commissioned_mount_calibration(
            sensor_id
        )

    def _artifact_pose_frame(self, job_id: str) -> str:
        map_json = self._artifact_root(job_id) / "map.json"
        artifact = json.loads(map_json.read_text(encoding="utf-8"))
        try:
            return artifact_pose_frame(artifact, legacy=LIDAR_POSE_FRAME)
        except ValueError as exc:
            raise NavigationWorkspaceError(str(exc)) from exc

    def _normalize_route_frame(
        self, job_id: str, payload: dict[str, Any]
    ) -> dict[str, Any]:
        value = dict(payload)
        source_frame = str(
            value.get("sourceTrajectoryPoseFrame") or self._artifact_pose_frame(job_id)
        )
        raw_frame = value.get("routePoseFrame")
        route_frame = str(
            raw_frame
            or (
                source_frame
                if value.get("routeSource") == "recorded"
                else BASE_POSE_FRAME
            )
        )
        if source_frame not in POSE_FRAMES or route_frame not in POSE_FRAMES:
            raise NavigationWorkspaceError("navigation workspace pose frame is invalid")
        if route_frame == LIDAR_POSE_FRAME:
            if value.get("routeSource") != "recorded":
                raise NavigationWorkspaceError(
                    "an edited route cannot be migrated from lidar_link implicitly"
                )
            try:
                transformed = planar_trajectory_to_base(
                    [[point[0], point[1], 0.0] for point in value.get("route") or []],
                    pose_frame=LIDAR_POSE_FRAME,
                    calibration=self.mount_calibration,
                )
            except (IndexError, TypeError, ValueError) as exc:
                raise NavigationWorkspaceError(
                    "legacy recorded route pose-frame migration failed"
                ) from exc
            value["route"] = [[point[0], point[1]] for point in transformed]
        value.update(
            {
                "routePoseFrame": BASE_POSE_FRAME,
                "sourceTrajectoryPoseFrame": source_frame,
                "mountCalibrationId": self.mount_calibration.calibration_id,
                "mountCalibrationSha256": self.mount_calibration.digest,
            }
        )
        return value

    def _job_root(self, job_id: str) -> Path:
        if not SAFE_MAP_ID.fullmatch(job_id):
            raise KeyError(job_id)
        root = self.data_root / "map-jobs" / job_id
        if not root.is_dir() or not (root / "artifacts").is_dir():
            raise KeyError(job_id)
        return root

    def _artifact_root(self, job_id: str) -> Path:
        return self._job_root(job_id) / "artifacts"

    def _path(self, job_id: str) -> Path:
        return self._job_root(job_id) / "navigation-workspace.json"

    def _legacy_path(self, job_id: str) -> Path:
        return self._artifact_root(job_id) / "navigation-workspace.json"

    def path(self, job_id: str) -> Path:
        """Return the persisted editable workspace, including legacy reads."""
        current = self._path(job_id)
        return current if current.is_file() else self._legacy_path(job_id)

    def get(self, job_id: str) -> dict[str, Any]:
        path = self._path(job_id)
        if not path.is_file():
            # Early V6 worktrees wrote this mutable operator state into the
            # immutable artifact directory. Read such a file if one exists,
            # but every subsequent update is written to the job root.
            path = self._legacy_path(job_id)
        if path.is_file():
            raw = json.loads(path.read_text(encoding="utf-8"))
            value = validate_workspace(self._normalize_route_frame(job_id, raw))
            map_json = self._artifact_root(job_id) / "map.json"
            if value.get("sourceMapSha256") != _file_hash(map_json):
                raise NavigationWorkspaceError(
                    "navigation workspace belongs to a different map artifact"
                )
            return value
        return self.default(job_id)

    def default(self, job_id: str) -> dict[str, Any]:
        artifact_root = self._artifact_root(job_id)
        map_json = artifact_root / "map.json"
        artifact = json.loads(map_json.read_text(encoding="utf-8"))
        trajectory = artifact.get("trajectory") or []
        coverage = artifact.get("routeCoverage")
        if coverage is not None:
            try:
                start = coverage["startMapPose"]
                route_start_gap = float(coverage["routeStartGapSec"])
                start_error = math.dist(
                    (float(start["x"]), float(start["y"]), float(start["z"])),
                    tuple(float(value) for value in trajectory[0][:3]),
                )
            except (KeyError, TypeError, ValueError, IndexError) as exc:
                raise NavigationWorkspaceError(
                    "map recording-route coverage is invalid"
                ) from exc
            if (
                coverage.get("schema")
                != "gogoguard.recording_route_coverage.v1"
                or coverage.get("complete") is not True
                or not math.isfinite(route_start_gap)
                or not -0.5 <= route_start_gap <= 0.5
                or start_error > 1e-6
            ):
                raise NavigationWorkspaceError(
                    "map route does not cover the recording start"
                )
        try:
            source_pose_frame = artifact_pose_frame(
                artifact, legacy=LIDAR_POSE_FRAME
            )
            base_trajectory = planar_trajectory_to_base(
                trajectory,
                pose_frame=source_pose_frame,
                calibration=self.mount_calibration,
            )
        except ValueError as exc:
            raise NavigationWorkspaceError(str(exc)) from exc
        route = _route_points(base_trajectory)
        return validate_workspace(
            {
                "schema": WORKSPACE_SCHEMA,
                "mapJobId": job_id,
                "mapVersion": job_id,
                "revision": 0,
                "routeSource": "recorded",
                "routePoseFrame": BASE_POSE_FRAME,
                "sourceTrajectoryPoseFrame": source_pose_frame,
                "mountCalibrationId": self.mount_calibration.calibration_id,
                "mountCalibrationSha256": self.mount_calibration.digest,
                "route": route,
                "allowedArea": None,
                "robotRadiusM": DEFAULT_ROBOT_RADIUS_M,
                "navigationSurface": {
                    "schema": NAVIGATION_SURFACE_SCHEMA,
                    "resolutionM": DEFAULT_SURFACE_RESOLUTION_M,
                    "obstacleMinZ": DEFAULT_OBSTACLE_MIN_Z,
                    "obstacleMaxZ": DEFAULT_OBSTACLE_MAX_Z,
                    "manualBlockedCells": [],
                    "manualClearCells": [],
                    "reviewed": False,
                },
                "sourceMapSha256": _file_hash(map_json),
                "updatedAt": _utc_now(),
            }
        )

    def update(
        self,
        job_id: str,
        payload: dict[str, Any],
        *,
        require_current: bool = False,
    ) -> dict[str, Any]:
        current = self.get(job_id)
        if require_current:
            try:
                base_revision = int(payload.get("baseRevision"))
            except (TypeError, ValueError) as exc:
                raise NavigationWorkspaceError(
                    "navigation workspace page is stale; refresh before saving"
                ) from exc
            base_hash = str(payload.get("baseWorkspaceHash") or "")
            if (
                base_revision != int(current["revision"])
                or base_hash != current["workspaceHash"]
            ):
                raise NavigationWorkspaceError(
                    "navigation workspace changed after this page loaded; refresh before saving"
                )
        value = dict(payload)
        route_source = (
            "recorded" if value.get("routeSource") == "recorded" else "edited"
        )
        if route_source == "recorded" and require_current:
            # A recorded route is immutable derived evidence, not browser-owned
            # geometry.  Rebuild it from the map artifact so an old page can
            # never repost lidar_link coordinates and have them stamped as
            # base_link.
            value["route"] = self.default(job_id)["route"]
        elif require_current and value.get("routePoseFrame") != BASE_POSE_FRAME:
            raise NavigationWorkspaceError(
                "edited route save must explicitly use the base_link pose frame"
            )
        value.update(
            {
                "schema": WORKSPACE_SCHEMA,
                "mapJobId": job_id,
                "mapVersion": job_id,
                "revision": int(current.get("revision") or 0) + 1,
                "sourceMapSha256": current["sourceMapSha256"],
                "routeSource": route_source,
                "routePoseFrame": BASE_POSE_FRAME,
                "sourceTrajectoryPoseFrame": current[
                    "sourceTrajectoryPoseFrame"
                ],
                "mountCalibrationId": self.mount_calibration.calibration_id,
                "mountCalibrationSha256": self.mount_calibration.digest,
                "updatedAt": _utc_now(),
            }
        )
        normalized = validate_workspace(value)
        _atomic_json(self._path(job_id), normalized)
        return normalized


def _map_point(value: Any, label: str) -> tuple[float, float, float]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) < 3
    ):
        raise NavigationWorkspaceError(f"{label} must contain x/y/z")
    return (
        _finite(value[0], f"{label}.x"),
        _finite(value[1], f"{label}.y"),
        _finite(value[2], f"{label}.z"),
    )


def _surface_cell(x: float, y: float, resolution: float) -> tuple[int, int]:
    return math.floor(x / resolution), math.floor(y / resolution)


def _ground_preview_resolution(
    value: dict[str, Any], cloud: Sequence[tuple[float, float, float]]
) -> float:
    """Keep the operator preview detailed without returning an unbounded grid."""

    requested = max(
        DEFAULT_GROUND_RESOLUTION_M,
        float(value["navigationSurface"]["resolutionM"]) * 2.0,
    )
    polygon = value.get("allowedArea") or []
    points = [(float(point[0]), float(point[1])) for point in polygon]
    if not points:
        points = [(point[0], point[1]) for point in cloud]
    if not points:
        return requested
    span_x = max(point[0] for point in points) - min(point[0] for point in points)
    span_y = max(point[1] for point in points) - min(point[1] for point in points)
    estimated = max(1.0, span_x / requested) * max(1.0, span_y / requested)
    if estimated <= MAX_GROUND_PREVIEW_CELLS:
        return requested
    return max(
        requested,
        math.sqrt(max(span_x * span_y, 1.0) / MAX_GROUND_PREVIEW_CELLS),
    )


def _ground_reference(cell_elevations: dict[tuple[int, int], float]) -> float | None:
    """Find the dominant low horizontal layer while ignoring roofs and tall objects."""

    if not cell_elevations:
        return None
    ordered = sorted(cell_elevations.values())
    lower_half = ordered[: max(1, (len(ordered) + 1) // 2)]
    buckets: dict[int, int] = {}
    for elevation in lower_half:
        bucket = round(elevation / GROUND_REFERENCE_BIN_M)
        buckets[bucket] = buckets.get(bucket, 0) + 1
    dominant = max(
        buckets,
        key=lambda bucket: (
            buckets[bucket],
            -abs(bucket * GROUND_REFERENCE_BIN_M - lower_half[len(lower_half) // 2]),
            -bucket,
        ),
    )
    center = dominant * GROUND_REFERENCE_BIN_M
    supported = [
        elevation
        for elevation in lower_half
        if abs(elevation - center) <= GROUND_REFERENCE_BIN_M * 1.5
    ]
    supported.sort()
    return supported[len(supported) // 2] if supported else center


def _cell_ground_measurements(
    cloud: Sequence[tuple[float, float, float]], resolution: float
) -> tuple[
    dict[tuple[int, int], list[float]],
    dict[tuple[int, int], float],
]:
    samples: dict[tuple[int, int], list[float]] = {}
    for point in cloud:
        samples.setdefault(_surface_cell(point[0], point[1], resolution), []).append(
            point[2]
        )
    # The lowest return is the local ground hypothesis. Later neighborhood and
    # dominant-layer checks reject isolated low returns and upper surfaces.
    elevations = {cell: min(values) for cell, values in samples.items()}
    return samples, elevations


def _ground_target_cells(
    value: dict[str, Any],
    measured: set[tuple[int, int]],
    resolution: float,
) -> tuple[set[tuple[int, int]], str]:
    polygon = value.get("allowedArea")
    if polygon:
        polygon_xy = [(point[0], point[1]) for point in polygon]
        minimum_x = math.floor(min(point[0] for point in polygon_xy) / resolution)
        maximum_x = math.ceil(max(point[0] for point in polygon_xy) / resolution)
        minimum_y = math.floor(min(point[1] for point in polygon_xy) / resolution)
        maximum_y = math.ceil(max(point[1] for point in polygon_xy) / resolution)
        cells = {
            (cell_x, cell_y)
            for cell_x in range(minimum_x, maximum_x + 1)
            for cell_y in range(minimum_y, maximum_y + 1)
            if _point_in_polygon(
                (
                    (cell_x + 0.5) * resolution,
                    (cell_y + 0.5) * resolution,
                ),
                polygon_xy,
            )
        }
        return cells, "operator_allowed_area"

    # Before the operator draws the green boundary, show a bounded continuous
    # patch around actual floor returns. This is deliberately not a convex hull:
    # a hull could paint across a courtyard, wall or other unobserved void.
    dilation = max(1, int(math.ceil(0.60 / resolution)))
    cells: set[tuple[int, int]] = set()
    for cell_x, cell_y in measured:
        for dx in range(-dilation, dilation + 1):
            for dy in range(-dilation, dilation + 1):
                if math.hypot(dx, dy) <= dilation:
                    cells.add((cell_x + dx, cell_y + dy))
                    if len(cells) >= MAX_GROUND_PREVIEW_CELLS:
                        return cells, "measured_support_neighborhood"
    return cells, "measured_support_neighborhood"


def _navigation_ground_surface(
    value: dict[str, Any], cloud: Sequence[tuple[float, float, float]]
) -> dict[str, Any]:
    """Build a review-only 2.5D ground layer from the point-cloud lower envelope.

    It intentionally does not authorize navigation. The reviewed green polygon
    and red occupancy cells remain the Nav2 static-map authority until this
    terrain model has a separate field-acceptance receipt.
    """

    resolution = _ground_preview_resolution(value, cloud)
    samples, elevations = _cell_ground_measurements(cloud, resolution)
    reference = _ground_reference(elevations)
    if reference is None:
        return {
            "schema": "gogoguard.ground_surface_preview.v1",
            "frame": "map",
            "resolutionM": resolution,
            "referenceElevationM": None,
            "regionSource": "none",
            "navigationAuthority": False,
            "measuredCells": [],
            "inferredCells": [],
            "unknownCells": [],
            "stats": {
                "measuredCellCount": 0,
                "inferredCellCount": 0,
                "unknownCellCount": 0,
                "traversableCellCount": 0,
                "cautionCellCount": 0,
            },
        }

    raw_cells = set(elevations)
    measured: set[tuple[int, int]] = set()
    for cell, elevation in elevations.items():
        if abs(elevation - reference) > GROUND_REFERENCE_BAND_M:
            continue
        support = len(samples[cell]) >= 2 or any(
            neighbor in raw_cells
            and abs(elevations[neighbor] - elevation) <= 0.18
            for neighbor in (
                (cell[0] - 1, cell[1]),
                (cell[0] + 1, cell[1]),
                (cell[0], cell[1] - 1),
                (cell[0], cell[1] + 1),
                (cell[0] - 1, cell[1] - 1),
                (cell[0] - 1, cell[1] + 1),
                (cell[0] + 1, cell[1] - 1),
                (cell[0] + 1, cell[1] + 1),
            )
        )
        if support:
            measured.add(cell)

    targets, region_source = _ground_target_cells(value, measured, resolution)
    interpolation_cells = max(
        1, int(math.ceil(GROUND_INTERPOLATION_RADIUS_M / resolution))
    )
    measured_rows: list[list[Any]] = []
    measured_values: dict[tuple[int, int], tuple[float, float, float, str]] = {}
    for cell in sorted(measured & targets):
        elevation = elevations[cell]
        ground_band = [
            sample for sample in samples[cell] if sample <= elevation + 0.12
        ]
        mean = sum(ground_band) / len(ground_band)
        roughness = math.sqrt(
            sum((sample - mean) ** 2 for sample in ground_band) / len(ground_band)
        )
        slopes = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if not dx and not dy:
                    continue
                neighbor = (cell[0] + dx, cell[1] + dy)
                if neighbor in measured:
                    distance = math.hypot(dx, dy) * resolution
                    slopes.append(
                        math.degrees(
                            math.atan2(abs(elevations[neighbor] - elevation), distance)
                        )
                    )
        slope = max(slopes, default=0.0)
        status = "caution" if slope > 24.0 or roughness > 0.08 else "traversable"
        confidence = min(0.98, 0.72 + min(len(samples[cell]), 5) * 0.05)
        measured_values[cell] = (elevation, slope, roughness, status)
        measured_rows.append(
            [
                cell[0],
                cell[1],
                round(elevation, 3),
                round(confidence, 2),
                round(slope, 1),
                round(roughness, 3),
                status,
            ]
        )

    inferred_rows: list[list[Any]] = []
    unknown_rows: list[list[Any]] = []
    for cell in sorted(targets - measured):
        nearby: list[tuple[float, float]] = []
        for dx in range(-interpolation_cells, interpolation_cells + 1):
            for dy in range(-interpolation_cells, interpolation_cells + 1):
                neighbor = (cell[0] + dx, cell[1] + dy)
                if neighbor not in measured_values:
                    continue
                distance = math.hypot(dx, dy) * resolution
                if distance <= GROUND_INTERPOLATION_RADIUS_M:
                    nearby.append((distance, measured_values[neighbor][0]))
        nearby.sort()
        closest = nearby[:4]
        if closest:
            spread = max(value[1] for value in closest) - min(
                value[1] for value in closest
            )
            weights = [1.0 / max(value[0], resolution * 0.5) for value in closest]
            elevation = sum(
                value[1] * weight for value, weight in zip(closest, weights)
            ) / sum(weights)
            nearest_distance = closest[0][0]
            confidence = max(
                0.25,
                min(0.68, 0.68 - nearest_distance / (GROUND_INTERPOLATION_RADIUS_M * 2.5)),
            )
            status = "caution" if spread > 0.18 else "traversable"
            inferred_rows.append(
                [
                    cell[0],
                    cell[1],
                    round(elevation, 3),
                    round(confidence, 2),
                    status,
                ]
            )
        else:
            unknown_rows.append([cell[0], cell[1], round(reference, 3)])

    traversable = sum(row[-1] == "traversable" for row in measured_rows) + sum(
        row[-1] == "traversable" for row in inferred_rows
    )
    caution = len(measured_rows) + len(inferred_rows) - traversable
    return {
        "schema": "gogoguard.ground_surface_preview.v1",
        "frame": "map",
        "resolutionM": resolution,
        "referenceElevationM": round(reference, 3),
        "regionSource": region_source,
        "navigationAuthority": False,
        "cellFormats": {
            "measuredCells": [
                "cellX", "cellY", "elevationM", "confidence", "slopeDeg", "roughnessM", "status"
            ],
            "inferredCells": [
                "cellX", "cellY", "elevationM", "confidence", "status"
            ],
            "unknownCells": ["cellX", "cellY", "referenceElevationM"],
        },
        "measuredCells": measured_rows,
        "inferredCells": inferred_rows,
        "unknownCells": unknown_rows,
        "stats": {
            "measuredCellCount": len(measured_rows),
            "inferredCellCount": len(inferred_rows),
            "unknownCellCount": len(unknown_rows),
            "traversableCellCount": traversable,
            "cautionCellCount": caution,
        },
        "note": (
            "点云下包络生成的2.5D地面参考；实测、插值和未知区分开显示，"
            "未经现场验收，不直接改变Nav2通行权限"
        ),
    }


def navigation_surface_cells(
    workspace: dict[str, Any],
    map_points: Iterable[Sequence[Any]],
    *,
    include_ground: bool = False,
) -> dict[str, Any]:
    """Project one reviewed height slice into deterministic map-frame cells."""

    value = validate_workspace(workspace)
    surface = value["navigationSurface"]
    resolution = float(surface["resolutionM"])
    minimum_z = float(surface["obstacleMinZ"])
    maximum_z = float(surface["obstacleMaxZ"])
    cloud = [_map_point(point, "map point") for point in map_points]
    raw_counts: dict[tuple[int, int], int] = {}
    for point in cloud:
        if minimum_z <= point[2] <= maximum_z:
            cell = _surface_cell(point[0], point[1], resolution)
            raw_counts[cell] = raw_counts.get(cell, 0) + 1
    raw_cells = set(raw_counts)
    # GLIM maps vary from sparse editor exports to dense full-resolution PCDs.
    # Requiring a fixed point count would erase real walls from sparse maps or
    # preserve every outlier in dense maps.  Keep a cell when it has repeated
    # support, or when at least one neighboring cell supports the same surface;
    # this is a small occupancy aggregation, not semantic object recognition.
    automatic = {
        cell
        for cell, count in raw_counts.items()
        if count >= 2
        or any(
            (cell[0] + dx, cell[1] + dy) in raw_cells
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            if dx or dy
        )
    }
    rejected_isolated = len(raw_cells) - len(automatic)
    # The robot physically occupied the recorded corridor while this map was
    # captured. Clear that proven body footprint from the automatic projection
    # so lidar self-returns, nearby floor noise or a sparse wall edge cannot
    # make the recorded start pose occupied. An operator-edited route is not
    # evidence of free space and never receives this treatment.
    route_cleared: set[tuple[int, int]] = set()
    if value["routeSource"] == "recorded":
        route = [(point[0], point[1]) for point in value["route"]]
        route_cleared = _route_footprint_cells(route, resolution)
        automatic -= route_cleared
    manual_clear = {tuple(cell) for cell in surface["manualClearCells"]}
    manual_blocked = {tuple(cell) for cell in surface["manualBlockedCells"]}
    occupied = (automatic - manual_clear) | manual_blocked
    preview = {
        "schema": "gogoguard.navigation_surface_preview.v1",
        "frame": "map",
        "resolutionM": resolution,
        "obstacleMinZ": minimum_z,
        "obstacleMaxZ": maximum_z,
        "automaticCells": [list(cell) for cell in sorted(automatic)],
        "occupiedCells": [list(cell) for cell in sorted(occupied)],
        "manualBlockedCells": [list(cell) for cell in sorted(manual_blocked)],
        "manualClearCells": [list(cell) for cell in sorted(manual_clear)],
        "stats": {
            "sourcePointCount": len(cloud),
            "rawHeightSliceCellCount": len(raw_cells),
            "rejectedIsolatedCellCount": rejected_isolated,
            "automaticCellCount": len(automatic),
            "occupiedCellCount": len(occupied),
            "manualBlockedCellCount": len(manual_blocked),
            "manualClearCellCount": len(manual_clear),
            "recordedCorridorClearCellCount": len(route_cleared),
        },
    }
    if include_ground:
        preview["ground"] = _navigation_ground_surface(value, cloud)
    return preview


def _raster_bounds(
    value: dict[str, Any],
    cloud: Sequence[tuple[float, float, float]],
) -> tuple[float, float, float, float, float, int, int]:
    polygon = [(point[0], point[1]) for point in value["allowedArea"]]
    bounds_points = [(point[0], point[1]) for point in cloud] + polygon
    if not bounds_points:
        raise NavigationWorkspaceError("map has no points for navigation bounds")
    margin = max(2.0, value["robotRadiusM"] * 3.0)
    raw_minimum_x = min(point[0] for point in bounds_points) - margin
    raw_maximum_x = max(point[0] for point in bounds_points) + margin
    raw_minimum_y = min(point[1] for point in bounds_points) - margin
    raw_maximum_y = max(point[1] for point in bounds_points) + margin
    requested = float(value["navigationSurface"]["resolutionM"])
    # Snapping can add at most one cell on each side. Reserve those two cells
    # when a very large map must be coarsened to the Nav2 image-size limit.
    resolution = max(
        requested,
        (raw_maximum_x - raw_minimum_x) / 4093.0,
        (raw_maximum_y - raw_minimum_y) / 4093.0,
    )
    # The reviewed surface is indexed from map-frame (0, 0). The PGM origin
    # must use the same lattice. An arbitrary cloud-bound origin shifts cell
    # centers by a few centimetres and can put a previously cleared wall pixel
    # back inside the commissioned rectangular robot footprint.
    minimum_x = math.floor(raw_minimum_x / resolution) * resolution
    maximum_x = math.ceil(raw_maximum_x / resolution) * resolution
    minimum_y = math.floor(raw_minimum_y / resolution) * resolution
    maximum_y = math.ceil(raw_maximum_y / resolution) * resolution
    width = max(1, int(round((maximum_x - minimum_x) / resolution)))
    height = max(1, int(round((maximum_y - minimum_y) / resolution)))
    if width > 4095 or height > 4095:
        raise NavigationWorkspaceError("navigation raster exceeds the supported size")
    # Reconstruct the upper edges from the serialized origin, dimensions and
    # resolution. This is exactly how map_server interprets the PGM/YAML pair.
    maximum_x = minimum_x + width * resolution
    maximum_y = minimum_y + height * resolution
    return minimum_x, maximum_x, minimum_y, maximum_y, resolution, width, height


def _raster_cell(
    x: float,
    y: float,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> tuple[int, int]:
    return (
        math.floor((x - origin_x) / resolution),
        math.floor((y - origin_y) / resolution),
    )


def _raster_cell_center(
    cell: tuple[int, int],
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> tuple[float, float]:
    return (
        origin_x + (cell[0] + 0.5) * resolution,
        origin_y + (cell[1] + 0.5) * resolution,
    )


def _raster_footprint_cells(
    x: float,
    y: float,
    yaw: float,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> set[tuple[int, int]]:
    footprint = _footprint_polygon(x, y, yaw)
    minimum = _raster_cell(
        min(point[0] for point in footprint),
        min(point[1] for point in footprint),
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
    )
    maximum = _raster_cell(
        max(point[0] for point in footprint),
        max(point[1] for point in footprint),
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
    )
    cells: set[tuple[int, int]] = set()
    for cell_x in range(minimum[0], maximum[0] + 1):
        for cell_y in range(minimum[1], maximum[1] + 1):
            center = _raster_cell_center(
                (cell_x, cell_y),
                origin_x=origin_x,
                origin_y=origin_y,
                resolution=resolution,
            )
            if _point_in_polygon(center, footprint):
                cells.add((cell_x, cell_y))
    return cells


def _raster_cell_is_free(
    cell: tuple[int, int],
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    polygon: Sequence[tuple[float, float]],
    occupied: set[tuple[int, int]],
    source_resolution: float,
) -> bool:
    if not (0 <= cell[0] < width and 0 <= cell[1] < height):
        return False
    world_x, world_y = _raster_cell_center(
        cell,
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
    )
    return _point_in_polygon((world_x, world_y), polygon) and _surface_cell(
        world_x, world_y, source_resolution
    ) not in occupied


def write_static_navigation_map(
    workspace: dict[str, Any],
    map_points: Iterable[Sequence[Any]],
    output_root: Path,
) -> tuple[Path, Path, dict[str, Any]]:
    """Build the Nav2 StaticLayer map without modifying the localization PCD."""

    value = validate_workspace(workspace, require_ready=True)
    cloud = [_map_point(point, "map point") for point in map_points]
    polygon = [(point[0], point[1]) for point in value["allowedArea"]]
    (
        minimum_x,
        _maximum_x,
        minimum_y,
        _maximum_y,
        resolution,
        width,
        height,
    ) = _raster_bounds(value, cloud)
    surface = navigation_surface_cells(value, cloud)
    occupied = {tuple(cell) for cell in surface["occupiedCells"]}
    route = [(point[0], point[1]) for point in value["route"]]
    footprint_cells = _route_footprint_cells(
        route, float(value["navigationSurface"]["resolutionM"])
    )
    conflicts = occupied & footprint_cells
    if conflicts:
        raise NavigationWorkspaceError(
            "the padded rectangular robot footprint intersects the reviewed static map"
        )
    pixels = bytearray(width * height)
    source_resolution = float(value["navigationSurface"]["resolutionM"])
    for cell_y in range(height):
        row = height - 1 - cell_y
        offset = row * width
        for cell_x in range(width):
            free = _raster_cell_is_free(
                (cell_x, cell_y),
                origin_x=minimum_x,
                origin_y=minimum_y,
                resolution=resolution,
                width=width,
                height=height,
                polygon=polygon,
                occupied=occupied,
                source_resolution=source_resolution,
            )
            pixels[offset + cell_x] = 255 if free else 0

    # Validate the exact top-down byte raster that map_server will consume,
    # after every origin, resolution and source-cell conversion has happened.
    # Pre-raster validation alone cannot catch quantization or row-origin drift.
    raster_conflicts: set[tuple[int, int]] = set()
    for x, y, yaw in _route_pose_samples(route, resolution / 2.0):
        for cell_x, cell_y in _raster_footprint_cells(
            x,
            y,
            yaw,
            origin_x=minimum_x,
            origin_y=minimum_y,
            resolution=resolution,
        ):
            if not (0 <= cell_x < width and 0 <= cell_y < height):
                raster_conflicts.add((cell_x, cell_y))
                continue
            row = height - 1 - cell_y
            if pixels[row * width + cell_x] == 0:
                raster_conflicts.add((cell_x, cell_y))
    if raster_conflicts:
        raise NavigationWorkspaceError(
            "the padded rectangular robot footprint intersects the final static-map raster"
        )
    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    pgm_path = output_root / "navigation-map.pgm"
    yaml_path = output_root / "navigation-map.yaml"
    temporary = pgm_path.with_suffix(".pgm.tmp")
    with temporary.open("wb") as handle:
        handle.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        handle.write(pixels)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, pgm_path)
    yaml_text = (
        "image: navigation-map.pgm\n"
        f"resolution: {resolution:.9f}\n"
        f"origin: [{minimum_x:.9f}, {minimum_y:.9f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
    )
    yaml_temporary = yaml_path.with_suffix(".yaml.tmp")
    yaml_temporary.write_text(yaml_text, encoding="utf-8")
    os.replace(yaml_temporary, yaml_path)
    metadata = {
        "schema": "gogoguard.static_navigation_map.v1",
        "frame": "map",
        "routePoseFrame": value["routePoseFrame"],
        "paddedBaseFootprintM": [list(point) for point in PADDED_BASE_FOOTPRINT_M],
        "sourceTrajectoryPoseFrame": value["sourceTrajectoryPoseFrame"],
        "mountCalibrationId": value["mountCalibrationId"],
        "mountCalibrationSha256": value["mountCalibrationSha256"],
        "width": width,
        "height": height,
        "resolutionM": resolution,
        "origin": [minimum_x, minimum_y, 0.0],
        "obstacleMinZ": value["navigationSurface"]["obstacleMinZ"],
        "obstacleMaxZ": value["navigationSurface"]["obstacleMaxZ"],
        "automaticCellCount": surface["stats"]["automaticCellCount"],
        "occupiedCellCount": surface["stats"]["occupiedCellCount"],
        "recordedCorridorClearCellCount": surface["stats"][
            "recordedCorridorClearCellCount"
        ],
        "routeFootprintConflictCellCount": 0,
        "routeFootprintRasterValidated": True,
        "rasterOriginAligned": True,
        "workspaceHash": value["workspaceHash"],
    }
    _atomic_json(output_root / "navigation-map.json", metadata)
    return pgm_path, yaml_path, metadata


def plan_navigation_preview(
    workspace: dict[str, Any],
    map_points: Iterable[Sequence[Any]],
    start: Sequence[Any],
    goal: Sequence[Any],
) -> dict[str, Any]:
    """Check static-map connectivity with the same body-radius assumption as Nav2."""

    value = validate_workspace(workspace)
    if value["allowedArea"] is None:
        raise NavigationWorkspaceError(
            "draw the green allowed area before previewing a route"
        )
    start_xy = _point(start, "start")
    goal_xy = _point(goal, "goal")
    polygon = [(point[0], point[1]) for point in value["allowedArea"]]
    for label, point in (("start", start_xy), ("goal", goal_xy)):
        if not _point_in_polygon(point, polygon):
            raise NavigationWorkspaceError(f"preview {label} is outside the green area")
    cloud = [_map_point(point, "map point") for point in map_points]
    surface = navigation_surface_cells(value, cloud)
    source_resolution = float(surface["resolutionM"])
    occupied = {tuple(cell) for cell in surface["occupiedCells"]}
    (
        minimum_x,
        _maximum_x,
        minimum_y,
        _maximum_y,
        resolution,
        width,
        height,
    ) = _raster_bounds(value, cloud)
    start_cell = _raster_cell(
        *start_xy,
        origin_x=minimum_x,
        origin_y=minimum_y,
        resolution=resolution,
    )
    goal_cell = _raster_cell(
        *goal_xy,
        origin_x=minimum_x,
        origin_y=minimum_y,
        resolution=resolution,
    )
    directions = (
        (-1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
        (1, 0),
        (1, -1),
        (0, -1),
        (-1, -1),
    )
    headings = tuple(math.atan2(dy, dx) for dx, dy in directions)
    traversable_cache: dict[tuple[int, int, int], bool] = {}

    def traversable(cell: tuple[int, int], heading: int) -> bool:
        key = (cell[0], cell[1], heading)
        if key in traversable_cache:
            return traversable_cache[key]
        world_x, world_y = _raster_cell_center(
            cell,
            origin_x=minimum_x,
            origin_y=minimum_y,
            resolution=resolution,
        )
        footprint = _footprint_polygon(world_x, world_y, headings[heading])
        footprint_cells = _raster_footprint_cells(
            world_x,
            world_y,
            headings[heading],
            origin_x=minimum_x,
            origin_y=minimum_y,
            resolution=resolution,
        )
        result = all(_point_in_polygon(corner, polygon) for corner in footprint) and all(
            _raster_cell_is_free(
                footprint_cell,
                origin_x=minimum_x,
                origin_y=minimum_y,
                resolution=resolution,
                width=width,
                height=height,
                polygon=polygon,
                occupied=occupied,
                source_resolution=source_resolution,
            )
            for footprint_cell in footprint_cells
        )
        traversable_cache[key] = result
        return result

    start_states = [
        (start_cell, heading)
        for heading in range(len(directions))
        if traversable(start_cell, heading)
    ]
    if not start_states or not any(
        traversable(goal_cell, heading) for heading in range(len(directions))
    ):
        raise NavigationWorkspaceError(
            "preview start or goal has no room for the padded robot footprint"
        )
    import heapq

    State = tuple[tuple[int, int], int]
    frontier: list[tuple[float, float, State]] = []
    parent: dict[State, State | None] = {}
    cost: dict[State, float] = {}
    for state in start_states:
        parent[state] = None
        cost[state] = 0.0
        heapq.heappush(
            frontier,
            (math.dist(start_cell, goal_cell), 0.0, state),
        )
    visited = 0
    goal_state: State | None = None
    while frontier:
        _priority, current_cost, current_state = heapq.heappop(frontier)
        if current_cost > cost.get(current_state, math.inf) + 1.0e-9:
            continue
        visited += 1
        current, current_heading = current_state
        if current == goal_cell:
            goal_state = current_state
            break
        for heading, (dx, dy) in enumerate(directions):
            neighbor = (current[0] + dx, current[1] + dy)
            if not traversable(neighbor, heading):
                continue
            # A heading change must also fit at the current center; this keeps
            # the review path from rotating the long body through a wall.
            if heading != current_heading and not traversable(current, heading):
                continue
            turn_steps = min(
                (heading - current_heading) % len(directions),
                (current_heading - heading) % len(directions),
            )
            next_cost = (
                current_cost
                + (math.sqrt(2.0) if dx and dy else 1.0)
                + turn_steps * 0.05
            )
            neighbor_state = (neighbor, heading)
            if next_cost + 1.0e-9 >= cost.get(neighbor_state, math.inf):
                continue
            cost[neighbor_state] = next_cost
            parent[neighbor_state] = current_state
            heuristic = math.hypot(goal_cell[0] - neighbor[0], goal_cell[1] - neighbor[1])
            heapq.heappush(
                frontier,
                (next_cost + heuristic, next_cost, neighbor_state),
            )
    if goal_state is None:
        return {
            "schema": "gogoguard.navigation_plan_preview.v1",
            "reachable": False,
            "reason": "STATIC_MAP_DISCONNECTED",
            "path": [],
            "visitedCellCount": visited,
        }
    states: list[State] = []
    current_state: State | None = goal_state
    while current_state is not None:
        states.append(current_state)
        current_state = parent[current_state]
    cells = [state[0] for state in states]
    cells.reverse()
    # The preview is for human review, not controller input. Keep corners and
    # every tenth cell so a long result stays legible and bounded in the UI.
    reduced: list[tuple[int, int]] = []
    previous_direction = None
    for index, cell in enumerate(cells):
        direction = None if index == 0 else (
            cell[0] - cells[index - 1][0], cell[1] - cells[index - 1][1]
        )
        if index in {0, len(cells) - 1} or direction != previous_direction or index % 10 == 0:
            reduced.append(cell)
        previous_direction = direction
    path = [[start_xy[0], start_xy[1]]]
    path.extend(
        [
            list(
                _raster_cell_center(
                    cell,
                    origin_x=minimum_x,
                    origin_y=minimum_y,
                    resolution=resolution,
                )
            )
            for cell in reduced[1:-1]
        ]
    )
    path.append([goal_xy[0], goal_xy[1]])
    length_m = sum(
        math.dist(first, second) * resolution
        for first, second in zip(cells, cells[1:])
    )
    return {
        "schema": "gogoguard.navigation_plan_preview.v1",
        "reachable": True,
        "reason": "OK",
        "path": path,
        "lengthM": round(length_m, 3),
        "visitedCellCount": visited,
        "note": "编辑器连通性预演；狗端正式路径由 Nav2 SmacPlanner2D 计算",
    }


def write_keepout_mask(
    workspace: dict[str, Any],
    map_points: Iterable[Sequence[Any]],
    output_root: Path,
    *,
    resolution_m: float = 0.10,
) -> tuple[Path, Path, dict[str, Any]]:
    """Rasterize outside-green as a standard Nav2 keepout mask."""

    value = validate_workspace(workspace, require_ready=True)
    polygon = [(point[0], point[1]) for point in value["allowedArea"]]
    cloud = [_point(point, "map point") for point in map_points]
    bounds_points = cloud + polygon
    if not bounds_points:
        raise NavigationWorkspaceError("map has no points for mask bounds")
    margin = max(2.0, value["robotRadiusM"] * 3.0)
    minimum_x = min(point[0] for point in bounds_points) - margin
    maximum_x = max(point[0] for point in bounds_points) + margin
    minimum_y = min(point[1] for point in bounds_points) - margin
    maximum_y = max(point[1] for point in bounds_points) + margin
    resolution = max(float(resolution_m), (maximum_x - minimum_x) / 4095.0, (maximum_y - minimum_y) / 4095.0)
    width = max(1, int(math.ceil((maximum_x - minimum_x) / resolution)))
    height = max(1, int(math.ceil((maximum_y - minimum_y) / resolution)))
    pixels = bytearray(width * height)
    for row in range(height):
        world_y = maximum_y - (row + 0.5) * resolution
        offset = row * width
        for column in range(width):
            world_x = minimum_x + (column + 0.5) * resolution
            # ROS map_server with negate=0 treats black (0) as occupied and
            # white (255) as free. Outside the green polygon is keepout.
            pixels[offset + column] = 255 if _point_in_polygon((world_x, world_y), polygon) else 0
    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    pgm_path = output_root / "allowed-area-mask.pgm"
    yaml_path = output_root / "allowed-area-mask.yaml"
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    temporary = pgm_path.with_suffix(".pgm.tmp")
    with temporary.open("wb") as handle:
        handle.write(header)
        handle.write(pixels)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, pgm_path)
    yaml_text = (
        "image: allowed-area-mask.pgm\n"
        f"resolution: {resolution:.9f}\n"
        f"origin: [{minimum_x:.9f}, {minimum_y:.9f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
    )
    yaml_temporary = yaml_path.with_suffix(".yaml.tmp")
    yaml_temporary.write_text(yaml_text, encoding="utf-8")
    os.replace(yaml_temporary, yaml_path)
    metadata = {
        "schema": "gogoguard.allowed_area_mask.v1",
        "frame": "map",
        "width": width,
        "height": height,
        "resolutionM": resolution,
        "origin": [minimum_x, minimum_y, 0.0],
        "robotRadiusM": value["robotRadiusM"],
        "workspaceHash": value["workspaceHash"],
    }
    _atomic_json(output_root / "allowed-area-mask.json", metadata)
    return pgm_path, yaml_path, metadata
