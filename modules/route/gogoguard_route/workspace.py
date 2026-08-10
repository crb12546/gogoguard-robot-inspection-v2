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


SAFE_MAP_ID = re.compile(r"^map-[A-Za-z0-9]{12}$")
WORKSPACE_SCHEMA = "gogoguard.navigation_workspace.v1"
DEFAULT_ROBOT_RADIUS_M = 0.48
MAX_ROUTE_POINTS = 5000
MAX_BOUNDARY_POINTS = 1000


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
    if require_ready and allowed_area is None:
        raise NavigationWorkspaceError("draw and save the green allowed area before publishing")
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
    normalized = {
        "schema": WORKSPACE_SCHEMA,
        "mapJobId": map_job_id,
        "mapVersion": str(payload.get("mapVersion") or map_job_id),
        "revision": max(0, int(payload.get("revision") or 0)),
        "routeSource": (
            "recorded" if payload.get("routeSource") == "recorded" else "edited"
        ),
        "route": route,
        "allowedArea": allowed_area,
        "robotRadiusM": radius,
        "sourceMapSha256": str(payload.get("sourceMapSha256") or ""),
        "updatedAt": str(payload.get("updatedAt") or _utc_now()),
    }
    normalized["ready"] = allowed_area is not None
    normalized["workspaceHash"] = _canonical_hash(
        {key: value for key, value in normalized.items() if key != "workspaceHash"}
    )
    return normalized


class NavigationWorkspaceStore:
    """Version editable route data beside, never inside, immutable GLIM artifacts."""

    def __init__(self, data_root: Path) -> None:
        self.data_root = Path(data_root)

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

    def get(self, job_id: str) -> dict[str, Any]:
        path = self._path(job_id)
        if not path.is_file():
            # Early V6 worktrees wrote this mutable operator state into the
            # immutable artifact directory. Read such a file if one exists,
            # but every subsequent update is written to the job root.
            path = self._legacy_path(job_id)
        if path.is_file():
            value = validate_workspace(json.loads(path.read_text(encoding="utf-8")))
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
        route = _route_points(artifact.get("trajectory") or [])
        return validate_workspace(
            {
                "schema": WORKSPACE_SCHEMA,
                "mapJobId": job_id,
                "mapVersion": job_id,
                "revision": 0,
                "routeSource": "recorded",
                "route": route,
                "allowedArea": None,
                "robotRadiusM": DEFAULT_ROBOT_RADIUS_M,
                "sourceMapSha256": _file_hash(map_json),
                "updatedAt": _utc_now(),
            }
        )

    def update(self, job_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        current = self.get(job_id)
        value = dict(payload)
        value.update(
            {
                "schema": WORKSPACE_SCHEMA,
                "mapJobId": job_id,
                "mapVersion": job_id,
                "revision": int(current.get("revision") or 0) + 1,
                "sourceMapSha256": current["sourceMapSha256"],
                "updatedAt": _utc_now(),
            }
        )
        normalized = validate_workspace(value)
        _atomic_json(self._path(job_id), normalized)
        return normalized


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
