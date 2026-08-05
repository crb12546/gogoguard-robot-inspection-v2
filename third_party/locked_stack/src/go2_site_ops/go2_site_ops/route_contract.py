"""Map-frame route contracts shared by console, release pipeline and robot."""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence, Tuple


IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class RouteError(ValueError):
    pass


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Route:
    route_id: str
    waypoints: Tuple[Waypoint, ...]
    source_hash: str
    length_m: float


Point2 = Tuple[float, float]


def finite(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise RouteError("%s must be numeric" % label)
    if not math.isfinite(result):
        raise RouteError("%s must be finite" % label)
    return result


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def angle_distance(first: float, second: float) -> float:
    return abs(normalize_angle(first - second))


def parse_route(payload: Any, source_hash: str = "") -> Route:
    if not isinstance(payload, Mapping) or payload.get("schema") != "go2.route.v1":
        raise RouteError("route schema must be go2.route.v1")
    if payload.get("frame") != "map":
        raise RouteError("route frame must be map")
    route_id = str(payload.get("routeId", "")).strip()
    if not IDENTIFIER.fullmatch(route_id):
        raise RouteError("routeId must be a safe non-empty identifier")
    records = payload.get("waypoints")
    if not isinstance(records, list) or len(records) < 2:
        raise RouteError("route requires at least two waypoints")
    waypoints = []
    length_m = 0.0
    for index, record in enumerate(records):
        if not isinstance(record, Mapping):
            raise RouteError("waypoint %d must be an object" % index)
        point = Waypoint(
            finite(record.get("x"), "waypoint[%d].x" % index),
            finite(record.get("y"), "waypoint[%d].y" % index),
            normalize_angle(finite(record.get("yaw"), "waypoint[%d].yaw" % index)),
        )
        if waypoints:
            distance = math.hypot(point.x - waypoints[-1].x, point.y - waypoints[-1].y)
            if distance < 0.01:
                raise RouteError(
                    "adjacent waypoints %d/%d are closer than 1cm" % (index - 1, index)
                )
            length_m += distance
        waypoints.append(point)
    return Route(route_id, tuple(waypoints), source_hash, length_m)


def load_route(path: Path) -> Route:
    raw = Path(path).read_bytes()
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise RouteError("invalid route JSON: %s" % exc)
    return parse_route(payload, hashlib.sha256(raw).hexdigest())


def route_payload(route_id: str, points: Sequence[Sequence[Any]]) -> Mapping[str, Any]:
    if not IDENTIFIER.fullmatch(str(route_id).strip()):
        raise RouteError("routeId must be a safe non-empty identifier")
    parsed = []
    for index, point in enumerate(points):
        if not isinstance(point, Sequence) or isinstance(point, (str, bytes)) or len(point) < 2:
            raise RouteError("route point %d must contain x/y" % index)
        parsed.append((finite(point[0], "point.x"), finite(point[1], "point.y")))
    if len(parsed) < 2:
        raise RouteError("route requires at least two points")
    records = []
    for index, point in enumerate(parsed):
        other = parsed[index + 1] if index + 1 < len(parsed) else parsed[index - 1]
        if index + 1 < len(parsed):
            yaw = math.atan2(other[1] - point[1], other[0] - point[0])
        else:
            yaw = math.atan2(point[1] - other[1], point[0] - other[0])
        records.append({"x": point[0], "y": point[1], "yaw": yaw})
    payload = {
        "schema": "go2.route.v1",
        "routeId": str(route_id).strip(),
        "frame": "map",
        "waypoints": records,
    }
    parse_route(payload)
    return payload


def sample_route(route: Route, spacing_m: float) -> Tuple[Waypoint, ...]:
    spacing_m = finite(spacing_m, "path_sample_spacing_m")
    if spacing_m < 0.05 or spacing_m > 0.5:
        raise RouteError("path sample spacing must be between 0.05m and 0.5m")
    sampled = [route.waypoints[0]]
    for start, end in zip(route.waypoints, route.waypoints[1:]):
        distance = math.hypot(end.x - start.x, end.y - start.y)
        steps = max(1, int(math.ceil(distance / spacing_m)))
        yaw_delta = normalize_angle(end.yaw - start.yaw)
        for step in range(1, steps + 1):
            ratio = step / steps
            sampled.append(
                Waypoint(
                    start.x + (end.x - start.x) * ratio,
                    start.y + (end.y - start.y) * ratio,
                    normalize_angle(start.yaw + yaw_delta * ratio),
                )
            )
    return tuple(sampled)


def _segment_intersects_rectangle(
    start: Waypoint,
    end: Waypoint,
    bounds: Sequence[float],
) -> bool:
    minimum_x, minimum_y, maximum_x, maximum_y = bounds
    dx = end.x - start.x
    dy = end.y - start.y
    lower = 0.0
    upper = 1.0
    for p, q in (
        (-dx, start.x - minimum_x),
        (dx, maximum_x - start.x),
        (-dy, start.y - minimum_y),
        (dy, maximum_y - start.y),
    ):
        if abs(p) < 1.0e-12:
            if q < 0.0:
                return False
            continue
        ratio = q / p
        if p < 0.0:
            lower = max(lower, ratio)
        else:
            upper = min(upper, ratio)
        if lower > upper:
            return False
    return True


def _cross(first: Point2, second: Point2, third: Point2) -> float:
    return (
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )


def _point_on_segment(point: Point2, start: Point2, end: Point2) -> bool:
    if abs(_cross(start, end, point)) > 1.0e-9:
        return False
    return (
        min(start[0], end[0]) - 1.0e-9 <= point[0] <= max(start[0], end[0]) + 1.0e-9
        and min(start[1], end[1]) - 1.0e-9 <= point[1] <= max(start[1], end[1]) + 1.0e-9
    )


def _segments_intersect(
    first_start: Point2,
    first_end: Point2,
    second_start: Point2,
    second_end: Point2,
) -> bool:
    first_side = _cross(first_start, first_end, second_start)
    second_side = _cross(first_start, first_end, second_end)
    third_side = _cross(second_start, second_end, first_start)
    fourth_side = _cross(second_start, second_end, first_end)
    if (
        ((first_side > 0.0 and second_side < 0.0) or (first_side < 0.0 and second_side > 0.0))
        and ((third_side > 0.0 and fourth_side < 0.0) or (third_side < 0.0 and fourth_side > 0.0))
    ):
        return True
    return any(
        (
            abs(side) <= 1.0e-9
            and _point_on_segment(point, segment_start, segment_end)
        )
        for side, point, segment_start, segment_end in (
            (first_side, second_start, first_start, first_end),
            (second_side, second_end, first_start, first_end),
            (third_side, first_start, second_start, second_end),
            (fourth_side, first_end, second_start, second_end),
        )
    )


def _point_in_polygon(point: Point2, polygon: Sequence[Point2]) -> bool:
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


def _point_segment_distance(point: Point2, start: Point2, end: Point2) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    squared = dx * dx + dy * dy
    if squared <= 1.0e-18:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = max(
        0.0,
        min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / squared),
    )
    return math.hypot(
        point[0] - (start[0] + ratio * dx),
        point[1] - (start[1] + ratio * dy),
    )


def _segment_distance(
    first_start: Point2,
    first_end: Point2,
    second_start: Point2,
    second_end: Point2,
) -> float:
    if _segments_intersect(first_start, first_end, second_start, second_end):
        return 0.0
    return min(
        _point_segment_distance(first_start, second_start, second_end),
        _point_segment_distance(first_end, second_start, second_end),
        _point_segment_distance(second_start, first_start, first_end),
        _point_segment_distance(second_end, first_start, first_end),
    )


def _boundary_polygon(region: Mapping[str, Any]) -> Optional[Tuple[Point2, ...]]:
    evidence = region.get("evidence")
    if not isinstance(evidence, Mapping) or "boundaryPolygon" not in evidence:
        return None
    value = evidence.get("boundaryPolygon")
    if not isinstance(value, list) or len(value) < 3:
        raise RouteError("forbidden region boundaryPolygon needs at least three points")
    points = []
    for index, item in enumerate(value):
        if not isinstance(item, Sequence) or isinstance(item, (str, bytes)) or len(item) < 2:
            raise RouteError("forbidden region polygon point %d is invalid" % index)
        point = (
            finite(item[0], "boundaryPolygon.x"),
            finite(item[1], "boundaryPolygon.y"),
        )
        if points and math.hypot(point[0] - points[-1][0], point[1] - points[-1][1]) < 0.01:
            raise RouteError("forbidden region polygon has adjacent duplicate points")
        points.append(point)
    if len(points) > 3 and points[0] == points[-1]:
        points.pop()
    if len(points) < 3 or abs(sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(points, points[1:] + points[:1])
    )) < 1.0e-6:
        raise RouteError("forbidden region polygon has no usable area")
    return tuple(points)


def _segment_intersects_polygon_clearance(
    start: Waypoint,
    end: Waypoint,
    polygon: Sequence[Point2],
    clearance: float,
) -> bool:
    route_start = (start.x, start.y)
    route_end = (end.x, end.y)
    if _point_in_polygon(route_start, polygon) or _point_in_polygon(route_end, polygon):
        return True
    edges = tuple(zip(polygon, polygon[1:] + polygon[:1]))
    return any(
        _segment_distance(route_start, route_end, edge_start, edge_end) <= clearance
        for edge_start, edge_end in edges
    )


def validate_route_forbidden_regions(
    route: Route,
    regions: Iterable[Mapping[str, Any]],
    clearance_m: float = 0.45,
) -> None:
    clearance = finite(clearance_m, "clearance_m")
    if clearance < 0.0:
        raise RouteError("clearance_m cannot be negative")
    for region in regions:
        polygon = _boundary_polygon(region)
        if polygon is not None:
            blocked = any(
                _segment_intersects_polygon_clearance(start, end, polygon, clearance)
                for start, end in zip(route.waypoints, route.waypoints[1:])
            )
            if blocked:
                raise RouteError(
                    "route crosses forbidden region %s"
                    % str(region.get("taskId", "unknown"))
                )
            continue
        bounds = region.get("bounds")
        if not isinstance(bounds, Mapping):
            raise RouteError("forbidden region has no bounds")
        minimum = bounds.get("min")
        maximum = bounds.get("max")
        if not isinstance(minimum, Sequence) or not isinstance(maximum, Sequence):
            raise RouteError("forbidden region bounds are invalid")
        rectangle = (
            finite(minimum[0], "bounds.min.x") - clearance,
            finite(minimum[1], "bounds.min.y") - clearance,
            finite(maximum[0], "bounds.max.x") + clearance,
            finite(maximum[1], "bounds.max.y") + clearance,
        )
        if any(
            _segment_intersects_rectangle(start, end, rectangle)
            for start, end in zip(route.waypoints, route.waypoints[1:])
        ):
            raise RouteError(
                "route crosses forbidden region %s"
                % str(region.get("taskId", "unknown"))
            )
