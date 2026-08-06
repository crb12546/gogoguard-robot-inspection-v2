from __future__ import annotations

import hashlib
import json
import math
import os
import re
import struct
import tempfile
import threading
from pathlib import Path
from typing import Any, Iterable


SAFE_ID = re.compile(r"^map-[A-Za-z0-9]{12}$")


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


def _read_binary_ply(path: Path) -> tuple[list[tuple[float, float, float, float]], str]:
    with path.open("rb") as handle:
        header_lines: list[str] = []
        while True:
            raw = handle.readline()
            if not raw:
                raise ValueError("PLY header is incomplete")
            line = raw.decode("ascii").strip()
            header_lines.append(line)
            if line == "end_header":
                break
        if "format binary_little_endian 1.0" not in header_lines:
            raise ValueError("only binary little-endian PLY maps are supported")
        vertex_line = next((line for line in header_lines if line.startswith("element vertex ")), "")
        count = int(vertex_line.rsplit(" ", 1)[-1])
        properties = [line.split()[-1] for line in header_lines if line.startswith("property float ")]
        if properties[:3] != ["x", "y", "z"] or properties[3:4] != ["intensity"]:
            raise ValueError("map PLY must contain float x/y/z/intensity")
        row = struct.Struct("<ffff")
        payload = handle.read(count * row.size)
        if len(payload) != count * row.size:
            raise ValueError("map PLY vertex payload is truncated")
        points = list(row.iter_unpack(payload))
    return points, hashlib.sha256(path.read_bytes()).hexdigest()


def _write_binary_pcd(path: Path, points: Iterable[tuple[float, float, float, float]]) -> int:
    values = list(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
        "TYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(values)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(values)}\nDATA binary\n"
    ).encode("ascii")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(header)
            row = struct.Struct("<ffff")
            for point in values:
                handle.write(row.pack(*point))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return len(values)


def _route_points(trajectory: list[list[float]], spacing_m: float = 0.25) -> list[tuple[float, float]]:
    if len(trajectory) < 2:
        raise ValueError("GLIM trajectory contains fewer than two poses")
    raw = [(float(item[0]), float(item[1])) for item in trajectory]
    sampled = [raw[0]]
    for point in raw[1:]:
        if math.hypot(point[0] - sampled[-1][0], point[1] - sampled[-1][1]) >= spacing_m:
            sampled.append(point)
    # The lying-down sequence is stationary in XY, so distance sampling removes
    # it. Preserve the last moving pose only when it contributes useful route.
    if math.hypot(raw[-1][0] - sampled[-1][0], raw[-1][1] - sampled[-1][1]) >= 0.10:
        sampled.append(raw[-1])
    if len(sampled) < 2:
        raise ValueError("GLIM trajectory has no traversable planar route")
    return sampled


def _route_payload(route_id: str, points: list[tuple[float, float]]) -> dict[str, Any]:
    waypoints = []
    for index, point in enumerate(points):
        neighbor = points[index + 1] if index + 1 < len(points) else points[index - 1]
        if index + 1 < len(points):
            yaw = math.atan2(neighbor[1] - point[1], neighbor[0] - point[0])
        else:
            yaw = math.atan2(point[1] - neighbor[1], point[0] - neighbor[0])
        waypoints.append({"x": point[0], "y": point[1], "yaw": yaw})
    return {"schema": "go2.route.v1", "routeId": route_id, "frame": "map", "waypoints": waypoints}


class RouteManager:
    def __init__(self, data_root: Path, *, site_id: str) -> None:
        self.data_root = Path(data_root)
        self.site_id = site_id
        self.root = self.data_root / "navigation" / "candidates"
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def prepare_map_job(self, job_id: str) -> dict[str, Any]:
        if not SAFE_ID.fullmatch(job_id):
            raise ValueError("invalid map job id")
        artifact_root = self.data_root / "map-jobs" / job_id / "artifacts"
        map_json = artifact_root / "map.json"
        map_ply = artifact_root / "map.ply"
        if not map_json.is_file() or not map_ply.is_file():
            raise ValueError("completed GLIM artifacts are unavailable")
        with self._lock:
            destination = self.root / job_id
            destination.mkdir(parents=True, exist_ok=True)
            source_map_json_hash = _file_hash(map_json)
            source_ply_hash = _file_hash(map_ply)
            candidate_path = destination / "candidate.json"
            if candidate_path.is_file():
                try:
                    existing = json.loads(candidate_path.read_text(encoding="utf-8"))
                    existing_paths = {
                        "localization_map_hash": Path(existing["localization_map"]),
                        "runtime_profile_hash": Path(existing["runtime_profile"]),
                    }
                    route_path = Path(existing["route"])
                    route_payload = json.loads(route_path.read_text(encoding="utf-8"))
                    artifacts_unchanged = all(
                        path.is_file() and _file_hash(path) == existing[digest_name]
                        for digest_name, path in existing_paths.items()
                    ) and (
                        route_path.is_file()
                        and _canonical_hash(route_payload) == existing["route_hash"]
                    )
                    if (
                        existing.get("source_ply_sha256") == source_ply_hash
                        and existing.get("source_map_json_sha256") == source_map_json_hash
                        and artifacts_unchanged
                    ):
                        return existing
                except (KeyError, OSError, TypeError, ValueError):
                    pass
            artifact = json.loads(map_json.read_text(encoding="utf-8"))
            if artifact.get("source") != "cloud-glim":
                raise ValueError("navigation requires a cloud GLIM artifact")
            points, verified_source_ply_hash = _read_binary_ply(map_ply)
            if verified_source_ply_hash != source_ply_hash:
                raise ValueError("map PLY changed while preparing navigation")
            pcd_path = destination / "map.pcd"
            point_count = _write_binary_pcd(pcd_path, points)
            planar = _route_points(artifact.get("trajectory") or [])
            route_id = f"route-{job_id[4:]}-recorded"
            route = _route_payload(route_id, planar)
            route_path = destination / "route.json"
            _atomic_json(route_path, route)
            first = route["waypoints"][0]
            profile = {
                "schema": "go2.runtime_profile.v1",
                "frames": {"map": "map", "odom": "odom", "base": "base_link"},
                "localization": {
                    "mapArtifact": "localization_map",
                    "qualityProfileId": "go2-vgicp-orin-v1",
                    "initializationZone": {
                        "kind": "circle",
                        "center": {"x": first["x"], "y": first["y"], "z": 0.0},
                        "radiusM": 5.0,
                        "expectedYawRad": first["yaw"],
                        "yawToleranceDeg": 60.0,
                    },
                },
                "routeArtifact": "route",
                "navigation": {
                    "controllerProfileId": "go2-nav2-mppi-omni-v1",
                    "collisionProfileId": "go2-mid360-collision-v1",
                },
                "patrol": {
                    "loopMode": "once",
                    "speedLimitMps": 0.25,
                    "startMaxDistanceM": 1.5,
                    "startMaxYawDeg": 45.0,
                    "pathSampleSpacingM": 0.15,
                },
            }
            profile_path = destination / "runtime_profile.json"
            _atomic_json(profile_path, profile)
            length_m = sum(
                math.hypot(second[0] - first_point[0], second[1] - first_point[1])
                for first_point, second in zip(planar, planar[1:])
            )
            metadata = {
                "schema": "gogoguard.navigation_candidate.v1",
                "candidate_id": job_id,
                "site_id": self.site_id,
                "map_job_id": job_id,
                "map_version": job_id,
                "route_id": route_id,
                "route_length_m": round(length_m, 3),
                "route_waypoint_count": len(planar),
                "point_count": point_count,
                "source_ply_sha256": source_ply_hash,
                "source_map_json_sha256": source_map_json_hash,
                "localization_map": str(pcd_path),
                "route": str(route_path),
                "runtime_profile": str(profile_path),
                "localization_map_hash": _file_hash(pcd_path),
                "route_hash": _canonical_hash(route),
                "runtime_profile_hash": _file_hash(profile_path),
            }
            _atomic_json(candidate_path, metadata)
            return metadata

    def get(self, candidate_id: str) -> dict[str, Any]:
        if not SAFE_ID.fullmatch(candidate_id):
            raise KeyError(candidate_id)
        path = self.root / candidate_id / "candidate.json"
        if not path.is_file():
            raise KeyError(candidate_id)
        return json.loads(path.read_text(encoding="utf-8"))
