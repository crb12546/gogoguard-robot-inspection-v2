from __future__ import annotations

import hashlib
import json
import math
import os
import re
import shutil
import struct
import tempfile
import threading
import zipfile
from pathlib import Path
from typing import Any, Iterable

from gogoguard_contracts import is_safe_external_id

from .workspace import NavigationWorkspaceStore, validate_workspace, write_keepout_mask


SAFE_ID = re.compile(r"^map-[A-Za-z0-9]{12}$")
CANDIDATE_GENERATION = 7


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
            json.dump(
                payload,
                handle,
                ensure_ascii=False,
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _write_deterministic_zip_member(
    archive: zipfile.ZipFile,
    source: Path,
    relative_name: str,
) -> None:
    info = zipfile.ZipInfo(relative_name, date_time=(1980, 1, 1, 0, 0, 0))
    info.compress_type = zipfile.ZIP_STORED
    info.external_attr = 0o100644 << 16
    with source.open("rb") as reader, archive.open(info, "w") as writer:
        shutil.copyfileobj(reader, writer, length=1024 * 1024)


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


def _execution_route_payload(route: dict[str, Any], spacing_m: float) -> dict[str, Any]:
    """Mirror the runtime's sealed route interpolation for checkpoint binding."""
    if not 0.05 <= float(spacing_m) <= 0.5:
        raise ValueError("execution route spacing is invalid")
    waypoints = list(route.get("waypoints") or [])
    if len(waypoints) < 2:
        raise ValueError("route requires at least two waypoints")
    sampled = [dict(waypoints[0])]
    for start, end in zip(waypoints, waypoints[1:]):
        distance = math.hypot(float(end["x"]) - float(start["x"]), float(end["y"]) - float(start["y"]))
        steps = max(1, int(math.ceil(distance / float(spacing_m))))
        yaw_delta = math.atan2(
            math.sin(float(end["yaw"]) - float(start["yaw"])),
            math.cos(float(end["yaw"]) - float(start["yaw"])),
        )
        for step in range(1, steps + 1):
            ratio = step / steps
            yaw = float(start["yaw"]) + yaw_delta * ratio
            sampled.append(
                {
                    "routeProgressIndex": len(sampled),
                    "x": float(start["x"]) + (float(end["x"]) - float(start["x"])) * ratio,
                    "y": float(start["y"]) + (float(end["y"]) - float(start["y"])) * ratio,
                    "yaw": math.atan2(math.sin(yaw), math.cos(yaw)),
                }
            )
    sampled[0] = {"routeProgressIndex": 0, **sampled[0]}
    return {
        "schema": "gogoguard.execution_route.v1",
        "routeId": route["routeId"],
        "frame": "map",
        "spacingM": float(spacing_m),
        "waypoints": sampled,
    }


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _path_yaw(points: list[tuple[float, float]], index: int) -> float:
    if len(points) < 2:
        return 0.0
    before = points[max(0, index - 1)]
    after = points[min(len(points) - 1, index + 1)]
    return math.atan2(after[1] - before[1], after[0] - before[0])


class RouteManager:
    def __init__(self, data_root: Path, *, site_id: str) -> None:
        self.data_root = Path(data_root)
        self.site_id = site_id
        self.workspaces = NavigationWorkspaceStore(self.data_root)
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
            # On the robot, route-bound checkpoints advance with the editable
            # workspace and live beside immutable GLIM artifacts.  Mac map
            # jobs retain the recording-time source in artifacts/.
            current_checkpoints = artifact_root.parent / "checkpoints.json"
            legacy_checkpoints = artifact_root / "checkpoints.json"
            source_checkpoints = (
                current_checkpoints
                if current_checkpoints.is_file()
                else legacy_checkpoints
            )
            source_checkpoint_hash = (
                _file_hash(source_checkpoints) if source_checkpoints.is_file() else None
            )
            checkpoint_asset_path = destination / "checkpoints.json"
            workspace = validate_workspace(
                self.workspaces.get(job_id), require_ready=True
            )
            workspace_hash = workspace["workspaceHash"]
            candidate_path = destination / "candidate.json"
            if candidate_path.is_file():
                try:
                    existing = json.loads(candidate_path.read_text(encoding="utf-8"))
                    existing_paths = {
                        "localization_map_hash": Path(existing["localization_map"]),
                        "runtime_profile_hash": Path(existing["runtime_profile"]),
                        "allowed_area_mask_hash": Path(existing["allowed_area_mask"]),
                        "allowed_area_mask_image_hash": Path(
                            existing["allowed_area_mask_image"]
                        ),
                        "allowed_area_mask_metadata_hash": Path(
                            existing["allowed_area_mask_metadata"]
                        ),
                    }
                    route_path = Path(existing["route"])
                    route_payload = json.loads(route_path.read_text(encoding="utf-8"))
                    artifacts_unchanged = all(
                        path.is_file() and _file_hash(path) == existing[digest_name]
                        for digest_name, path in existing_paths.items()
                    ) and (
                        route_path.is_file()
                        and _canonical_hash(route_payload) == existing["route_hash"]
                    ) and (
                        (
                            source_checkpoint_hash is None
                            and not checkpoint_asset_path.exists()
                        )
                        or (
                            source_checkpoint_hash is not None
                            and checkpoint_asset_path.is_file()
                            and _file_hash(checkpoint_asset_path)
                            == source_checkpoint_hash
                            and existing.get("checkpoint_asset_hash")
                            == source_checkpoint_hash
                        )
                    )
                    if (
                        existing.get("candidate_generation") == CANDIDATE_GENERATION
                        and
                        existing.get("source_ply_sha256") == source_ply_hash
                        and existing.get("source_map_json_sha256") == source_map_json_hash
                        and existing.get("source_checkpoint_sha256")
                        == source_checkpoint_hash
                        and existing.get("workspace_hash") == workspace_hash
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
            planar = [(float(item[0]), float(item[1])) for item in workspace["route"]]
            route_id = f"route-{job_id[4:]}-workspace-r{workspace['revision']}"
            route = _route_payload(route_id, planar)
            route_path = destination / "route.json"
            _atomic_json(route_path, route)
            mask_pgm_path, mask_yaml_path, mask_metadata = write_keepout_mask(
                workspace,
                points,
                destination,
            )
            mask_metadata_path = destination / "allowed-area-mask.json"
            first = route["waypoints"][0]
            profile = {
                "schema": "go2.runtime_profile.v1",
                "frames": {"map": "map", "odom": "odom", "base": "base_link"},
                "localization": {
                    "mapArtifact": "localization_map",
                    "qualityProfileId": "go2-vgicp-orin-v2",
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
                    "plannerProfileId": "go2-nav2-smac-2d-v1",
                    "allowedAreaMaskArtifact": "allowed_area_mask",
                    "allowedAreaWorkspaceHash": workspace_hash,
                    "robotRadiusM": workspace["robotRadiusM"],
                },
                "patrol": {
                    "loopMode": "once",
                    "speedLimitMps": 0.60,
                    "startMaxDistanceM": 1.5,
                    "startMaxYawDeg": 45.0,
                    "pathSampleSpacingM": 0.15,
                },
            }
            profile_path = destination / "runtime_profile.json"
            _atomic_json(profile_path, profile)
            execution_route = _execution_route_payload(
                route, float(profile["patrol"]["pathSampleSpacingM"])
            )
            if source_checkpoints.is_file():
                shutil.copy2(source_checkpoints, checkpoint_asset_path)
            elif checkpoint_asset_path.exists():
                checkpoint_asset_path.unlink()
            length_m = sum(
                math.hypot(second[0] - first_point[0], second[1] - first_point[1])
                for first_point, second in zip(planar, planar[1:])
            )
            metadata = {
                "schema": "gogoguard.navigation_candidate.v1",
                "candidate_generation": CANDIDATE_GENERATION,
                "candidate_id": job_id,
                "site_id": self.site_id,
                "map_job_id": job_id,
                "map_version": job_id,
                "route_id": route_id,
                "route_length_m": round(length_m, 3),
                "route_waypoint_count": len(planar),
                "execution_route_point_count": len(execution_route["waypoints"]),
                "checkpoint_asset": (
                    str(checkpoint_asset_path) if checkpoint_asset_path.is_file() else None
                ),
                "checkpoint_asset_hash": (
                    _file_hash(checkpoint_asset_path) if checkpoint_asset_path.is_file() else None
                ),
                "point_count": point_count,
                "source_ply_sha256": source_ply_hash,
                "source_map_json_sha256": source_map_json_hash,
                "source_checkpoint_sha256": source_checkpoint_hash,
                "workspace_hash": workspace_hash,
                "workspace_revision": workspace["revision"],
                "localization_map": str(pcd_path),
                "route": str(route_path),
                "runtime_profile": str(profile_path),
                "allowed_area_mask": str(mask_yaml_path),
                "allowed_area_mask_image": str(mask_pgm_path),
                "allowed_area_mask_metadata": str(mask_metadata_path),
                "localization_map_hash": _file_hash(pcd_path),
                "route_hash": _canonical_hash(route),
                "runtime_profile_hash": _file_hash(profile_path),
                "allowed_area_mask_hash": _file_hash(mask_yaml_path),
                "allowed_area_mask_image_hash": _file_hash(mask_pgm_path),
                "allowed_area_mask_metadata_hash": _file_hash(mask_metadata_path),
                "allowed_area_mask_dimensions": {
                    "width": mask_metadata["width"],
                    "height": mask_metadata["height"],
                    "resolutionM": mask_metadata["resolutionM"],
                },
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

    def checkpoint_descriptor(self, candidate_id: str) -> dict[str, Any]:
        candidate = self.get(candidate_id)
        route = json.loads(Path(candidate["route"]).read_text(encoding="utf-8"))
        runtime_profile = json.loads(
            Path(candidate["runtime_profile"]).read_text(encoding="utf-8")
        )
        execution_route = _execution_route_payload(
            route, float(runtime_profile["patrol"]["pathSampleSpacingM"])
        )
        return self._bind_recorded_checkpoints(candidate_id, candidate, execution_route)

    def _bind_recorded_checkpoints(
        self,
        candidate_id: str,
        candidate: dict[str, Any],
        execution_route: dict[str, Any],
    ) -> dict[str, Any]:
        job_path = self.data_root / "map-jobs" / candidate_id / "job.json"
        try:
            job = json.loads(job_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, OSError, ValueError):
            job = {}
        session_id = str(job.get("session_id") or job.get("sessionId") or "")
        recording_root = self.data_root / "recordings" / session_id
        source_path = recording_root / "samples" / "inspection" / "checkpoints.json"
        base = {
            "schema": "gogoguard.checkpoints.v1",
            "mapVersion": candidate["map_version"],
            "routeId": candidate["route_id"],
            "recordingSessionId": session_id or None,
            "executionRoutePointCount": len(execution_route.get("waypoints") or []),
            "checkpoints": [],
            "audit": {
                "ready": True,
                "needsReview": 0,
                "message": "本次录制没有标记巡检点",
            },
        }
        if not session_id or not source_path.is_file():
            return base
        source = json.loads(source_path.read_text(encoding="utf-8"))
        raw_items = source.get("checkpoints")
        if not isinstance(raw_items, list):
            raise ValueError("recording checkpoints are invalid")
        map_json = json.loads(
            (self.data_root / "map-jobs" / candidate_id / "artifacts" / "map.json").read_text(
                encoding="utf-8"
            )
        )
        optimized = [
            (float(item[0]), float(item[1]))
            for item in map_json.get("trajectory") or []
            if isinstance(item, list) and len(item) >= 2
        ]
        if len(optimized) < 2 and raw_items:
            raise ValueError("GLIM trajectory is unavailable for checkpoint binding")
        raw_path: list[tuple[float, float]] = []
        snapshots_path = recording_root / "samples" / "snapshots.jsonl"
        if snapshots_path.is_file():
            with snapshots_path.open("r", encoding="utf-8") as handle:
                for line in handle:
                    try:
                        pose = json.loads(line).get("pose") or {}
                        raw_path.append((float(pose["x"]), float(pose["y"])))
                    except (ValueError, TypeError, KeyError, json.JSONDecodeError):
                        continue
        execution = list(execution_route.get("waypoints") or [])
        bound: list[dict[str, Any]] = []
        needs_review = 0
        checkpoint_ids: set[str] = set()
        denominator = max(1, len(raw_path) - 1)
        for ordinal, raw in enumerate(raw_items, start=1):
            checkpoint_id = str(raw.get("checkpointId") or f"cp_{ordinal:02d}")
            if (
                not is_safe_external_id(checkpoint_id)
                or checkpoint_id in checkpoint_ids
            ):
                raise ValueError("recording checkpoint identity is invalid or duplicated")
            checkpoint_ids.add(checkpoint_id)
            sample_index = max(0, int(raw.get("recordingSampleIndex") or 0))
            ratio = min(1.0, sample_index / denominator)
            optimized_index = min(len(optimized) - 1, int(round(ratio * (len(optimized) - 1))))
            target = optimized[optimized_index]
            route_index, route_point = min(
                enumerate(execution),
                key=lambda pair: math.hypot(
                    float(pair[1]["x"]) - target[0],
                    float(pair[1]["y"]) - target[1],
                ),
            )
            distance = math.hypot(
                float(route_point["x"]) - target[0],
                float(route_point["y"]) - target[1],
            )
            raw_pose = raw.get("rawPose") if isinstance(raw.get("rawPose"), dict) else {}
            raw_yaw = float(raw_pose.get("yaw") or 0.0)
            if not math.isfinite(raw_yaw):
                raise ValueError("recording checkpoint yaw is invalid")
            raw_tangent = _path_yaw(raw_path, min(sample_index, len(raw_path) - 1)) if raw_path else raw_yaw
            optimized_tangent = _path_yaw(optimized, optimized_index)
            body_yaw = _wrap_angle(optimized_tangent + _wrap_angle(raw_yaw - raw_tangent))
            review = distance > 1.0
            needs_review += int(review)
            sample_frames = []
            for source_relative in raw.get("sampleFrames") or []:
                source_name = Path(str(source_relative)).name
                sample_frames.append(f"samples/{checkpoint_id}-{source_name}")
            bound.append(
                {
                    "checkpointId": checkpoint_id,
                    "routeProgressIndex": int(route_point.get("routeProgressIndex", route_index)),
                    "position": {
                        "x": float(route_point["x"]),
                        "y": float(route_point["y"]),
                    },
                    "bodyYaw": body_yaw,
                    "camera": dict(raw.get("camera") or {}),
                    "spin": raw.get("spin") is not False,
                    "dwellSec": 3,
                    "note": str(raw.get("note") or ""),
                    "sampleFrames": sample_frames,
                    "binding": {
                        "recordingSampleIndex": sample_index,
                        "optimizedTrajectoryIndex": optimized_index,
                        "distanceToExecutionRouteM": round(distance, 3),
                        "needsReview": review,
                    },
                }
            )
        # Platform and runtime consume the route in progress order. Stable
        # sorting preserves multiple camera views at the same route index.
        bound.sort(key=lambda item: int(item["routeProgressIndex"]))
        base["checkpoints"] = bound
        base["audit"] = {
            "ready": needs_review == 0,
            "needsReview": needs_review,
            "message": (
                "所有巡检点已绑定到最终执行路线"
                if needs_review == 0
                else f"有 {needs_review} 个巡检点离最终路线超过 1 米，请先复核蓝色路线"
            ),
        }
        return base

    def export_platform_bundle(self, candidate_id: str) -> dict[str, Any]:
        """Build one portable map release without any robot-local paths."""
        candidate = self.get(candidate_id)
        workspace = validate_workspace(
            self.workspaces.get(candidate_id), require_ready=True
        )
        route_id = str(candidate["route_id"])
        output_root = self.data_root / "platform-assets" / candidate_id / route_id
        output_root.mkdir(parents=True, exist_ok=True)
        route_payload = json.loads(Path(candidate["route"]).read_text(encoding="utf-8"))
        runtime_profile = json.loads(
            Path(candidate["runtime_profile"]).read_text(encoding="utf-8")
        )
        execution_route_path = output_root / "execution-route.json"
        execution_route = _execution_route_payload(
            route_payload,
            float(runtime_profile["patrol"]["pathSampleSpacingM"]),
        )
        _atomic_json(
            execution_route_path,
            execution_route,
        )
        checkpoints = self._bind_recorded_checkpoints(
            candidate_id, candidate, execution_route
        )
        checkpoints_path = output_root / "checkpoints.json"
        _atomic_json(checkpoints_path, checkpoints)
        # Always materialize the validated workspace inside the portable
        # release. Legacy jobs may still be read from the old read-only
        # artifact location, which must never leak into a new bundle path.
        portable_workspace_path = output_root / "navigation-workspace.json"
        _atomic_json(portable_workspace_path, workspace)
        sources: list[tuple[str, str, Path, str]] = [
            ("localization_map", "map.pcd", Path(candidate["localization_map"]), "application/pcd"),
            ("route", "route.json", Path(candidate["route"]), "application/json"),
            (
                "execution_route",
                "execution-route.json",
                execution_route_path,
                "application/json",
            ),
            (
                "allowed_area_workspace",
                "navigation-workspace.json",
                portable_workspace_path,
                "application/json",
            ),
            (
                "allowed_area_raster_metadata",
                "allowed-area-mask.json",
                Path(candidate["allowed_area_mask_metadata"]),
                "application/json",
            ),
            (
                "checkpoints",
                "checkpoints.json",
                checkpoints_path,
                "application/json",
            ),
        ]
        preview = self.data_root / "map-jobs" / candidate_id / "artifacts" / "map.svg"
        if preview.is_file():
            sources.append(("preview", "map.svg", preview, "image/svg+xml"))

        session_id = str(checkpoints.get("recordingSessionId") or "")
        recording_root = self.data_root / "recordings" / session_id
        raw_by_id: dict[str, dict[str, Any]] = {}
        raw_checkpoint_path = recording_root / "samples" / "inspection" / "checkpoints.json"
        if raw_checkpoint_path.is_file():
            raw_payload = json.loads(raw_checkpoint_path.read_text(encoding="utf-8"))
            raw_by_id = {
                str(item.get("checkpointId")): item
                for item in raw_payload.get("checkpoints") or []
                if isinstance(item, dict)
            }
        for checkpoint in checkpoints["checkpoints"]:
            raw = raw_by_id.get(str(checkpoint["checkpointId"]), {})
            for source_relative, target_relative in zip(
                raw.get("sampleFrames") or [], checkpoint.get("sampleFrames") or []
            ):
                source = (recording_root / str(source_relative)).resolve()
                if recording_root.resolve() not in source.parents:
                    raise ValueError("checkpoint sample path is unsafe")
                sources.append(("checkpoint_sample", str(target_relative), source, "image/jpeg"))

        files: list[dict[str, Any]] = []
        for role, relative_name, source, content_type in sources:
            if not source.is_file():
                raise RuntimeError(f"platform map asset is missing: {role}")
            target = output_root / relative_name
            target.parent.mkdir(parents=True, exist_ok=True)
            if source.resolve() != target.resolve():
                temporary = target.with_name(f".{target.name}.part")
                with source.open("rb") as reader, temporary.open("wb") as writer:
                    shutil.copyfileobj(reader, writer, length=1024 * 1024)
                    writer.flush()
                    os.fsync(writer.fileno())
                os.replace(temporary, target)
            files.append(
                {
                    "role": role,
                    "path": relative_name,
                    "bytes": target.stat().st_size,
                    "sha256": _file_hash(target),
                    "contentType": content_type,
                }
            )

        manifest = {
            "schema": "gogoguard.map_asset_bundle.v1",
            "siteId": self.site_id,
            "mapVersion": candidate["map_version"],
            "routeId": route_id,
            "frame": "map",
            "units": "metres",
            "gravityAligned": True,
            "workspaceRevision": candidate["workspace_revision"],
            "workspaceHash": candidate["workspace_hash"],
            "routeWaypointCount": candidate["route_waypoint_count"],
            "executionRoutePointCount": candidate["execution_route_point_count"],
            "routeLengthM": candidate["route_length_m"],
            "pointCount": candidate["point_count"],
            "allowedArea": {
                "geometryPath": "navigation-workspace.json",
                "robotRadiusM": workspace["robotRadiusM"],
            },
            "checkpointBinding": {
                "mapVersionField": "mapVersion",
                "routeIdField": "routeId",
                "routeIndexField": "routeProgressIndex",
                "executionRoutePath": "execution-route.json",
                "checkpointsPath": "checkpoints.json",
            },
            "checkpointCount": len(checkpoints["checkpoints"]),
            "files": files,
        }
        manifest_path = output_root / "manifest.json"
        _atomic_json(manifest_path, manifest)
        archive_path = output_root / f"{candidate_id}-{route_id}.zip"
        temporary_archive = archive_path.with_name(f".{archive_path.name}.part")
        with zipfile.ZipFile(
            temporary_archive, "w", compression=zipfile.ZIP_STORED, allowZip64=True
        ) as archive:
            _write_deterministic_zip_member(archive, manifest_path, "manifest.json")
            for item in files:
                _write_deterministic_zip_member(
                    archive, output_root / item["path"], item["path"]
                )
        os.replace(temporary_archive, archive_path)
        return {
            "schema": "gogoguard.map_asset_export.v1",
            "mapVersion": candidate["map_version"],
            "routeId": route_id,
            "workspaceRevision": candidate["workspace_revision"],
            "workspaceHash": candidate["workspace_hash"],
            "manifest": str(manifest_path),
            "archive": str(archive_path),
            "archiveBytes": archive_path.stat().st_size,
            "archiveSha256": _file_hash(archive_path),
            "uploadState": "awaiting_platform_asset_api",
        }
