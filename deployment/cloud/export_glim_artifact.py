from __future__ import annotations

import argparse
import bisect
import datetime as dt
import html
import json
import math
import shutil
from pathlib import Path
from typing import Any


ROUTE_COVERAGE_SCHEMA = "gogoguard.recording_route_coverage.v1"
ALIGNMENT_WINDOW_S = 20.0
ALIGNMENT_NEAREST_TOLERANCE_S = 0.08
MAX_RECORDING_START_GAP_S = 0.5
MAX_ALIGNMENT_RMS_M = 0.20
MAX_ALIGNMENT_ERROR_M = 0.45
MAX_JOIN_GAP_M = 0.75


def _timestamp(value: str) -> float:
    try:
        parsed = dt.datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    except (TypeError, ValueError) as exc:
        raise ValueError("recording timestamp is invalid") from exc
    if parsed.tzinfo is None:
        raise ValueError("recording timestamp must include a timezone")
    result = parsed.timestamp()
    if not math.isfinite(result):
        raise ValueError("recording timestamp is invalid")
    return result


def read_optimized_poses(path: Path) -> list[dict[str, float]]:
    poses: list[dict[str, float]] = []
    previous_timestamp = -math.inf
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        values = raw.split()
        if not values:
            continue
        if len(values) != 8:
            raise ValueError("GLIM trajectory contains a malformed pose")
        timestamp, x, y, z, qx, qy, qz, qw = (
            float(value) for value in values
        )
        fields = (timestamp, x, y, z, qx, qy, qz, qw)
        if not all(math.isfinite(value) for value in fields):
            raise ValueError("GLIM trajectory contains a non-finite pose")
        if timestamp <= previous_timestamp:
            raise ValueError("GLIM trajectory timestamps are not strictly increasing")
        if math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) < 0.5:
            raise ValueError("GLIM trajectory contains an invalid quaternion")
        poses.append(
            {
                "timestamp": timestamp,
                "x": x,
                "y": y,
                "z": z,
                "qx": qx,
                "qy": qy,
                "qz": qz,
                "qw": qw,
            }
        )
        previous_timestamp = timestamp
    if len(poses) < 2:
        raise ValueError("GLIM trajectory contains fewer than two poses")
    return poses


def _recording_samples(path: Path) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    previous_timestamp = -math.inf
    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                value = json.loads(line)
                pose = value["pose"]
                timestamp = _timestamp(value["captured_at"])
                sample = {
                    "timestamp": timestamp,
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "z": float(pose.get("z", 0.0)),
                }
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                raise ValueError("recording pose timeline is invalid") from exc
            if not all(math.isfinite(item) for item in sample.values()):
                raise ValueError("recording pose timeline is invalid")
            if timestamp == previous_timestamp and samples:
                previous = samples[-1]
                if math.dist(
                    (sample["x"], sample["y"], sample["z"]),
                    (previous["x"], previous["y"], previous["z"]),
                ) <= 1e-6:
                    # Marking a checkpoint persists the same sampled frame in
                    # the recording timeline. It is one observation, not a
                    # backwards clock jump or a second route pose.
                    continue
            if timestamp <= previous_timestamp:
                raise ValueError("recording pose timestamps are not strictly increasing")
            samples.append(sample)
            previous_timestamp = timestamp
    if len(samples) < 2:
        raise ValueError("recording pose timeline is unavailable")
    return samples


def _fit_planar_transform(
    pairs: list[tuple[dict[str, float], dict[str, float]]]
) -> tuple[float, float, float, float, float, float]:
    if len(pairs) < 8:
        raise ValueError("too few synchronized poses to recover the recording start")
    raw_x = sum(raw["x"] for raw, _ in pairs) / len(pairs)
    raw_y = sum(raw["y"] for raw, _ in pairs) / len(pairs)
    map_x = sum(mapped["x"] for _, mapped in pairs) / len(pairs)
    map_y = sum(mapped["y"] for _, mapped in pairs) / len(pairs)
    dot = 0.0
    cross = 0.0
    spread = 0.0
    for raw, mapped in pairs:
        rx, ry = raw["x"] - raw_x, raw["y"] - raw_y
        mx, my = mapped["x"] - map_x, mapped["y"] - map_y
        dot += rx * mx + ry * my
        cross += rx * my - ry * mx
        spread += rx * rx + ry * ry
    if spread < 0.04 or math.hypot(dot, cross) < 1e-6:
        raise ValueError("recording motion is insufficient to align the missing route prefix")
    angle = math.atan2(cross, dot)
    cosine, sine = math.cos(angle), math.sin(angle)
    translate_x = map_x - (cosine * raw_x - sine * raw_y)
    translate_y = map_y - (sine * raw_x + cosine * raw_y)
    z_offset = sum(mapped["z"] - raw["z"] for raw, mapped in pairs) / len(pairs)
    errors = []
    for raw, mapped in pairs:
        predicted_x = cosine * raw["x"] - sine * raw["y"] + translate_x
        predicted_y = sine * raw["x"] + cosine * raw["y"] + translate_y
        errors.append(math.hypot(predicted_x - mapped["x"], predicted_y - mapped["y"]))
    rms = math.sqrt(sum(error * error for error in errors) / len(errors))
    maximum = max(errors)
    if rms > MAX_ALIGNMENT_RMS_M or maximum > MAX_ALIGNMENT_ERROR_M:
        raise ValueError(
            f"recording/GLIM alignment is not reliable (rms={rms:.3f}m, max={maximum:.3f}m)"
        )
    return angle, translate_x, translate_y, z_offset, rms, maximum


def reconstruct_recording_trajectory(
    optimized_poses: list[dict[str, float]],
    session: dict[str, Any],
    snapshots_path: Path,
) -> tuple[list[list[float]], dict[str, Any]]:
    recording_started_at = str(session.get("started_at") or "")
    recording_started_timestamp = _timestamp(recording_started_at)
    samples = _recording_samples(snapshots_path)
    optimized_start = float(optimized_poses[0]["timestamp"])
    if samples[0]["timestamp"] - recording_started_timestamp > MAX_RECORDING_START_GAP_S:
        raise ValueError("recording pose timeline does not cover the recording start")
    if optimized_start < recording_started_timestamp - MAX_RECORDING_START_GAP_S:
        raise ValueError("GLIM trajectory starts before the sealed recording")

    optimized_timestamps = [float(pose["timestamp"]) for pose in optimized_poses]
    missing_prefix = [sample for sample in samples if sample["timestamp"] < optimized_start]
    pre_roll_gap = max(0.0, optimized_start - recording_started_timestamp)
    transformed_prefix: list[list[float]] = []
    alignment: dict[str, Any] = {"method": "not_required", "sampleCount": 0}

    if pre_roll_gap > MAX_RECORDING_START_GAP_S:
        pairs: list[tuple[dict[str, float], dict[str, float]]] = []
        for sample in samples:
            if not optimized_start <= sample["timestamp"] <= optimized_start + ALIGNMENT_WINDOW_S:
                continue
            index = bisect.bisect_left(optimized_timestamps, sample["timestamp"])
            candidates = [
                candidate
                for candidate in (index - 1, index)
                if 0 <= candidate < len(optimized_poses)
            ]
            nearest = min(
                candidates,
                key=lambda candidate: abs(
                    optimized_timestamps[candidate] - sample["timestamp"]
                ),
            )
            if (
                abs(optimized_timestamps[nearest] - sample["timestamp"])
                <= ALIGNMENT_NEAREST_TOLERANCE_S
            ):
                pairs.append((sample, optimized_poses[nearest]))
        angle, translate_x, translate_y, z_offset, rms, maximum = (
            _fit_planar_transform(pairs)
        )
        cosine, sine = math.cos(angle), math.sin(angle)
        for sample in missing_prefix:
            transformed_prefix.append(
                [
                    cosine * sample["x"] - sine * sample["y"] + translate_x,
                    sine * sample["x"] + cosine * sample["y"] + translate_y,
                    sample["z"] + z_offset,
                ]
            )
        if not transformed_prefix:
            raise ValueError("recording route prefix is unavailable")
        join_gap = math.hypot(
            transformed_prefix[-1][0] - optimized_poses[0]["x"],
            transformed_prefix[-1][1] - optimized_poses[0]["y"],
        )
        if join_gap > MAX_JOIN_GAP_M:
            raise ValueError(f"recording route prefix does not join GLIM (gap={join_gap:.3f}m)")
        alignment = {
            "method": "timestamp_matched_se2",
            "sampleCount": len(pairs),
            "windowSec": ALIGNMENT_WINDOW_S,
            "nearestToleranceSec": ALIGNMENT_NEAREST_TOLERANCE_S,
            "rotationRad": angle,
            "translationM": [translate_x, translate_y],
            "zOffsetM": z_offset,
            "rmsErrorM": rms,
            "maxErrorM": maximum,
            "joinGapM": join_gap,
        }

    optimized_trajectory = [
        [pose["x"], pose["y"], pose["z"]] for pose in optimized_poses
    ]
    trajectory = transformed_prefix + optimized_trajectory
    route_start_timestamp = (
        missing_prefix[0]["timestamp"] if transformed_prefix else optimized_start
    )
    route_start_gap = route_start_timestamp - recording_started_timestamp
    if route_start_gap > MAX_RECORDING_START_GAP_S:
        raise ValueError("exported route does not cover the recording start")
    route_start = trajectory[0]
    coverage = {
        "schema": ROUTE_COVERAGE_SCHEMA,
        "complete": True,
        "recordingStartedAt": recording_started_at,
        "recordingStartTimestamp": recording_started_timestamp,
        "routeStartTimestamp": route_start_timestamp,
        "routeStartGapSec": route_start_gap,
        "optimizedStartTimestamp": optimized_start,
        "optimizedStartGapSec": pre_roll_gap,
        "preRollPoseCount": len(transformed_prefix),
        "startMapPose": {"x": route_start[0], "y": route_start[1], "z": route_start[2]},
        "alignment": alignment,
    }
    return trajectory, coverage


def main() -> None:
    parser = argparse.ArgumentParser(description="Export one manually cleaned GLIM map")
    parser.add_argument("--ply", required=True, type=Path)
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--source-job", required=True)
    parser.add_argument("--editor-session")
    parser.add_argument("--build", type=Path)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--snapshots", required=True, type=Path)
    args = parser.parse_args()

    if bool(args.editor_session) == bool(args.build):
        raise SystemExit("exactly one of --editor-session or --build is required")

    import numpy as np
    import open3d as o3d

    cloud = o3d.io.read_point_cloud(str(args.ply))
    points_array = np.asarray(cloud.points)
    if points_array.ndim != 2 or points_array.shape[1] != 3 or not len(points_array):
        raise SystemExit("GLIM export contains no XYZ points")
    points_array = points_array[np.isfinite(points_array).all(axis=1)]
    if len(points_array) > 50000:
        indices = np.linspace(0, len(points_array) - 1, 50000, dtype=np.int64)
        points_array = points_array[indices]
    points = [[float(x), float(y), float(z), 0.7] for x, y, z in points_array]

    try:
        optimized_poses = read_optimized_poses(args.trajectory)
        session = json.loads(args.session.read_text(encoding="utf-8"))
        trajectory, route_coverage = reconstruct_recording_trajectory(
            optimized_poses, session, args.snapshots
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc)) from exc

    args.output.mkdir(parents=True, exist_ok=True)
    artifact: dict[str, Any] = {
        "schema": "gogoguard.map_artifact.v1",
        "source": "cloud-glim",
        "trajectoryPoseFrame": "lidar_link",
        "trajectory": trajectory,
        "routeCoverage": route_coverage,
        "points": points,
    }
    if args.editor_session:
        artifact.update(
            {
                "build_id": args.editor_session,
                "build_state": "manually_cleaned",
                "parent_map_job_id": args.source_job,
                "editor": "official-glim-map-editor",
            }
        )
    else:
        build = json.loads(args.build.read_text(encoding="utf-8"))
        artifact.update(
            {"build_id": build["buildId"], "build_state": build["state"]}
        )
    (args.output / "map.json").write_text(
        json.dumps(
            artifact,
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    (args.output / "trajectory-poses.json").write_text(
        json.dumps(
        {
            "schema": "gogoguard.optimized_trajectory.v1",
            "frame": "map",
            "poseFrame": "lidar_link",
            "poses": optimized_poses,
            "routeCoverage": route_coverage,
        },
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    shutil.copy2(args.ply, args.output / "map.ply")
    if args.editor_session:
        (args.output / "glim-build.json").write_text(
            json.dumps(
                {
                    "schema": "gogoguard.glim_edited_build.v1",
                    "buildId": args.editor_session,
                    "state": "manually_cleaned",
                    "sourceJobId": args.source_job,
                    "editor": "official-glim-map-editor",
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
    else:
        shutil.copy2(args.build, args.output / "glim-build.json")

    overview_points = points[:: max(1, len(points) // 2500)]
    xs = [point[0] for point in overview_points] + [point[0] for point in trajectory]
    ys = [point[1] for point in overview_points] + [point[1] for point in trajectory]
    min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)
    scale = min(700 / max(max_x - min_x, 0.1), 440 / max(max_y - min_y, 0.1))

    def xy(point):
        return 50 + (point[0] - min_x) * scale, 490 - (point[1] - min_y) * scale

    dots = "".join(
        f'<circle cx="{xy(point)[0]:.1f}" cy="{xy(point)[1]:.1f}" r="1.1" fill="#64748b" opacity=".55"/>'
        for point in overview_points
    )
    route = " ".join(f"{xy(point)[0]:.1f},{xy(point)[1]:.1f}" for point in trajectory)
    label = html.escape(
        f"GLIM {'cleaned ' if args.editor_session else ''}map · {len(points)} displayed points"
    )
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg" width="800" height="540" viewBox="0 0 800 540">'
        '<rect width="800" height="540" fill="#07111f"/>'
        f'<text x="28" y="34" fill="#dbeafe" font-family="sans-serif" font-size="18">{label}</text>'
        f'{dots}<polyline points="{route}" fill="none" stroke="#22d3ee" stroke-width="4"/>'
        f'<circle cx="{xy(trajectory[-1])[0]:.1f}" cy="{xy(trajectory[-1])[1]:.1f}" r="7" fill="#fb923c"/>'
        "</svg>"
    )
    (args.output / "overview.svg").write_text(svg, encoding="utf-8")


if __name__ == "__main__":
    main()
