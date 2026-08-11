from __future__ import annotations

import argparse
import html
import json
import shutil
from pathlib import Path

import numpy as np
import open3d as o3d


def main() -> None:
    parser = argparse.ArgumentParser(description="Export one manually cleaned GLIM map")
    parser.add_argument("--ply", required=True, type=Path)
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--source-job", required=True)
    parser.add_argument("--editor-session", required=True)
    args = parser.parse_args()

    cloud = o3d.io.read_point_cloud(str(args.ply))
    points_array = np.asarray(cloud.points)
    if points_array.ndim != 2 or points_array.shape[1] != 3 or not len(points_array):
        raise SystemExit("GLIM export contains no XYZ points")
    points_array = points_array[np.isfinite(points_array).all(axis=1)]
    if len(points_array) > 50000:
        indices = np.linspace(0, len(points_array) - 1, 50000, dtype=np.int64)
        points_array = points_array[indices]
    points = [[float(x), float(y), float(z), 0.7] for x, y, z in points_array]

    trajectory = []
    optimized_poses = []
    for raw in args.trajectory.read_text(encoding="utf-8").splitlines():
        values = raw.split()
        if len(values) == 8:
            timestamp, x, y, z, qx, qy, qz, qw = (float(value) for value in values)
            trajectory.append([x, y, z])
            optimized_poses.append(
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
    if len(trajectory) < 2:
        raise SystemExit("GLIM trajectory contains fewer than two poses")

    args.output.mkdir(parents=True, exist_ok=True)
    artifact = {
        "schema": "gogoguard.map_artifact.v1",
        "source": "cloud-glim",
        "build_id": args.editor_session,
        "build_state": "manually_cleaned",
        "parent_map_job_id": args.source_job,
        "editor": "official-glim-map-editor",
        "trajectory": trajectory,
        "points": points,
    }
    (args.output / "map.json").write_text(
        json.dumps(artifact, ensure_ascii=False, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    (args.output / "trajectory-poses.json").write_text(
        json.dumps(
            {
                "schema": "gogoguard.optimized_trajectory.v1",
                "frame": "map",
                "poses": optimized_poses,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )
    shutil.copy2(args.ply, args.output / "map.ply")
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
    label = html.escape(f"GLIM cleaned map · {len(points)} displayed points")
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
