from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any, Iterable

from gogoguard_contracts import DiagnosticMode, IncidentBundle, IncidentState
from .incidents import IncidentStore, new_incident_id


def import_legacy_runtime_trace(
    data_root: Path,
    trace_paths: Iterable[Path],
    *,
    trigger: str = "legacy_runtime_fault",
    candidate_path: Path | None = None,
    site_id: str = "",
    robot_id: str = "",
    sensor_id: str = "",
) -> dict[str, Any]:
    """Turn the old metadata-only flight trace into an honest partial replay."""
    records = []
    for path in trace_paths:
        with Path(path).open("r", encoding="utf-8", errors="replace") as handle:
            for line in handle:
                try:
                    value = json.loads(line)
                except ValueError:
                    continue
                if value.get("schema") == "go2.runtime_trace.v1":
                    records.append(value)
    if not records:
        raise ValueError("runtime trace contains no usable records")
    records.sort(key=lambda value: float(value.get("wallTime") or 0))
    candidate = {}
    if candidate_path and candidate_path.is_file():
        candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    route = []
    route_source = Path(str(candidate.get("route") or ""))
    if route_source.is_file():
        route_payload = json.loads(route_source.read_text(encoding="utf-8"))
        route = route_payload.get("waypoints") or route_payload.get("points") or []
        route = [
            [float(item.get("x", 0)), float(item.get("y", 0))]
            if isinstance(item, dict) else [float(item[0]), float(item[1])]
            for item in route
        ]
    events = []
    stream_kinds = {
        "runtime": "runtime", "localization": "decision", "safety": "decision",
        "authorization": "decision", "nav2_raw_cmd": "decision",
        "nav2_smoothed_cmd": "decision", "collision_monitored_cmd": "decision",
        "final_cmd": "decision", "body_cloud": "cloud_metadata",
    }
    last_pose = None
    for record in records:
        stream = str(record.get("stream") or "")
        payload = dict(record.get("payload") or {})
        timestamp = float(record.get("wallTime") or 0)
        if stream == "odometry":
            last_pose = {name: payload.get(name) for name in ("x", "y", "z", "yaw")}
            events.append({"kind": "pose", "timestamp": timestamp, "pose": last_pose, "source": "odometry"})
        elif stream in stream_kinds:
            events.append({"kind": stream_kinds[stream], "topic": stream, "timestamp": timestamp, "payload": payload})
    incident_id = new_incident_id()
    store = IncidentStore(data_root)
    root = store.root / incident_id
    root.mkdir(parents=True, exist_ok=False)
    replay = {
        "schema": "gogoguard.incident_replay.v1",
        "triggered_at_epoch": records[-1]["wallTime"],
        "events": events,
        "route": route,
        "last_pose": last_pose,
        "runtime": next((item["payload"] for item in reversed(events) if item["kind"] == "runtime"), {}),
        "limitations": [
            "body_cloud contains metadata only; XYZ points were not recorded",
            "the historical local costmap was not recorded",
            "the historical camera stream was not recorded",
        ],
    }
    (root / "replay.json").write_text(json.dumps(replay, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")
    (root / "map-reference.json").write_text(json.dumps(candidate, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for index, source in enumerate(trace_paths):
        shutil.copy2(source, root / f"legacy-trace-{index + 1:03d}.jsonl")
    bundle = IncidentBundle(
        incident_id=incident_id,
        trigger=trigger,
        site_id=site_id,
        robot_id=robot_id,
        sensor_id=sensor_id,
        map_version=candidate.get("map_version"),
        route_id=candidate.get("route_id"),
        diagnostic_mode=DiagnosticMode.PRODUCTION,
        summary={"legacyTraceRecords": len(records), "previewEventCount": len(events), "source": "metadata_only_runtime_trace"},
    )
    return store.seal(
        bundle,
        evidence_present=["runtime_decisions", "odometry", "commands", "route"],
        evidence_missing=["point_cloud_xyz", "local_costmap", "camera", "planner_search_cells"],
        state=IncidentState.PARTIAL,
    )
