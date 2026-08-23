from __future__ import annotations

import json
import math
import re
import subprocess
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Callable

from gogoguard_contracts import MapJob, MapJobState, json_ready, utc_now
from gogoguard_evidence import EventJournal


class MapWorkerError(RuntimeError):
    pass


ROUTE_COVERAGE_SCHEMA = "gogoguard.recording_route_coverage.v1"
MAX_RECORDING_START_GAP_S = 0.5
MAX_ALIGNMENT_RMS_M = 0.20
MAX_ALIGNMENT_ERROR_M = 0.45
POSE_FRAMES = {"base_link", "lidar_link"}


def _validate_optimized_trajectory(path: Path) -> dict:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise MapWorkerError("GLIM optimized trajectory contract is unreadable") from exc
    if not isinstance(value, dict):
        raise MapWorkerError("GLIM optimized trajectory contract is invalid")
    poses = value.get("poses")
    if (
        value.get("schema") != "gogoguard.optimized_trajectory.v1"
        or value.get("frame") != "map"
        or value.get("poseFrame", "lidar_link") not in POSE_FRAMES
        or not isinstance(poses, list)
        or len(poses) < 2
    ):
        raise MapWorkerError("GLIM optimized trajectory contract is invalid")
    previous_timestamp = -math.inf
    for pose in poses:
        try:
            fields = tuple(
                float(pose[name])
                for name in ("timestamp", "x", "y", "z", "qx", "qy", "qz", "qw")
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise MapWorkerError("GLIM optimized trajectory pose is invalid") from exc
        timestamp, _, _, _, qx, qy, qz, qw = fields
        if not all(math.isfinite(item) for item in fields) or timestamp <= previous_timestamp:
            raise MapWorkerError("GLIM optimized trajectory pose is invalid")
        if math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) < 0.5:
            raise MapWorkerError("GLIM optimized trajectory quaternion is invalid")
        previous_timestamp = timestamp
    return value


def _recording_timestamp(value: str) -> float:
    try:
        parsed = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    except (TypeError, ValueError) as exc:
        raise MapWorkerError("recording start timestamp is invalid") from exc
    if parsed.tzinfo is None:
        raise MapWorkerError("recording start timestamp must include a timezone")
    return parsed.timestamp()


def _validate_recording_route_coverage(
    map_path: Path, optimized_path: Path, session_path: Path
) -> dict:
    try:
        artifact = json.loads(Path(map_path).read_text(encoding="utf-8"))
        optimized = json.loads(Path(optimized_path).read_text(encoding="utf-8"))
        session = json.loads(Path(session_path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise MapWorkerError("recording route coverage contract is unreadable") from exc
    coverage = artifact.get("routeCoverage")
    artifact_pose_frame = artifact.get("trajectoryPoseFrame", "lidar_link")
    optimized_pose_frame = optimized.get("poseFrame", "lidar_link")
    if (
        not isinstance(coverage, dict)
        or coverage.get("schema") != ROUTE_COVERAGE_SCHEMA
        or coverage.get("complete") is not True
        or optimized.get("routeCoverage") != coverage
        or artifact_pose_frame not in POSE_FRAMES
        or optimized_pose_frame not in POSE_FRAMES
        or artifact_pose_frame != optimized_pose_frame
    ):
        raise MapWorkerError("cloud GLIM result does not cover the recording start")
    trajectory = artifact.get("trajectory")
    poses = optimized.get("poses")
    if not isinstance(trajectory, list) or not trajectory or not isinstance(poses, list) or not poses:
        raise MapWorkerError("cloud GLIM route coverage is invalid")
    try:
        recording_start = float(coverage["recordingStartTimestamp"])
        route_start = float(coverage["routeStartTimestamp"])
        optimized_start = float(coverage["optimizedStartTimestamp"])
        route_gap = float(coverage["routeStartGapSec"])
        optimized_gap = float(coverage["optimizedStartGapSec"])
        pre_roll_count = int(coverage["preRollPoseCount"])
        start_pose = coverage["startMapPose"]
        start_xyz = tuple(float(start_pose[name]) for name in ("x", "y", "z"))
        trajectory_start = tuple(float(value) for value in trajectory[0][:3])
        first_optimized_timestamp = float(poses[0]["timestamp"])
        session_start = _recording_timestamp(session["started_at"])
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise MapWorkerError("cloud GLIM route coverage is invalid") from exc
    numeric = (
        recording_start,
        route_start,
        optimized_start,
        route_gap,
        optimized_gap,
        *start_xyz,
        *trajectory_start,
        first_optimized_timestamp,
        session_start,
    )
    if not all(math.isfinite(value) for value in numeric):
        raise MapWorkerError("cloud GLIM route coverage is invalid")
    if (
        abs(recording_start - session_start) > 1e-3
        or abs(route_gap - (route_start - recording_start)) > 1e-3
        or abs(optimized_gap - (optimized_start - recording_start)) > 1e-3
        or abs(optimized_start - first_optimized_timestamp) > 1e-3
        or math.dist(start_xyz, trajectory_start) > 1e-6
        or route_gap < -MAX_RECORDING_START_GAP_S
        or route_gap > MAX_RECORDING_START_GAP_S
        or pre_roll_count < 0
    ):
        raise MapWorkerError("cloud GLIM route coverage is inconsistent")
    alignment = coverage.get("alignment")
    if optimized_gap > MAX_RECORDING_START_GAP_S:
        if (
            pre_roll_count < 1
            or not isinstance(alignment, dict)
            or alignment.get("method") != "timestamp_matched_se2"
        ):
            raise MapWorkerError("cloud GLIM route prefix was not reconstructed")
        try:
            sample_count = int(alignment["sampleCount"])
            rms = float(alignment["rmsErrorM"])
            maximum = float(alignment["maxErrorM"])
        except (KeyError, TypeError, ValueError) as exc:
            raise MapWorkerError("cloud GLIM route alignment receipt is invalid") from exc
        if (
            sample_count < 8
            or not all(math.isfinite(value) for value in (rms, maximum))
            or rms > MAX_ALIGNMENT_RMS_M
            or maximum > MAX_ALIGNMENT_ERROR_M
        ):
            raise MapWorkerError("cloud GLIM route alignment is not reliable")
    return coverage


class MapJobManager:
    def __init__(self, data_root: Path, worker: str, journal: EventJournal, cloud: dict | None = None) -> None:
        self.data_root = Path(data_root)
        self.root = self.data_root / "map-jobs"
        self.root.mkdir(parents=True, exist_ok=True)
        self.worker = worker
        self.journal = journal
        self.cloud = cloud or {}
        self._jobs: dict[str, MapJob] = {}
        self._lock = threading.Lock()

    def submit(
        self,
        session,
        source_resolver: Callable[[str, Callable[..., None]], Path] | None = None,
    ) -> MapJob:
        if str(session.state.value if hasattr(session.state, "value") else session.state) != "sealed":
            raise MapWorkerError("only a sealed recording can be mapped")
        job_id = "map-" + uuid.uuid4().hex[:12]
        job = MapJob(job_id=job_id, session_id=session.session_id)
        with self._lock:
            self._jobs[job_id] = job
        self._save(job)
        threading.Thread(
            target=self._run,
            args=(job_id, Path(session.root), source_resolver),
            daemon=True,
        ).start()
        self.journal.append("map.queued", job_id=job_id, session_id=session.session_id, worker=self.worker)
        return self.get(job_id)

    def _run(
        self,
        job_id: str,
        session_root: Path,
        source_resolver: Callable[[str, Callable[..., None]], Path] | None,
    ) -> None:
        try:
            if source_resolver is not None:
                self._update(
                    job_id,
                    MapJobState.UPLOADING,
                    1,
                    "正在从机器狗同步封存录制包",
                    stage="robot_to_workstation",
                )

                def report(**changes) -> None:
                    progress = int(changes.pop("progress", 1))
                    message = str(changes.pop("message", "正在从机器狗同步录制包"))
                    self._update(
                        job_id,
                        MapJobState.UPLOADING,
                        max(1, min(30, progress)),
                        message,
                        stage="robot_to_workstation",
                        **changes,
                    )

                session_root = Path(source_resolver(job_id, report))
            target = self._run_demo if self.worker == "demo" else self._run_ssh
            target(job_id, session_root)
        except Exception as exc:
            current_progress = min(self.get(job_id).progress, 99)
            self._update(
                job_id,
                MapJobState.FAILED,
                current_progress,
                "建图工作流失败",
                stage="failed",
                error=str(exc),
            )
            self.journal.append("map.failed", job_id=job_id, error=str(exc))

    def get(self, job_id: str) -> MapJob:
        with self._lock:
            if job_id not in self._jobs:
                path = self.root / job_id / "job.json"
                if not path.exists():
                    raise KeyError(job_id)
                value = json.loads(path.read_text(encoding="utf-8"))
                value["state"] = MapJobState(value["state"])
                self._jobs[job_id] = MapJob(**value)
            value = self._jobs[job_id]
            return MapJob(**json_ready(value) | {"state": value.state})

    def list(self) -> list[MapJob]:
        identifiers = set(self._jobs)
        identifiers.update(path.parent.name for path in self.root.glob("map-*/job.json"))
        return [self.get(identifier) for identifier in sorted(identifiers, reverse=True)]

    def retry(
        self,
        job_id: str,
        session,
        source_resolver: Callable[[str, Callable[..., None]], Path] | None = None,
    ) -> MapJob:
        job = self.get(job_id)
        if job.state != MapJobState.FAILED:
            raise MapWorkerError("only a failed map job can be retried")
        if session.session_id != job.session_id:
            raise MapWorkerError("map job and recording session do not match")
        with self._lock:
            current = self._jobs[job_id]
            current.state = MapJobState.QUEUED
            current.progress = 0
            current.stage = "queued"
            current.message = "建图任务已重新排队"
            current.error = None
            current.updated_at = utc_now()
            self._save(current)
        threading.Thread(
            target=self._run,
            args=(job_id, Path(session.root), source_resolver),
            daemon=True,
        ).start()
        self.journal.append("map.retried", job_id=job_id, session_id=session.session_id)
        return self.get(job_id)

    def _update(self, job_id: str, state: MapJobState, progress: int, message: str, **changes) -> None:
        with self._lock:
            job = self._jobs[job_id]
            job.state, job.progress, job.message = state, progress, message
            job.updated_at = utc_now()
            for key, value in changes.items():
                setattr(job, key, value)
            self._save(job)

    def _run_demo(self, job_id: str, session_root: Path) -> None:
        try:
            self._update(
                job_id, MapJobState.UPLOADING, 35, "正在校验工作站录制包",
                stage="workstation_validation",
            )
            time.sleep(0.35)
            self._update(
                job_id, MapJobState.PROCESSING, 72, "GLIM 建图模拟处理中",
                stage="cloud_glim",
            )
            samples = session_root / "samples" / "snapshots.jsonl"
            trajectory: list[list[float]] = []
            points: list[list[float]] = []
            for line in samples.read_text(encoding="utf-8").splitlines():
                if not line:
                    continue
                sample = json.loads(line)
                pose = sample.get("pose", {})
                trajectory.append([float(pose.get("x", 0)), float(pose.get("y", 0)), float(pose.get("z", 0))])
                if sample.get("points"):
                    points = sample["points"]
            if not trajectory or not points:
                raise MapWorkerError("recording contains no usable trajectory or point cloud")
            self._update(
                job_id, MapJobState.ARTIFACTS, 92, "正在生成地图产物",
                stage="artifacts",
            )
            artifact_root = self.root / job_id / "artifacts"
            artifact_root.mkdir(parents=True, exist_ok=True)
            self._write_ply(artifact_root / "map.ply", points)
            self._write_overview(artifact_root / "overview.svg", trajectory, points)
            (artifact_root / "map.json").write_text(json.dumps({
                "schema": "gogoguard.map_artifact.v1", "job_id": job_id,
                "trajectory": trajectory, "points": points,
                "trajectoryPoseFrame": "base_link",
                "source": "demo-worker-not-glim",
            }, ensure_ascii=False) + "\n", encoding="utf-8")
            prefix = f"/artifacts/{job_id}"
            metrics = {"trajectory_samples": len(trajectory), "point_count": len(points), "worker": "demo"}
            self._update(job_id, MapJobState.COMPLETE, 100, "演示地图已生成（不是 GLIM 结果）", stage="complete",
                         artifact_root=str(artifact_root), overview_url=prefix + "/overview.svg",
                         point_cloud_url=prefix + "/map.json", metrics=metrics)
            self.journal.append("map.completed", job_id=job_id, **metrics)
        except Exception as exc:
            current_progress = min(self.get(job_id).progress, 99)
            self._update(job_id, MapJobState.FAILED, current_progress, "建图失败", stage="failed", error=str(exc))
            self.journal.append("map.failed", job_id=job_id, error=str(exc))

    def _run_ssh(self, job_id: str, session_root: Path) -> None:
        try:
            host, user = self.cloud.get("host"), self.cloud.get("user")
            remote_root, remote_command = self.cloud.get("remote_root"), self.cloud.get("remote_command")
            if not all((host, user, remote_root, remote_command)):
                raise MapWorkerError("cloud GLIM SSH adapter is not configured")
            remote_job = f"{remote_root.rstrip('/')}/{job_id}"
            total_bytes = sum(path.stat().st_size for path in session_root.rglob("*") if path.is_file())
            self._update(
                job_id, MapJobState.UPLOADING, 32, "上传录制包到云端 GLIM",
                stage="workstation_to_cloud", bytes_total=total_bytes,
                bytes_transferred=0, transfer_rate_bps=0.0,
            )
            subprocess.run(["ssh", f"{user}@{host}", "mkdir", "-p", remote_job], check=True)
            self._rsync_with_progress(
                job_id,
                ["rsync", "-a", "--partial", "--progress", str(session_root) + "/", f"{user}@{host}:{remote_job}/input/"],
                start_progress=32,
                end_progress=70,
                stage="workstation_to_cloud",
                message="上传录制包到云端 GLIM",
                total_bytes=total_bytes,
            )
            self._update(
                job_id, MapJobState.PROCESSING, 72, "云端 GLIM 校验输入",
                stage="cloud_validation", bytes_transferred=total_bytes,
            )
            self._run_cloud_job(job_id, ["ssh", f"{user}@{host}", remote_command, remote_job])
            artifact_root = self.root / job_id / "artifacts"
            artifact_root.mkdir(parents=True, exist_ok=True)
            self._update(
                job_id, MapJobState.ARTIFACTS, 94, "下载并校验 GLIM 产物",
                stage="cloud_to_workstation",
            )
            self._rsync_with_progress(
                job_id,
                ["rsync", "-a", "--partial", "--progress", f"{user}@{host}:{remote_job}/output/", str(artifact_root) + "/"],
                start_progress=94,
                end_progress=99,
                stage="cloud_to_workstation",
                message="下载并校验 GLIM 产物",
                total_bytes=0,
            )
            required = {
                "map.json",
                "map.ply",
                "overview.svg",
                "glim-build.json",
                "trajectory-poses.json",
            }
            returned = {path.name for path in artifact_root.iterdir() if path.is_file()}
            if not required.issubset(returned):
                raise MapWorkerError("cloud worker returned an incomplete GLIM artifact set")
            optimized_path = artifact_root / "trajectory-poses.json"
            _validate_optimized_trajectory(optimized_path)
            coverage = _validate_recording_route_coverage(
                artifact_root / "map.json",
                optimized_path,
                session_root / "session.json",
            )
            prefix = f"/artifacts/{job_id}"
            self._update(job_id, MapJobState.COMPLETE, 100, "云端 GLIM 地图已返回", stage="complete",
                         artifact_root=str(artifact_root), overview_url=prefix + "/overview.svg",
                         point_cloud_url=prefix + "/map.json", metrics={
                             "worker": "cloud-glim",
                             "route_start_gap_s": coverage["routeStartGapSec"],
                             "optimized_start_gap_s": coverage["optimizedStartGapSec"],
                             "route_prefix_pose_count": coverage["preRollPoseCount"],
                         })
        except Exception as exc:
            current_progress = min(self.get(job_id).progress, 99)
            self._update(job_id, MapJobState.FAILED, current_progress, "云端建图失败", stage="failed", error=str(exc))

    def _rsync_with_progress(
        self,
        job_id: str,
        command: list[str],
        *,
        start_progress: int,
        end_progress: int,
        stage: str,
        message: str,
        total_bytes: int,
    ) -> None:
        started = time.monotonic()
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        output: list[str] = []
        percent_pattern = re.compile(r"\s(\d{1,3})%")
        byte_pattern = re.compile(r"^\s*([\d,]+)\s+")
        assert process.stdout is not None
        for line in process.stdout:
            output.append(line)
            percent_match = percent_pattern.search(line)
            byte_match = byte_pattern.search(line)
            if percent_match:
                percent = max(0, min(100, int(percent_match.group(1))))
                progress = start_progress + round(
                    (end_progress - start_progress) * percent / 100
                )
                transferred = (
                    int(byte_match.group(1).replace(",", ""))
                    if byte_match else round(total_bytes * percent / 100)
                )
                elapsed = max(time.monotonic() - started, 0.001)
                self._update(
                    job_id,
                    MapJobState.UPLOADING if end_progress <= 70 else MapJobState.ARTIFACTS,
                    progress,
                    message,
                    stage=stage,
                    bytes_transferred=transferred,
                    bytes_total=total_bytes,
                    transfer_rate_bps=transferred / elapsed,
                )
        return_code = process.wait()
        if return_code != 0:
            raise MapWorkerError("".join(output[-30:]).strip() or "rsync failed")

    def _run_cloud_job(self, job_id: str, command: list[str]) -> None:
        marker = re.compile(r"^GOGOGUARD_PROGRESS\s+(\d+)\s+(\S+)\s+(.+)$")
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        output: list[str] = []
        assert process.stdout is not None
        for line in process.stdout:
            output.append(line)
            match = marker.match(line.strip())
            if match:
                progress = max(72, min(93, int(match.group(1))))
                self._update(
                    job_id,
                    MapJobState.PROCESSING,
                    progress,
                    match.group(3),
                    stage=match.group(2),
                )
        return_code = process.wait()
        if return_code != 0:
            raise MapWorkerError("".join(output[-40:]).strip() or "cloud GLIM failed")

    def import_edited(
        self, parent_job_id: str, job_id: str, artifact_root: Path
    ) -> dict:
        """Register an immutable map version exported by the official GLIM editor."""
        parent = self.get(parent_job_id)
        if parent.state != MapJobState.COMPLETE:
            raise MapWorkerError("parent GLIM map is not complete")
        if not re.fullmatch(r"map-[A-Za-z0-9]{12}", job_id):
            raise MapWorkerError("edited GLIM map id is invalid")
        artifact_root = Path(artifact_root)
        required = {
            "map.json",
            "map.ply",
            "overview.svg",
            "glim-build.json",
            "trajectory-poses.json",
        }
        if not required.issubset({path.name for path in artifact_root.iterdir()}):
            raise MapWorkerError("edited GLIM artifact set is incomplete")
        optimized_path = artifact_root / "trajectory-poses.json"
        _validate_optimized_trajectory(optimized_path)
        artifact = json.loads((artifact_root / "map.json").read_text(encoding="utf-8"))
        if (
            artifact.get("source") != "cloud-glim"
            or artifact.get("parent_map_job_id") != parent_job_id
            or artifact.get("editor") != "official-glim-map-editor"
        ):
            raise MapWorkerError("edited GLIM artifact provenance is invalid")
        # Historical edited maps predate the full recording-route receipt and
        # remain readable. Every newly deployed exporter includes the receipt;
        # when present, edited-map import validates it against the sealed local
        # session exactly as the normal cloud download path does.
        if artifact.get("routeCoverage") is not None:
            _validate_recording_route_coverage(
                artifact_root / "map.json",
                optimized_path,
                self.data_root / "recordings" / parent.session_id / "session.json",
            )
        prefix = f"/artifacts/{job_id}"
        job = MapJob(
            job_id=job_id,
            session_id=parent.session_id,
            state=MapJobState.COMPLETE,
            progress=100,
            stage="complete",
            message="GLIM 手工清理地图已返回",
            artifact_root=str(artifact_root),
            overview_url=prefix + "/overview.svg",
            point_cloud_url=prefix + "/map.json",
            metrics={
                "worker": "cloud-glim-editor",
                "parent_map_job_id": parent_job_id,
                "point_count": len(artifact.get("points") or []),
            },
        )
        with self._lock:
            if job_id in self._jobs or (self.root / job_id / "job.json").exists():
                raise MapWorkerError("edited GLIM map id already exists")
            self._jobs[job_id] = job
            self._save(job)
        self.journal.append(
            "map.edited",
            job_id=job_id,
            parent_job_id=parent_job_id,
            worker="official-glim-map-editor",
        )
        return json_ready(job)

    @staticmethod
    def _write_ply(path: Path, points: list[list[float]]) -> None:
        header = ["ply", "format ascii 1.0", f"element vertex {len(points)}", "property float x", "property float y", "property float z", "end_header"]
        body = [f"{float(p[0]):.4f} {float(p[1]):.4f} {float(p[2]):.4f}" for p in points]
        path.write_text("\n".join(header + body) + "\n", encoding="utf-8")

    @staticmethod
    def _write_overview(path: Path, trajectory: list[list[float]], points: list[list[float]]) -> None:
        xs = [p[0] for p in points] + [p[0] for p in trajectory]
        ys = [p[1] for p in points] + [p[1] for p in trajectory]
        min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)
        scale = min(700 / max(max_x - min_x, 0.1), 440 / max(max_y - min_y, 0.1))
        def xy(point):
            return 50 + (point[0] - min_x) * scale, 490 - (point[1] - min_y) * scale
        dots = "".join(f'<circle cx="{xy(p)[0]:.1f}" cy="{xy(p)[1]:.1f}" r="1.1" fill="#64748b" opacity=".55"/>' for p in points[::2])
        route = " ".join(f"{xy(p)[0]:.1f},{xy(p)[1]:.1f}" for p in trajectory)
        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="800" height="540" viewBox="0 0 800 540"><rect width="800" height="540" fill="#07111f"/><text x="28" y="34" fill="#dbeafe" font-family="sans-serif" font-size="18">Mapping overview · demo evidence</text>{dots}<polyline points="{route}" fill="none" stroke="#22d3ee" stroke-width="4"/><circle cx="{xy(trajectory[-1])[0]:.1f}" cy="{xy(trajectory[-1])[1]:.1f}" r="7" fill="#fb923c"/></svg>'''
        path.write_text(svg, encoding="utf-8")

    def _save(self, job: MapJob) -> None:
        root = self.root / job.job_id
        root.mkdir(parents=True, exist_ok=True)
        (root / "job.json").write_text(json.dumps(json_ready(job), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
