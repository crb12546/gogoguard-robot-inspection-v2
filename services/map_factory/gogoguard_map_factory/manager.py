from __future__ import annotations

import json
import math
import subprocess
import threading
import time
import uuid
from pathlib import Path

from gogoguard_contracts import MapJob, MapJobState, json_ready, utc_now
from gogoguard_evidence import EventJournal


class MapWorkerError(RuntimeError):
    pass


class MapJobManager:
    def __init__(self, data_root: Path, worker: str, journal: EventJournal, cloud: dict | None = None) -> None:
        self.root = data_root / "map-jobs"
        self.root.mkdir(parents=True, exist_ok=True)
        self.worker = worker
        self.journal = journal
        self.cloud = cloud or {}
        self._jobs: dict[str, MapJob] = {}
        self._lock = threading.Lock()

    def submit(self, session) -> MapJob:
        if str(session.state.value if hasattr(session.state, "value") else session.state) != "sealed":
            raise MapWorkerError("only a sealed recording can be mapped")
        job_id = "map-" + uuid.uuid4().hex[:12]
        job = MapJob(job_id=job_id, session_id=session.session_id)
        with self._lock:
            self._jobs[job_id] = job
        self._save(job)
        target = self._run_demo if self.worker == "demo" else self._run_ssh
        threading.Thread(target=target, args=(job_id, Path(session.root)), daemon=True).start()
        self.journal.append("map.queued", job_id=job_id, session_id=session.session_id, worker=self.worker)
        return self.get(job_id)

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
            self._update(job_id, MapJobState.UPLOADING, 12, "正在校验并上传录制包")
            time.sleep(0.35)
            self._update(job_id, MapJobState.PROCESSING, 42, "GLIM 建图模拟处理中")
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
            self._update(job_id, MapJobState.ARTIFACTS, 78, "正在生成地图产物")
            artifact_root = self.root / job_id / "artifacts"
            artifact_root.mkdir(parents=True, exist_ok=True)
            self._write_ply(artifact_root / "map.ply", points)
            self._write_overview(artifact_root / "overview.svg", trajectory, points)
            (artifact_root / "map.json").write_text(json.dumps({
                "schema": "gogoguard.map_artifact.v1", "job_id": job_id,
                "trajectory": trajectory, "points": points,
                "source": "demo-worker-not-glim",
            }, ensure_ascii=False) + "\n", encoding="utf-8")
            prefix = f"/artifacts/{job_id}"
            metrics = {"trajectory_samples": len(trajectory), "point_count": len(points), "worker": "demo"}
            self._update(job_id, MapJobState.COMPLETE, 100, "演示地图已生成（不是 GLIM 结果）",
                         artifact_root=str(artifact_root), overview_url=prefix + "/overview.svg",
                         point_cloud_url=prefix + "/map.json", metrics=metrics)
            self.journal.append("map.completed", job_id=job_id, **metrics)
        except Exception as exc:
            self._update(job_id, MapJobState.FAILED, 100, "建图失败", error=str(exc))
            self.journal.append("map.failed", job_id=job_id, error=str(exc))

    def _run_ssh(self, job_id: str, session_root: Path) -> None:
        try:
            host, user = self.cloud.get("host"), self.cloud.get("user")
            remote_root, remote_command = self.cloud.get("remote_root"), self.cloud.get("remote_command")
            if not all((host, user, remote_root, remote_command)):
                raise MapWorkerError("cloud GLIM SSH adapter is not configured")
            remote_job = f"{remote_root.rstrip('/')}/{job_id}"
            self._update(job_id, MapJobState.UPLOADING, 10, "上传录制包到云端 GLIM")
            subprocess.run(["ssh", f"{user}@{host}", "mkdir", "-p", remote_job], check=True)
            subprocess.run(["rsync", "-a", str(session_root) + "/", f"{user}@{host}:{remote_job}/input/"], check=True)
            self._update(job_id, MapJobState.PROCESSING, 45, "云端 GLIM 处理中")
            subprocess.run(["ssh", f"{user}@{host}", remote_command, remote_job], check=True)
            artifact_root = self.root / job_id / "artifacts"
            artifact_root.mkdir(parents=True, exist_ok=True)
            subprocess.run(["rsync", "-a", f"{user}@{host}:{remote_job}/output/", str(artifact_root) + "/"], check=True)
            if not (artifact_root / "map.json").exists():
                raise MapWorkerError("cloud worker returned no map.json contract")
            prefix = f"/artifacts/{job_id}"
            self._update(job_id, MapJobState.COMPLETE, 100, "云端 GLIM 地图已返回",
                         artifact_root=str(artifact_root), overview_url=prefix + "/overview.svg",
                         point_cloud_url=prefix + "/map.json", metrics={"worker": "cloud-glim"})
        except Exception as exc:
            self._update(job_id, MapJobState.FAILED, 100, "云端建图失败", error=str(exc))

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
