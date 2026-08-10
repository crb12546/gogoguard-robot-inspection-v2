from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from gogoguard_contracts import MapJobState


class GlimEditorError(RuntimeError):
    pass


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


def _free_loopback_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        return int(listener.getsockname()[1])


class GlimEditorManager:
    """Mac-owned SSH tunnel to an isolated official GLIM map-editor session."""

    def __init__(self, data_root: Path, maps, worker: str, cloud: dict | None) -> None:
        self.root = Path(data_root) / "map-editor-sessions"
        self.root.mkdir(parents=True, exist_ok=True)
        self.maps = maps
        self.worker = worker
        self.cloud = cloud or {}
        self._lock = threading.Lock()
        self._tunnels: dict[str, subprocess.Popen] = {}

    def close(self) -> None:
        with self._lock:
            processes = list(self._tunnels.values())
            self._tunnels.clear()
        for process in processes:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()

    def _cloud_identity(self) -> tuple[str, str, str, str]:
        host = str(self.cloud.get("host") or "")
        user = str(self.cloud.get("user") or "")
        remote_root = str(self.cloud.get("remote_root") or "").rstrip("/")
        remote_editor = str(
            self.cloud.get("remote_editor_command")
            or "/opt/go2/bin/gogoguard-glim-editor"
        )
        if self.worker != "ssh" or not all((host, user, remote_root, remote_editor)):
            raise GlimEditorError("官方 GLIM 3D 编辑器只在已配置的云端建图机上可用")
        return host, user, remote_root, remote_editor

    def _descriptor_path(self, job_id: str) -> Path:
        return self.root / f"{job_id}.json"

    def _read_descriptor(self, job_id: str) -> dict[str, Any]:
        path = self._descriptor_path(job_id)
        if not path.is_file():
            return {
                "schema": "gogoguard.glim_editor_session.v1",
                "sourceJobId": job_id,
                "state": "not_started",
            }
        return json.loads(path.read_text(encoding="utf-8"))

    @staticmethod
    def _last_json(output: str) -> dict[str, Any]:
        for line in reversed(output.splitlines()):
            try:
                value = json.loads(line)
            except ValueError:
                continue
            if isinstance(value, dict):
                return value
        raise GlimEditorError(output.strip() or "云端 GLIM 编辑器没有返回会话信息")

    @staticmethod
    def _run(command: list[str], *, timeout: float = 30.0) -> subprocess.CompletedProcess:
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise GlimEditorError(str(exc)) from exc
        if result.returncode != 0:
            raise GlimEditorError(result.stdout.strip() or f"command exited {result.returncode}")
        return result

    def _start_tunnel(
        self, job_id: str, session_id: str, remote_port: int, host: str, user: str
    ) -> tuple[subprocess.Popen, int]:
        old = self._tunnels.pop(job_id, None)
        if old is not None and old.poll() is None:
            old.terminate()
            try:
                old.wait(timeout=3)
            except subprocess.TimeoutExpired:
                old.kill()
        local_port = _free_loopback_port()
        process = subprocess.Popen(
            [
                "ssh",
                "-o",
                "ExitOnForwardFailure=yes",
                "-N",
                "-L",
                f"127.0.0.1:{local_port}:127.0.0.1:{remote_port}",
                f"{user}@{host}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise GlimEditorError("无法建立 Mac 到云端 GLIM 工作台的安全通道")
            try:
                with socket.create_connection(("127.0.0.1", local_port), timeout=0.2):
                    self._tunnels[job_id] = process
                    return process, local_port
            except OSError:
                time.sleep(0.15)
        process.terminate()
        raise GlimEditorError("云端 GLIM 工作台启动超时")

    @staticmethod
    def _local_descriptor(
        descriptor: dict[str, Any], job_id: str, local_port: int
    ) -> dict[str, Any]:
        return {
            **descriptor,
            "sourceJobId": job_id,
            "state": "active",
            "localPort": local_port,
            "url": (
                f"http://127.0.0.1:{local_port}/vnc.html"
                "?autoconnect=1&resize=scale&path=websockify"
            ),
            "instructions": (
                "在 GLIM 中删除临时的人、车等点云；然后点 File -> Save map，"
                "选择会话目录下的 SAVED_MAP，最后回到本页生成新地图版本。"
            ),
        }

    def start(self, job_id: str) -> dict[str, Any]:
        job = self.maps.get(job_id)
        if job.state != MapJobState.COMPLETE:
            raise GlimEditorError("只能清理已完成的 GLIM 地图")
        host, user, remote_root, remote_editor = self._cloud_identity()
        with self._lock:
            existing = self._read_descriptor(job_id)
            existing_process = self._tunnels.get(job_id)
            if (
                existing.get("state") == "active"
                and existing_process is not None
                and existing_process.poll() is None
            ):
                return dict(existing)
            existing_session = str(existing.get("sessionId") or "")
            existing_web_port = int(existing.get("webPort") or 0)
            if (
                existing.get("state") in {"active", "tunnel_closed"}
                and existing_session
                and existing_web_port
            ):
                try:
                    status_result = self._run(
                        [
                            "ssh",
                            f"{user}@{host}",
                            remote_editor,
                            "status",
                            existing_session,
                        ],
                        timeout=30.0,
                    )
                    remote_status = self._last_json(status_result.stdout)
                except GlimEditorError as exc:
                    if "does not exist" not in str(exc):
                        raise
                    remote_status = {"state": "missing"}
                if remote_status.get("state") == "active":
                    _, local_port = self._start_tunnel(
                        job_id,
                        existing_session,
                        existing_web_port,
                        host,
                        user,
                    )
                    descriptor = self._local_descriptor(
                        existing, job_id, local_port
                    )
                    _atomic_json(self._descriptor_path(job_id), descriptor)
                    return descriptor
            session_id = "edit-" + uuid.uuid4().hex[:12]
            remote_job = f"{remote_root}/{job_id}"
            remote = None
            last_error = None
            for attempt in range(8):
                slot = (int(uuid.uuid4().hex[:8], 16) + attempt) % 400
                try:
                    result = self._run(
                        [
                            "ssh",
                            f"{user}@{host}",
                            remote_editor,
                            "start",
                            session_id,
                            remote_job,
                            str(slot),
                        ],
                        timeout=90.0,
                    )
                    remote = self._last_json(result.stdout)
                    break
                except GlimEditorError as exc:
                    last_error = exc
                    if "slot is already in use" not in str(exc):
                        raise
            if remote is None:
                raise last_error or GlimEditorError("没有可用的 GLIM 编辑器端口")
            _, local_port = self._start_tunnel(
                job_id, session_id, int(remote["webPort"]), host, user
            )
            descriptor = self._local_descriptor(remote, job_id, local_port)
            _atomic_json(self._descriptor_path(job_id), descriptor)
            return descriptor

    def status(self, job_id: str) -> dict[str, Any]:
        self.maps.get(job_id)
        with self._lock:
            value = self._read_descriptor(job_id)
            process = self._tunnels.get(job_id)
            if value.get("state") == "active" and (
                process is None or process.poll() is not None
            ):
                value["state"] = "tunnel_closed"
                value.pop("url", None)
            return value

    def publish(self, job_id: str) -> dict[str, Any]:
        host, user, _, remote_editor = self._cloud_identity()
        with self._lock:
            descriptor = self._read_descriptor(job_id)
            session_id = str(descriptor.get("sessionId") or "")
            if descriptor.get("state") not in {"active", "tunnel_closed"} or not session_id:
                raise GlimEditorError("这张地图没有可导出的 GLIM 编辑会话")
            result = self._run(
                [
                    "ssh",
                    f"{user}@{host}",
                    remote_editor,
                    "export",
                    session_id,
                ],
                timeout=900.0,
            )
            exported = self._last_json(result.stdout)
            new_job_id = "map-" + uuid.uuid4().hex[:12]
            artifact_root = self.maps.root / new_job_id / "artifacts"
            artifact_root.mkdir(parents=True, exist_ok=False)
            try:
                self._run(
                    [
                        "rsync",
                        "-a",
                        "--partial",
                        f"{user}@{host}:{exported['output'].rstrip('/')}/",
                        str(artifact_root) + "/",
                    ],
                    timeout=300.0,
                )
                job = self.maps.import_edited(job_id, new_job_id, artifact_root)
            except Exception:
                shutil.rmtree(artifact_root.parent, ignore_errors=True)
                raise
            process = self._tunnels.pop(job_id, None)
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
            descriptor.update(
                {
                    "state": "published",
                    "publishedJobId": new_job_id,
                    "url": None,
                }
            )
            _atomic_json(self._descriptor_path(job_id), descriptor)
            return {
                "schema": "gogoguard.glim_editor_publish.v1",
                "sourceJobId": job_id,
                "mapJob": job,
                "message": "GLIM 清理结果已生成新地图版本，原地图保持不变",
            }

    def stop(self, job_id: str) -> dict[str, Any]:
        host, user, _, remote_editor = self._cloud_identity()
        with self._lock:
            descriptor = self._read_descriptor(job_id)
            session_id = str(descriptor.get("sessionId") or "")
            if session_id:
                self._run(
                    ["ssh", f"{user}@{host}", remote_editor, "stop", session_id],
                    timeout=30.0,
                )
            process = self._tunnels.pop(job_id, None)
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
            descriptor.update({"state": "closed", "url": None})
            _atomic_json(self._descriptor_path(job_id), descriptor)
            return descriptor
