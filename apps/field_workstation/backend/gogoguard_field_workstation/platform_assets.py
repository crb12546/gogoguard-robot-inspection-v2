from __future__ import annotations

import json
import hashlib
import os
import ssl
import tempfile
import threading
import time
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urljoin, urlsplit
from urllib.request import Request, urlopen


class PlatformAssetUploadError(RuntimeError):
    pass


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False, sort_keys=True, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class PlatformAssetUploader:
    """Workstation-owned, resumable asset transfer. It never activates a route."""

    def __init__(self, root: Path, config: dict[str, Any] | None) -> None:
        config = dict(config or {})
        self.root = Path(root) / "platform-uploads"
        self.root.mkdir(parents=True, exist_ok=True)
        self.base_url = str(
            config.get("base_url") or "http://39.96.37.187/api/v1"
        ).rstrip("/") + "/"
        parsed = urlsplit(self.base_url)
        if parsed.scheme not in {"http", "https"} or not parsed.hostname:
            raise ValueError("platform asset base URL is invalid")
        allow_http = config.get(
            "allow_insecure_http", parsed.hostname == "39.96.37.187"
        )
        if parsed.scheme == "http" and allow_http is not True:
            raise ValueError("platform asset HTTP requires allow_insecure_http=true")
        self.token_env = str(config.get("device_token_env") or "GOGOGUARD_DEVICE_TOKEN")
        self.timeout_s = float(config.get("timeout_s") or 30.0)
        self._threads: dict[str, threading.Thread] = {}
        self._lock = threading.Lock()

    def status(self, job_id: str) -> dict[str, Any]:
        path = self._state_path(job_id)
        if not path.is_file():
            return {
                "schema": "gogoguard.platform_asset_upload.v1",
                "jobId": job_id,
                "state": "not_started",
                "progress": 0,
                "message": "尚未上传到平台",
            }
        return json.loads(path.read_text(encoding="utf-8"))

    def start(self, job_id: str, descriptor: dict[str, Any]) -> dict[str, Any]:
        archive = Path(str(descriptor.get("archive") or ""))
        manifest = Path(str(descriptor.get("manifest") or ""))
        if not archive.is_file() or not manifest.is_file():
            raise PlatformAssetUploadError("platform bundle is unavailable")
        if (
            archive.stat().st_size != int(descriptor.get("archiveBytes") or -1)
            or _file_hash(archive) != str(descriptor.get("archiveSha256") or "")
        ):
            raise PlatformAssetUploadError("platform bundle changed after it was exported")
        if not os.environ.get(self.token_env, ""):
            raise PlatformAssetUploadError(
                f"platform device token is unavailable in {self.token_env}"
            )
        with self._lock:
            thread = self._threads.get(job_id)
            if thread and thread.is_alive():
                return self.status(job_id)
            previous = self.status(job_id)
            same_bundle = (
                previous.get("mapVersion") == descriptor.get("mapVersion")
                and previous.get("routeId") == descriptor.get("routeId")
                and previous.get("workspaceHash") == descriptor.get("workspaceHash")
                and previous.get("archiveSha256") == descriptor.get("archiveSha256")
            )
            if not same_bundle:
                _atomic_json(
                    self._state_path(job_id),
                    {
                        "schema": "gogoguard.platform_asset_upload.v1",
                        "jobId": job_id,
                        "state": "not_started",
                        "progress": 0,
                        "message": "路线版本已更新，等待上传新资产包",
                    },
                )
            self._write(
                job_id,
                state="queued",
                progress=0,
                message="已开始准备平台资产包",
                mapVersion=descriptor["mapVersion"],
                routeId=descriptor["routeId"],
                workspaceHash=descriptor.get("workspaceHash"),
                archiveBytes=descriptor["archiveBytes"],
                archiveSha256=descriptor["archiveSha256"],
            )
            thread = threading.Thread(
                target=self._run,
                args=(job_id, dict(descriptor)),
                name=f"platform-upload-{job_id}",
                daemon=True,
            )
            self._threads[job_id] = thread
            thread.start()
        return self.status(job_id)

    def _run(self, job_id: str, descriptor: dict[str, Any]) -> None:
        try:
            self._upload(job_id, descriptor)
        except Exception as exc:
            self._write(
                job_id,
                state="failed",
                message=str(exc)[:500] or type(exc).__name__,
            )

    def _upload(self, job_id: str, descriptor: dict[str, Any]) -> None:
        archive = Path(descriptor["archive"])
        manifest = json.loads(Path(descriptor["manifest"]).read_text(encoding="utf-8"))
        state = self.status(job_id)
        upload_id = str(state.get("uploadId") or "")
        remote: dict[str, Any] = {}
        if upload_id:
            try:
                remote = self._request("GET", f"assets/bundles/{quote(upload_id, safe='')}")
            except PlatformAssetUploadError:
                remote = {}
            if remote.get("status") in {"verified", "activated"}:
                activated = remote.get("status") == "activated"
                self._write(
                    job_id,
                    state="activated" if activated else "verified",
                    progress=100,
                    message=(
                        "平台已激活这一版地图与路线"
                        if activated
                        else "平台校验通过，等待运营激活"
                    ),
                    platform=remote,
                )
                return
            if remote.get("status") in {"rejected", "expired"}:
                upload_id = ""
                remote = {}
        if not upload_id:
            init = self._request(
                "POST",
                "assets/bundles/init",
                {
                    "zipBytes": int(descriptor["archiveBytes"]),
                    "zipSha256": str(descriptor["archiveSha256"]),
                    "mapVersion": str(descriptor["mapVersion"]),
                    "routeId": str(descriptor["routeId"]),
                    "files": [
                        {
                            "path": item["path"],
                            "sha256": item["sha256"],
                            "bytes": item["bytes"],
                        }
                        for item in manifest.get("files") or []
                    ],
                },
            )
            upload_id = str(init.get("uploadId") or "")
            if not upload_id:
                raise PlatformAssetUploadError("platform did not return uploadId")
            remote = init
            self._write(
                job_id,
                uploadId=upload_id,
                siteId=init.get("siteId"),
                siteName=init.get("siteName"),
                state="uploading",
                progress=1,
                message=f"正在上传到项目「{init.get('siteName') or '待确认'}」",
            )
        chunk_size = int(remote.get("chunkSize") or 8 * 1024 * 1024)
        total_chunks = int(
            remote.get("totalChunks")
            or (archive.stat().st_size + chunk_size - 1) // chunk_size
        )
        missing = remote.get("missing")
        missing = list(range(total_chunks)) if not isinstance(missing, list) else missing
        try:
            missing = sorted({int(index) for index in missing})
        except (TypeError, ValueError) as exc:
            raise PlatformAssetUploadError("platform returned invalid missing chunks") from exc
        if any(index < 0 or index >= total_chunks for index in missing):
            raise PlatformAssetUploadError("platform returned an out-of-range missing chunk")
        with archive.open("rb") as handle:
            for completed, index in enumerate(missing, start=1):
                index = int(index)
                handle.seek(index * chunk_size)
                data = handle.read(chunk_size)
                result = self._request(
                    "PUT",
                    f"assets/bundles/{quote(upload_id, safe='')}/chunk?index={index}",
                    raw=data,
                )
                received = result.get("received")
                count = len(received) if isinstance(received, list) else total_chunks - len(missing) + completed
                self._write(
                    job_id,
                    state="uploading",
                    progress=min(95, int(95 * count / max(total_chunks, 1))),
                    message=f"平台分片上传 {count}/{total_chunks}",
                    received=count,
                    totalChunks=total_chunks,
                )
        self._write(job_id, state="verifying", progress=97, message="平台正在校验地图、路线、点位和样张")
        complete = self._request(
            "POST", f"assets/bundles/{quote(upload_id, safe='')}/complete", {}
        )
        if complete.get("ok") is not True:
            raise PlatformAssetUploadError(
                str(complete.get("reason") or "platform rejected the completed asset bundle")
            )
        self._write(
            job_id,
            state="verified",
            progress=100,
            message="平台校验通过，等待运营激活",
            summary=complete.get("summary") or {},
            note=complete.get("note"),
        )

    def _request(
        self,
        method: str,
        path: str,
        body: dict[str, Any] | None = None,
        *,
        raw: bytes | None = None,
    ) -> dict[str, Any]:
        token = os.environ.get(self.token_env, "")
        if not token:
            raise PlatformAssetUploadError("platform device token is unavailable")
        content = raw
        headers = {"Accept": "application/json", "X-Device-Token": token}
        if body is not None:
            content = json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode()
            headers["Content-Type"] = "application/json"
        elif raw is not None:
            headers["Content-Type"] = "application/octet-stream"
        request = Request(urljoin(self.base_url, path), data=content, method=method, headers=headers)
        context = ssl.create_default_context()
        last_error: Exception | None = None
        for attempt in range(5):
            try:
                with urlopen(request, timeout=self.timeout_s, context=context) as response:
                    value = json.loads(response.read(1024 * 1024 + 1).decode("utf-8"))
                if not isinstance(value, dict):
                    raise PlatformAssetUploadError("platform returned a non-object response")
                return value
            except HTTPError as exc:
                try:
                    detail = json.loads(exc.read().decode("utf-8"))
                    message = detail.get("error") or detail.get("reason")
                except (ValueError, UnicodeDecodeError):
                    message = None
                if exc.code < 500:
                    raise PlatformAssetUploadError(message or f"platform returned HTTP {exc.code}") from exc
                last_error = exc
            except (URLError, TimeoutError) as exc:
                last_error = exc
            time.sleep(min(0.25 * (2**attempt), 2.0))
        raise PlatformAssetUploadError("platform asset transfer failed after retries") from last_error

    def _write(self, job_id: str, **updates: Any) -> None:
        value = self.status(job_id)
        value.update(updates)
        value["schema"] = "gogoguard.platform_asset_upload.v1"
        value["jobId"] = job_id
        value["updatedAt"] = time.time()
        _atomic_json(self._state_path(job_id), value)

    def _state_path(self, job_id: str) -> Path:
        if not job_id.startswith("map-") or "/" in job_id or ".." in job_id:
            raise ValueError("invalid map job id")
        return self.root / f"{job_id}.json"
