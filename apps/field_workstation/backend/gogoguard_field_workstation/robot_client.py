from __future__ import annotations

import hashlib
import http.client
import json
import os
import time
from pathlib import Path
from typing import Callable
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urljoin, urlparse
from urllib.request import Request, urlopen


class RobotConnectionError(RuntimeError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


class RobotClient:
    """Workstation-side client for the narrow robot Edge Agent contract."""

    def __init__(self, base_url: str, *, timeout_s: float = 10.0) -> None:
        self.base_url = base_url.rstrip("/") + "/"
        parsed = urlparse(self.base_url)
        if parsed.scheme not in {"http", "https"} or not parsed.hostname:
            raise ValueError("robot base_url must be an HTTP(S) URL")
        self.parsed = parsed
        self.timeout_s = timeout_s

    def get(self, path: str) -> dict:
        return self._json(path, "GET")

    def post(
        self,
        path: str,
        body: dict | None = None,
        *,
        timeout_s: float | None = None,
    ) -> dict:
        return self._json(path, "POST", body, timeout_s=timeout_s)

    def delete(self, path: str) -> dict:
        return self._json(path, "DELETE")

    def live(self) -> dict:
        return self.get("api/v1/live")

    def camera(self) -> dict:
        value = self.get("api/v1/camera")
        port = int(value.get("port") or 0)
        stream = quote(str(value.get("stream_path") or ""), safe="")
        if port and stream:
            scheme = "https" if self.parsed.scheme == "https" else "http"
            value["url"] = f"{scheme}://{self.parsed.hostname}:{port}/{stream}/?controls=false&muted=true&autoplay=true&playsInline=true"
        return value

    def gimbal(self) -> dict:
        return self.get("api/v1/gimbal")

    def move_gimbal(self, payload: dict) -> dict:
        return self.post("api/v1/gimbal/move", payload)

    def center_gimbal(self) -> dict:
        return self.post("api/v1/gimbal/center")

    def sessions(self) -> list[dict]:
        return list(self.get("api/v1/sessions").get("items") or [])

    def incidents(self) -> list[dict]:
        return list(self.get("api/v1/incidents").get("items") or [])

    def start_recording(self) -> dict:
        return self.post("api/v1/sessions/start")

    def stop_recording(self, session_id: str) -> dict:
        return self._json(
            f"api/v1/sessions/{quote(session_id, safe='')}/stop",
            "POST",
            timeout_s=max(self.timeout_s, 30.0),
        )

    def recording_checkpoints(self, session_id: str) -> dict:
        return self.get(
            f"api/v1/sessions/{quote(session_id, safe='')}/checkpoints"
        )

    def mark_recording_checkpoint(self, session_id: str, payload: dict) -> dict:
        return self.post(
            f"api/v1/sessions/{quote(session_id, safe='')}/checkpoints",
            payload,
            timeout_s=max(self.timeout_s, 15.0),
        )

    def delete_recording_checkpoint(self, session_id: str, checkpoint_id: str) -> dict:
        return self.delete(
            f"api/v1/sessions/{quote(session_id, safe='')}/checkpoints/"
            f"{quote(checkpoint_id, safe='')}"
        )

    def download_recording(
        self,
        session_id: str,
        destination: Path,
        report: Callable[..., None],
    ) -> Path:
        descriptor = self.get(
            f"api/v1/edge/recordings/{quote(session_id, safe='')}/export"
        )
        manifest = dict(descriptor["manifest"])
        session = dict(descriptor["session"])
        destination = Path(destination)
        destination.mkdir(parents=True, exist_ok=True)
        files = list(manifest.get("files") or [])
        total = int(descriptor.get("bytes_total") or sum(int(item["bytes"]) for item in files))
        completed = 0
        started = time.monotonic()
        for item in files:
            relative = Path(str(item["path"]))
            if relative.is_absolute() or ".." in relative.parts:
                raise RobotConnectionError("robot returned an unsafe recording path")
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            expected_size = int(item["bytes"])
            expected_sha = str(item["sha256"])
            if target.is_file() and target.stat().st_size == expected_size and _sha256(target) == expected_sha:
                completed += expected_size
                continue
            part = target.with_name(f".{target.name}.part")
            offset = min(part.stat().st_size, expected_size) if part.exists() else 0
            if part.exists() and part.stat().st_size > expected_size:
                part.unlink()
                offset = 0
            encoded = "/".join(quote(value, safe="") for value in relative.parts)
            request = Request(
                urljoin(
                    self.base_url,
                    f"api/v1/edge/recordings/{quote(session_id, safe='')}/files/{encoded}",
                ),
                headers={"Range": f"bytes={offset}-"} if offset else {},
            )
            try:
                response = urlopen(request, timeout=max(self.timeout_s, 60.0))
            except (HTTPError, URLError, TimeoutError) as exc:
                raise RobotConnectionError(f"recording download failed: {exc}") from exc
            mode = "ab" if offset and getattr(response, "status", 200) == 206 else "wb"
            if mode == "wb":
                offset = 0
            current = offset
            with response, part.open(mode) as output:
                while chunk := response.read(1024 * 1024):
                    output.write(chunk)
                    current += len(chunk)
                    elapsed = max(time.monotonic() - started, 0.001)
                    transferred = completed + current
                    report(
                        progress=1 + round(29 * transferred / max(total, 1)),
                        message=f"机器狗 → Mac：{transferred / 1024**2:.1f} / {total / 1024**2:.1f} MiB",
                        bytes_transferred=transferred,
                        bytes_total=total,
                        transfer_rate_bps=transferred / elapsed,
                    )
                output.flush()
                os.fsync(output.fileno())
            if part.stat().st_size != expected_size or _sha256(part) != expected_sha:
                raise RobotConnectionError(f"recording file verification failed: {relative}")
            os.replace(part, target)
            completed += expected_size

        manifest_path = destination / "recording_bundle.json"
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        session["root"] = str(destination)
        session["bundle_manifest"] = str(manifest_path)
        (destination / "session.json").write_text(
            json.dumps(session, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        report(
            progress=30,
            message="机器狗录制包已在 Mac 封存并校验",
            bytes_transferred=total,
            bytes_total=total,
            transfer_rate_bps=total / max(time.monotonic() - started, 0.001),
        )
        return destination

    def download_incident(self, incident_id: str, destination: Path) -> Path:
        descriptor = self.get(
            f"api/v1/edge/incidents/{quote(incident_id, safe='')}/export"
        )
        manifest = dict(descriptor["incident"])
        destination = Path(destination)
        destination.mkdir(parents=True, exist_ok=True)
        for item in descriptor.get("files") or []:
            relative = Path(str(item["path"]))
            if relative.is_absolute() or ".." in relative.parts:
                raise RobotConnectionError("robot returned an unsafe incident path")
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            expected_size = int(item["bytes"])
            expected_sha = str(item["sha256"])
            if target.is_file() and target.stat().st_size == expected_size and _sha256(target) == expected_sha:
                continue
            temporary = target.with_name(f".{target.name}.part")
            offset = min(temporary.stat().st_size, expected_size) if temporary.exists() else 0
            if temporary.exists() and temporary.stat().st_size > expected_size:
                temporary.unlink()
                offset = 0
            encoded = "/".join(quote(value, safe="") for value in relative.parts)
            request = Request(
                urljoin(
                    self.base_url,
                    f"api/v1/edge/incidents/{quote(incident_id, safe='')}/files/{encoded}",
                ),
                headers={"Range": f"bytes={offset}-"} if offset else {},
            )
            try:
                response = urlopen(request, timeout=max(self.timeout_s, 60.0))
            except (HTTPError, URLError, TimeoutError) as exc:
                raise RobotConnectionError(f"incident download failed: {exc}") from exc
            mode = "ab" if offset and getattr(response, "status", 200) == 206 else "wb"
            with response, temporary.open(mode) as output:
                while chunk := response.read(1024 * 1024):
                    output.write(chunk)
                output.flush()
                os.fsync(output.fileno())
            if temporary.stat().st_size != expected_size or _sha256(temporary) != expected_sha:
                temporary.unlink(missing_ok=True)
                raise RobotConnectionError(f"incident file verification failed: {relative}")
            os.replace(temporary, target)
        manifest["root"] = str(destination)
        (destination / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        return destination

    def deploy_map(
        self,
        job: dict,
        artifact_root: Path,
        *,
        additional_files: dict[str, Path] | None = None,
    ) -> dict:
        artifact_root = Path(artifact_root)
        sources = {
            path.name: path
            for path in artifact_root.iterdir()
            if path.is_file() and not path.name.startswith(".")
        }
        for name, path in (additional_files or {}).items():
            source = Path(path)
            if source.is_file():
                sources[str(name)] = source
        workspace_path = artifact_root.parent / "navigation-workspace.json"
        if workspace_path.is_file():
            # The Mac keeps operator-edited blue/green workspace state outside
            # immutable GLIM artifacts. The robot map-import bundle still
            # carries that file so both the deployed V6 legacy reader and the
            # corrected reader can prepare a generation-5 candidate.
            sources[workspace_path.name] = workspace_path
        files = []
        for name, path in sorted(sources.items()):
            files.append(
                {"name": name, "bytes": path.stat().st_size, "sha256": _sha256(path)}
            )

        remote_files: dict[str, dict] = {}
        try:
            descriptor = self.get(f"api/v1/edge/map-imports/{job['job_id']}")
            remote_files = {
                str(item.get("name", "")): item
                for item in descriptor.get("files") or []
                if isinstance(item, dict)
            }
        except RobotConnectionError:
            # Compatibility with an older edge release: PUT remains safe once
            # its server-side request-body handling is upgraded.
            remote_files = {}

        for item in files:
            remote = remote_files.get(item["name"])
            if remote and (
                int(remote.get("bytes", -1)) == item["bytes"]
                and str(remote.get("sha256", "")) == item["sha256"]
            ):
                continue
            path = sources[item["name"]]
            self._put_file(
                f"api/v1/edge/map-imports/{job['job_id']}/artifacts/"
                f"{quote(item['name'], safe='')}",
                path,
                item["sha256"],
            )
        payload = {
            "schema": "gogoguard.workstation_map_deployment.v1",
            "job_id": job["job_id"],
            "session_id": job.get("session_id", ""),
            "created_at": job.get("created_at"),
            "updated_at": job.get("updated_at"),
            "metrics": job.get("metrics") or {},
            "files": files,
        }
        return self.post(
            f"api/v1/edge/map-imports/{job['job_id']}/commit",
            payload,
            timeout_s=max(self.timeout_s, 60.0),
        )

    def _put_file(self, path: str, source: Path, digest: str) -> None:
        connection_type = (
            http.client.HTTPSConnection if self.parsed.scheme == "https" else http.client.HTTPConnection
        )
        port = self.parsed.port or (443 if self.parsed.scheme == "https" else 80)
        connection = connection_type(self.parsed.hostname, port, timeout=max(self.timeout_s, 60.0))
        request_path = "/" + path.lstrip("/")
        connection.putrequest("PUT", request_path)
        connection.putheader("Content-Length", str(source.stat().st_size))
        connection.putheader("Content-Type", "application/octet-stream")
        connection.putheader("X-Content-SHA256", digest)
        connection.endheaders()
        try:
            with source.open("rb") as handle:
                while chunk := handle.read(1024 * 1024):
                    connection.send(chunk)
            response = connection.getresponse()
            content = response.read()
            if response.status >= 300:
                try:
                    message = json.loads(content.decode("utf-8")).get("error")
                except (ValueError, UnicodeDecodeError):
                    message = content.decode("utf-8", errors="replace")
                raise RobotConnectionError(message or f"map upload failed: HTTP {response.status}")
        finally:
            connection.close()

    def _json(
        self,
        path: str,
        method: str,
        body: dict | None = None,
        *,
        timeout_s: float | None = None,
    ) -> dict:
        content = None if body is None else json.dumps(body, ensure_ascii=False).encode("utf-8")
        request = Request(
            urljoin(self.base_url, path.lstrip("/")),
            data=content,
            method=method,
            headers={"Content-Type": "application/json"} if content is not None else {},
        )
        try:
            with urlopen(request, timeout=timeout_s or self.timeout_s) as response:
                value = json.loads(response.read().decode("utf-8"))
        except HTTPError as exc:
            try:
                detail = json.loads(exc.read().decode("utf-8")).get("error")
            except (ValueError, UnicodeDecodeError):
                detail = str(exc)
            raise RobotConnectionError(detail or str(exc)) from exc
        except (URLError, TimeoutError, ValueError) as exc:
            raise RobotConnectionError(f"robot edge agent unavailable: {exc}") from exc
        if not isinstance(value, dict):
            raise RobotConnectionError("robot returned a non-object response")
        return value
