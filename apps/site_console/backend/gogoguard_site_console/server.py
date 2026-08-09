from __future__ import annotations

import json
import mimetypes
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

from gogoguard_contracts import json_ready


class SiteConsoleServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, application, static_root: Path):
        self.application = application
        self.static_root = static_root.resolve()
        super().__init__(address, SiteConsoleHandler)


class SiteConsoleHandler(BaseHTTPRequestHandler):
    server_version = "GogoguardSiteConsole/0.1"

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        try:
            if path == "/api/v1/status":
                return self._json(200, self.server.application.status())
            if path == "/api/v1/live":
                return self._json(200, self.server.application.live())
            if path == "/api/v1/camera":
                return self._json(200, self.server.application.camera_status())
            if path == "/api/v1/navigation":
                return self._json(200, self.server.application.navigation_status())
            if path == "/api/v1/navigation/profile":
                return self._json(200, self.server.application.navigation_profile())
            if path == "/api/v1/navigation/diagnostics":
                return self._json(200, self.server.application.navigation_diagnostics())
            if path == "/api/v1/diagnostics/profile":
                return self._json(200, self.server.application.diagnostic_profile())
            if path == "/api/v1/incidents":
                return self._json(200, {"items": self.server.application.incidents()})
            if path == "/api/v1/map-jobs":
                return self._json(200, {"items": self.server.application.map_jobs()})
            if path == "/api/v1/sessions":
                return self._json(200, {"items": self.server.application.sessions()})
            match = re.fullmatch(r"/api/v1/sessions/([A-Za-z0-9-]+)", path)
            if match:
                return self._json(200, self.server.application.session(match.group(1)))
            match = re.fullmatch(r"/api/v1/map-jobs/([A-Za-z0-9-]+)", path)
            if match:
                return self._json(200, self.server.application.map_job(match.group(1)))
            match = re.fullmatch(r"/api/v1/incidents/(incident-[A-Za-z0-9-]+)", path)
            if match:
                return self._json(200, self.server.application.incident(match.group(1)))
            match = re.fullmatch(
                r"/api/v1/edge/incidents/(incident-[A-Za-z0-9-]+)/export", path
            )
            if match:
                return self._json(200, self.server.application.incident_export(match.group(1)))
            match = re.fullmatch(
                r"/api/v1/edge/incidents/(incident-[A-Za-z0-9-]+)/files/(.+)", path
            )
            if match:
                return self._stream_file(
                    self.server.application.incident_file(match.group(1), unquote(match.group(2)))
                )
            match = re.fullmatch(
                r"/api/v1/incidents/(incident-[A-Za-z0-9-]+)/files/(.+)", path
            )
            if match:
                return self._stream_file(
                    self.server.application.incident_file(match.group(1), unquote(match.group(2)))
                )
            match = re.fullmatch(
                r"/api/v1/edge/recordings/([A-Za-z0-9-]+)/export", path
            )
            if match:
                return self._json(
                    200, self.server.application.recording_export(match.group(1))
                )
            match = re.fullmatch(
                r"/api/v1/edge/map-imports/(map-[A-Za-z0-9]{12})", path
            )
            if match:
                return self._json(
                    200, self.server.application.map_import_descriptor(match.group(1))
                )
            match = re.fullmatch(
                r"/api/v1/edge/recordings/([A-Za-z0-9-]+)/files/(.+)", path
            )
            if match:
                target = self.server.application.recording_export_file(
                    match.group(1), unquote(match.group(2))
                )
                return self._stream_file(target)
            if path.startswith("/artifacts/"):
                return self._artifact(path)
            return self._static(path)
        except KeyError:
            self._json(404, {"error": "not found"})
        except Exception as exc:
            self._json(500, {"error": str(exc)})

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            if path == "/api/v1/sessions/start":
                return self._json(201, self.server.application.start_recording())
            if path == "/api/v1/navigation/prepare":
                body = self._body()
                return self._json(201, self.server.application.prepare_navigation(str(body.get("job_id", ""))))
            match = re.fullmatch(r"/api/v1/map-jobs/(map-[A-Za-z0-9]{12})/label", path)
            if match:
                body = self._body()
                return self._json(
                    200,
                    self.server.application.update_map_label(
                        match.group(1), str(body.get("label") or "")
                    ),
                )
            match = re.fullmatch(r"/api/v1/map-jobs/(map-[A-Za-z0-9]{12})/retry", path)
            if match:
                return self._json(202, self.server.application.retry_map_job(match.group(1)))
            if path == "/api/v1/navigation/runtime/start":
                body = self._body()
                return self._json(200, self.server.application.start_navigation_runtime(str(body.get("candidate_id", ""))))
            if path == "/api/v1/navigation/runtime/stop":
                return self._json(200, self.server.application.stop_navigation_runtime())
            if path == "/api/v1/navigation/runtime/recover":
                return self._json(202, self.server.application.recover_navigation_runtime())
            if path == "/api/v1/navigation/localization/reset":
                return self._json(200, self.server.application.reset_localization())
            if path == "/api/v1/navigation/patrol/start":
                return self._json(200, self.server.application.start_patrol())
            if path == "/api/v1/navigation/patrol/stop":
                return self._json(200, self.server.application.stop_patrol())
            if path == "/api/v1/navigation/profile":
                body = self._body()
                return self._json(200, self.server.application.update_navigation_profile(dict(body.get("profile") or {})))
            if path == "/api/v1/diagnostics/profile":
                body = self._body()
                return self._json(200, self.server.application.update_diagnostic_profile(dict(body.get("profile") or {})))
            if path == "/api/v1/incidents/capture":
                return self._json(202, self.server.application.capture_incident())
            if path == "/api/v1/navigation/profile/rollback":
                return self._json(200, self.server.application.rollback_navigation_profile())
            match = re.fullmatch(
                r"/api/v1/edge/map-imports/(map-[A-Za-z0-9]{12})/commit", path
            )
            if match:
                return self._json(
                    200,
                    self.server.application.commit_map_import(
                        match.group(1), self._body()
                    ),
                )
            match = re.fullmatch(r"/api/v1/sessions/([A-Za-z0-9-]+)/stop", path)
            if match:
                return self._json(200, self.server.application.stop_recording(match.group(1)))
            match = re.fullmatch(r"/api/v1/sessions/([A-Za-z0-9-]+)/map", path)
            if match:
                return self._json(202, self.server.application.submit_recording(match.group(1)))
            self._json(404, {"error": "not found"})
        except Exception as exc:
            self._json(409, {"error": str(exc)})

    def do_PUT(self) -> None:
        path = urlparse(self.path).path
        try:
            match = re.fullmatch(
                r"/api/v1/edge/map-imports/(map-[A-Za-z0-9]{12})/artifacts/"
                r"([A-Za-z0-9][A-Za-z0-9_.-]{0,127})",
                path,
            )
            if not match:
                return self._json(404, {"error": "not found"})
            length = int(self.headers.get("Content-Length", "-1"))
            digest = self.headers.get("X-Content-SHA256", "").strip().lower()
            result = self.server.application.receive_map_artifact(
                match.group(1), match.group(2), self.rfile, length, digest
            )
            self._json(201, result)
        except Exception as exc:
            self._json(409, {"error": str(exc)})

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length < 0 or length > 1024 * 1024:
            raise ValueError("request body is too large")
        if length == 0:
            return {}
        payload = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("request body must be a JSON object")
        return payload

    def _artifact(self, request_path: str) -> None:
        parts = request_path.strip("/").split("/")
        if len(parts) != 3 or not re.fullmatch(r"map-[A-Za-z0-9]+", parts[1]):
            return self._json(404, {"error": "not found"})
        root = (self.server.application.data_root / "map-jobs" / parts[1] / "artifacts").resolve()
        target = (root / parts[2]).resolve()
        if root not in target.parents or not target.is_file():
            return self._json(404, {"error": "not found"})
        self._file(target)

    def _static(self, request_path: str) -> None:
        name = "index.html" if request_path == "/" else request_path.lstrip("/")
        target = (self.server.static_root / name).resolve()
        if self.server.static_root not in target.parents or not target.is_file():
            target = self.server.static_root / "index.html"
        self._file(target)

    def _file(self, target: Path) -> None:
        self._stream_file(target)

    def _stream_file(self, target: Path) -> None:
        size = target.stat().st_size
        content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        start, end = 0, max(size - 1, 0)
        status = 200
        range_header = self.headers.get("Range", "")
        match = re.fullmatch(r"bytes=(\d+)-(\d*)", range_header)
        if match and size:
            start = int(match.group(1))
            end = int(match.group(2)) if match.group(2) else size - 1
            if start >= size or end < start:
                self.send_response(416)
                self.send_header("Content-Range", f"bytes */{size}")
                self.end_headers()
                return
            end = min(end, size - 1)
            status = 206
        length = 0 if size == 0 else end - start + 1
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(length))
        self.send_header("Accept-Ranges", "bytes")
        if status == 206:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        with target.open("rb") as handle:
            handle.seek(start)
            remaining = length
            while remaining:
                chunk = handle.read(min(1024 * 1024, remaining))
                if not chunk:
                    break
                self.wfile.write(chunk)
                remaining -= len(chunk)

    def _json(self, status: int, value) -> None:
        content = json.dumps(
            json_ready(value),
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        ).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format: str, *args) -> None:
        return
