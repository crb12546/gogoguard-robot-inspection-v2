from __future__ import annotations

import json
import mimetypes
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


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
            if path == "/api/v1/sessions":
                return self._json(200, {"items": self.server.application.sessions()})
            match = re.fullmatch(r"/api/v1/sessions/([A-Za-z0-9-]+)", path)
            if match:
                return self._json(200, self.server.application.session(match.group(1)))
            match = re.fullmatch(r"/api/v1/map-jobs/([A-Za-z0-9-]+)", path)
            if match:
                return self._json(200, self.server.application.map_job(match.group(1)))
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
            match = re.fullmatch(r"/api/v1/sessions/([A-Za-z0-9-]+)/stop", path)
            if match:
                return self._json(200, self.server.application.stop_recording(match.group(1)))
            self._json(404, {"error": "not found"})
        except Exception as exc:
            self._json(409, {"error": str(exc)})

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
        content = target.read_bytes()
        content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def _json(self, status: int, value) -> None:
        content = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format: str, *args) -> None:
        return
