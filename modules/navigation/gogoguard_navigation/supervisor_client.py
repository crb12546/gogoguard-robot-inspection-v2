from __future__ import annotations

import json
import socket
from pathlib import Path
from typing import Any


class NavigationSupervisorClient:
    """Small local RPC client. Closing the web application never stops Nav2."""

    def __init__(self, socket_path: Path, *, timeout_s: float = 15.0) -> None:
        self.socket_path = Path(socket_path)
        self.timeout_s = timeout_s

    def close(self) -> None:
        return None

    def _call(self, method: str, **params: Any) -> Any:
        request = json.dumps({"method": method, "params": params}, ensure_ascii=False).encode("utf-8") + b"\n"
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.settimeout(self.timeout_s)
                client.connect(str(self.socket_path))
                client.sendall(request)
                with client.makefile("rb") as stream:
                    raw = stream.readline(1024 * 1024)
        except OSError as exc:
            raise RuntimeError(f"导航管控进程不可用: {exc}") from exc
        response = json.loads(raw.decode("utf-8"))
        if response.get("ok") is not True:
            raise RuntimeError(str(response.get("error") or "supervisor request failed"))
        return response.get("result")

    def status(self) -> dict[str, Any]: return self._call("status")
    def prepare(self, job_id: str) -> dict[str, Any]: return self._call("prepare", job_id=job_id)
    def start_runtime(self, candidate_id: str) -> dict[str, Any]: return self._call("runtime.start", candidate_id=candidate_id)
    def stop_runtime(self) -> dict[str, Any]: return self._call("runtime.stop")
    def reset_localization(self) -> dict[str, Any]: return self._call("localization.reset")
    def start_patrol(self) -> dict[str, Any]: return self._call("patrol.start")
    def start_selected_patrol(
        self,
        *,
        expected_map_version: str | None = None,
        expected_route_id: str | None = None,
        mission_plan: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        return self._call(
            "patrol.start_selected",
            expected_map_version=expected_map_version,
            expected_route_id=expected_route_id,
            mission_plan=mission_plan,
        )
    def stop_patrol(self) -> dict[str, Any]: return self._call("patrol.stop")
    def checkpoint_control(self, action: str) -> dict[str, Any]:
        return self._call("checkpoint.control", action=action)
    def recover_runtime(self) -> dict[str, Any]: return self._call("runtime.recover")
    def profile(self) -> dict[str, Any]: return self._call("profile.get")
    def update_profile(self, profile: dict[str, Any]) -> dict[str, Any]: return self._call("profile.update", profile=profile)
    def rollback_profile(self) -> dict[str, Any]: return self._call("profile.rollback")
    def diagnostics(self) -> dict[str, Any]: return self._call("diagnostics")
