from __future__ import annotations

import argparse
import fcntl
import json
import os
import signal
import socketserver
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Callable

from .manager import NavigationManager


class OperationRegistry:
    def __init__(self) -> None:
        self._items: dict[str, dict[str, Any]] = {}
        self._active_by_kind: dict[str, str] = {}
        self._lock = threading.Lock()

    def submit(self, kind: str, action: Callable[[], Any], *, lane: str = "control") -> dict[str, Any]:
        with self._lock:
            active_id = self._active_by_kind.get(lane)
            active = self._items.get(active_id or "")
            if active and active["state"] in {"accepted", "running"}:
                return dict(active)
            operation_id = f"op-{uuid.uuid4().hex[:12]}"
            item = {
                "schema": "gogoguard.operation.v1",
                "operationId": operation_id,
                "kind": kind,
                "state": "accepted",
                "message": "请求已接收",
                "createdAt": time.time(),
                "updatedAt": time.time(),
            }
            self._items[operation_id] = item
            self._active_by_kind[lane] = operation_id

        def run() -> None:
            self._change(operation_id, state="running", message="正在执行")
            try:
                result = action()
            except Exception as exc:
                self._change(operation_id, state="failed", message=str(exc), error=str(exc))
            else:
                completion_messages = {
                    "runtime.start": "运行进程启动请求已完成；定位是否可用以实时状态为准",
                    "patrol.start": "Nav2 已接收巡检请求；整条路线结果以巡检状态为准",
                    "patrol.start_selected": "已启动选定的地图与路线，Nav2 已接收巡检请求",
                    "runtime.stop": "导航速度链和 SDK 运动桥已停止；遥控权已释放",
                    "patrol.stop": "巡检、导航速度链和 SDK 运动桥已停止；遥控权已释放",
                }
                message = completion_messages.get(kind, "请求已完成")
                if kind in {"runtime.stop", "patrol.stop"} and isinstance(result, dict):
                    if not result.get("remoteControlReleased"):
                        error = "导航进程未完全停止，遥控权释放未确认"
                        self._change(
                            operation_id,
                            state="failed",
                            message=error,
                            error=error,
                            result=result,
                        )
                        return
                    if not result.get("stopMoveConfirmed"):
                        message = (
                            "导航速度链和 SDK 运动桥已停止，遥控权已释放；"
                            "Unitree StopMove 回执未确认"
                        )
                self._change(
                    operation_id,
                    state="complete",
                    message=message,
                    result=result,
                )

        threading.Thread(target=run, name=operation_id, daemon=True).start()
        return dict(item)

    def _change(self, operation_id: str, **values: Any) -> None:
        with self._lock:
            self._items[operation_id].update(values, updatedAt=time.time())

    def snapshot(self) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(item) for item in sorted(
                self._items.values(), key=lambda value: value["createdAt"], reverse=True
            )[:20]]


class SupervisorService:
    def __init__(self, manager: NavigationManager) -> None:
        self.manager = manager
        self.operations = OperationRegistry()
        self._control_epoch = 0
        self._control_lock = threading.Lock()

    def _epoch(self) -> int:
        with self._control_lock:
            return self._control_epoch

    def _invalidate_control(self) -> int:
        with self._control_lock:
            self._control_epoch += 1
            return self._control_epoch

    def _require_current(self, expected_epoch: int) -> None:
        if self._epoch() != expected_epoch:
            raise RuntimeError("操作已被停止并释放遥控权请求取消")

    def dispatch(self, method: str, params: dict[str, Any]) -> Any:
        if method == "status":
            status = self.manager.status()
            status["supervisor"] = {"running": True, "pid": os.getpid()}
            status["operations"] = self.operations.snapshot()
            return status
        if method == "prepare":
            return self.manager.prepare(str(params.get("job_id", "")))
        if method == "profile.get":
            return self.manager.profile()
        if method == "profile.update":
            return self.manager.update_profile(dict(params.get("profile") or {}))
        if method == "profile.rollback":
            return self.manager.rollback_profile()
        if method == "diagnostics":
            return self.manager.diagnostics()
        if method in {"runtime.stop", "patrol.stop"}:
            self._invalidate_control()
        control_epoch = self._epoch()
        actions = {
            "runtime.start": lambda: self._start_runtime(
                str(params.get("candidate_id", "")), control_epoch
            ),
            "runtime.stop": self.manager.stop_runtime,
            "localization.reset": self.manager.reset_localization,
            "patrol.start": lambda: self._start_patrol(control_epoch),
            "patrol.start_selected": lambda: self._start_selected_patrol(
                params, control_epoch
            ),
            "patrol.stop": self.manager.stop_patrol,
            "runtime.recover": lambda: self._recover(control_epoch),
        }
        if method in actions:
            lane = "stop" if method in {"runtime.stop", "patrol.stop"} else "control"
            return self.operations.submit(method, actions[method], lane=lane)
        raise ValueError(f"unknown supervisor method: {method}")

    def _start_runtime(self, candidate_id: str, control_epoch: int) -> dict[str, Any]:
        self._require_current(control_epoch)
        result = self.manager.start_runtime(candidate_id)
        self._require_current(control_epoch)
        return result

    def _start_patrol(self, control_epoch: int) -> dict[str, Any]:
        self._require_current(control_epoch)
        result = self.manager.start_patrol()
        self._require_current(control_epoch)
        return result

    def _start_selected_patrol(
        self, params: dict[str, Any], control_epoch: int
    ) -> dict[str, Any]:
        self._require_current(control_epoch)
        result = self.manager.start_selected_patrol(
            expected_map_version=params.get("expected_map_version"),
            expected_route_id=params.get("expected_route_id"),
            mission_plan=params.get("mission_plan"),
        )
        self._require_current(control_epoch)
        return result

    def _recover(self, control_epoch: int) -> dict[str, Any]:
        self._require_current(control_epoch)
        candidate = self.manager.status().get("candidate")
        if not candidate:
            raise RuntimeError("尚未选择地图与路线")
        self.manager.stop_runtime()
        self._require_current(control_epoch)
        result = self.manager.start_runtime(str(candidate["candidate_id"]))
        self._require_current(control_epoch)
        return result


class _RequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        try:
            request = json.loads(self.rfile.readline(1024 * 1024).decode("utf-8"))
            result = self.server.service.dispatch(
                str(request.get("method", "")), dict(request.get("params") or {})
            )
            response = {"ok": True, "result": result}
        except Exception as exc:
            response = {"ok": False, "error": str(exc)}
        self.wfile.write((json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8"))


class NavigationSupervisorServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True

    def __init__(self, socket_path: Path, service: SupervisorService) -> None:
        self.socket_path = Path(socket_path)
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        self.socket_path.unlink(missing_ok=True)
        self.service = service
        super().__init__(str(self.socket_path), _RequestHandler)

    def server_close(self) -> None:
        super().server_close()
        self.socket_path.unlink(missing_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="single owner for the navigation runtime")
    parser.add_argument("--data-root", type=Path, default=Path("/var/lib/gogoguard"))
    parser.add_argument("--socket", type=Path, default=Path("/var/lib/gogoguard/navigation/supervisor.sock"))
    parser.add_argument("--site-id", default="local-first-site")
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--sensor-id", required=True)
    args = parser.parse_args()
    lock_path = args.socket.with_suffix(".lock")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_handle = lock_path.open("w")
    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise SystemExit("navigation supervisor is already running") from exc
    manager = NavigationManager(
        args.data_root, site_id=args.site_id, robot_id=args.robot_id, sensor_id=args.sensor_id
    )
    server = NavigationSupervisorServer(args.socket, SupervisorService(manager))
    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        manager.close()


if __name__ == "__main__":
    main()
