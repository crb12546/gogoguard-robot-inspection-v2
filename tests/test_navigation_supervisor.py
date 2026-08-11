import tempfile
import threading
import time
import unittest
import json
from pathlib import Path
from unittest import mock

from gogoguard_navigation.supervisor import NavigationSupervisorServer, SupervisorService
from gogoguard_navigation.supervisor_client import NavigationSupervisorClient
from gogoguard_navigation.manager import NavigationManager


class FakeManager:
    def __init__(self):
        self.starts = 0

    def status(self):
        return {"runtime_process": {"running": self.starts > 0}, "candidate": None}

    def start_runtime(self, candidate_id):
        self.starts += 1
        time.sleep(0.03)
        return {"candidate_id": candidate_id}

    def stop_runtime(self):
        return {
            "success": True,
            "remoteControlReleased": True,
            "stopMoveConfirmed": True,
        }
    def start_patrol(self): return {"started": True}
    def start_selected_patrol(self, **params):
        self.starts += 1
        return {"started": True, "params": params}
    def checkpoint_control(self, action): return {"action": action, "accepted": True}
    def stop_patrol(self): return self.stop_runtime()
    def reset_localization(self): return {"reset": True}
    def prepare(self, job_id): return {"job_id": job_id}
    def profile(self): return {"schema": "profile"}
    def update_profile(self, profile): return {"profile": profile}
    def rollback_profile(self): return {"rolled_back": True}
    def diagnostics(self): return {"summary": "ok"}


class NavigationSupervisorTest(unittest.TestCase):
    def test_selected_patrol_waits_for_current_runtime_generation(self):
        manager = NavigationManager.__new__(NavigationManager)
        candidate = {
            "candidate_id": "map-123456789abc",
            "map_version": "map-123456789abc",
            "route_id": "route-r1",
        }
        manager._loaded_candidate = lambda: candidate
        manager._configure_mission = lambda _candidate, _plan: {
            "missionId": "mission-new",
            "missionHash": "hash-new",
            "checkpoints": [],
        }
        manager._launched_mission_hash = None
        statuses = [
            {
                "runtime_process": {"running": False},
                "motion_bridge": {"running": False},
                "runtime": {"runtimeInstanceId": "generation-old"},
            },
            {
                "runtime_process": {"running": True},
                "motion_bridge": {"running": True},
                "localization": {"usable": True, "reason": "OK"},
                "runtime": {
                    "runtimeInstanceId": "generation-old",
                    "costmapHealth": {"healthy": True, "reason": "OK"},
                },
            },
            {
                "runtime_process": {"running": True},
                "motion_bridge": {"running": True},
                "localization": {"usable": True, "reason": "OK"},
                "runtime": {
                    "runtimeInstanceId": "generation-new",
                    "costmapHealth": {"healthy": True, "reason": "OK"},
                },
            },
        ]
        observed = []

        def status():
            value = statuses.pop(0)
            observed.append(value)
            return value

        manager.status = status
        runtime_starts = []
        manager.start_runtime = lambda _candidate_id, **kwargs: runtime_starts.append(
            kwargs
        ) or {}
        patrol_calls = []
        manager.start_patrol = lambda: patrol_calls.append(len(observed)) or {
            "success": True
        }

        with mock.patch("gogoguard_navigation.manager.time.sleep"):
            result = manager.start_selected_patrol(
                mission_plan={"missionId": "mission-new"},
                readiness_timeout_s=5.0,
            )

        self.assertTrue(result["accepted"])
        self.assertEqual(patrol_calls, [3])
        self.assertEqual(runtime_starts[0]["mission"]["missionHash"], "hash-new")

    def test_new_route_prepare_invalidates_old_bound_mission(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager = NavigationManager(
                Path(temporary), site_id="site", robot_id="robot", sensor_id="sensor"
            )
            old_mission = {
                "schema": "gogoguard.navigation_mission.v1",
                "missionHash": "old-hash",
                "mapVersion": "map-123456789abc",
                "routeId": "route-r1",
            }
            manager._atomic_json(manager.mission_path, old_mission)
            candidate = {
                "candidate_id": "map-123456789abc",
                "map_version": "map-123456789abc",
                "route_id": "route-r5",
            }

            class Routes:
                @staticmethod
                def prepare_map_job(_job_id):
                    return candidate

            manager.routes = Routes()
            manager.prepare("job-1")

            self.assertFalse(manager.mission_path.exists())
            self.assertEqual(manager._loaded_candidate()["route_id"], "route-r5")

    def test_plain_runtime_start_never_inherits_persisted_mission(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager = NavigationManager(
                Path(temporary), site_id="site", robot_id="robot", sensor_id="sensor"
            )
            manager._atomic_json(
                manager.mission_path,
                {
                    "schema": "gogoguard.navigation_mission.v1",
                    "missionHash": "old-hash",
                    "mapVersion": "map-123456789abc",
                    "routeId": "route-r1",
                },
            )
            candidate = {
                "map_version": "map-123456789abc",
                "route_id": "route-r5",
            }

            self.assertIsNone(manager._mission_path_for_launch(candidate, None))

    def test_patrol_clear_requires_fresh_costmap_sequence(self):
        manager = NavigationManager.__new__(NavigationManager)
        manager._loaded_candidate = lambda: {
            "map_version": "map-123456789abc",
            "route_id": "route-r1",
        }

        class Running:
            @staticmethod
            def poll():
                return None

        manager._process = Running()
        manager._receiver_process = Running()
        manager.status = lambda: {
            "runtime_process": {"running": True},
            "runtime": {
                "costmapHealth": {
                    "sequence": 7,
                    "healthy": True,
                    "reason": "OK",
                }
            },
        }
        manager._service = lambda *_args, **_kwargs: {"success": True}

        with mock.patch(
            "gogoguard_navigation.manager.time.monotonic",
            side_effect=[0.0, 1.0, 5.0],
        ):
            with self.assertRaisesRegex(RuntimeError, "COSTMAP_REFRESH_PENDING"):
                manager.start_patrol()

    def test_checkpoint_mission_is_bound_to_execution_route_and_360_spin(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager = NavigationManager(
                Path(temporary), site_id="site", robot_id="robot", sensor_id="sensor"
            )
            candidate = {
                "map_version": "map-123456789abc",
                "route_id": "route-r1",
                "workspace_revision": 6,
                "execution_route_point_count": 100,
            }
            mission = manager._configure_mission(
                candidate,
                {
                    "missionId": "mission-1",
                    "mapVersion": "map-123456789abc",
                    "routeId": "route-r1",
                    "checkpoints": [
                        {"checkpointId": "point-1", "routeProgressIndex": 42}
                    ],
                },
            )
            self.assertEqual(mission["checkpoints"][0]["action"], "body_spin_360")
            self.assertAlmostEqual(mission["checkpoints"][0]["targetYawRad"], 6.283185307)
            self.assertEqual(len(mission["missionHash"]), 64)
            with self.assertRaisesRegex(ValueError, "outside execution route"):
                manager._configure_mission(
                    candidate,
                    {
                        "missionId": "mission-2",
                        "checkpoints": [
                            {"checkpointId": "point-2", "routeProgressIndex": 100}
                        ],
                    },
                )

    def test_activated_checkpoint_asset_is_authoritative_for_route_and_pose(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manager = NavigationManager(
                root, site_id="site", robot_id="robot", sensor_id="sensor"
            )
            asset_path = root / "checkpoints.json"
            asset_path.write_text(
                json.dumps(
                    {
                        "schema": "gogoguard.checkpoints.v1",
                        "mapVersion": "map-123456789abc",
                        "routeId": "route-r1",
                        "checkpoints": [
                            {
                                "checkpointId": "cp_01",
                                "routeProgressIndex": 42,
                                "bodyYaw": 1.2,
                                "camera": {"pan": -15, "tilt": 22.5, "roll": 0},
                                "spin": True,
                                "dwellSec": 3,
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            candidate = {
                "map_version": "map-123456789abc",
                "route_id": "route-r1",
                "workspace_revision": 6,
                "execution_route_point_count": 100,
                "checkpoint_asset": str(asset_path),
            }
            plan = {
                "missionId": "mission-1",
                "mapVersion": "map-123456789abc",
                "routeId": "route-r1",
                "checkpoints": [
                    {"checkpointId": "cp_01", "routeProgressIndex": 42}
                ],
            }
            mission = manager._configure_mission(candidate, plan)
            self.assertEqual(mission["checkpoints"][0]["bodyYawRad"], 1.2)
            self.assertEqual(mission["checkpoints"][0]["camera"]["pan"], -15.0)
            plan["checkpoints"][0]["routeProgressIndex"] = 43
            with self.assertRaisesRegex(ValueError, "differs from the activated asset"):
                manager._configure_mission(candidate, plan)
            plan["checkpoints"][0]["routeProgressIndex"] = 42
            plan["checkpoints"][0]["dwellSec"] = "3"
            with self.assertRaisesRegex(ValueError, "dwellSec"):
                manager._configure_mission(candidate, plan)

    def test_local_checkpoint_control_is_scoped_and_writes_correlated_control(self):
        with tempfile.TemporaryDirectory() as temporary:
            manager = NavigationManager(
                Path(temporary), site_id="site", robot_id="robot", sensor_id="sensor"
            )
            manager.status = lambda: {
                "runtime": {
                    "checkpoint": {
                        "decisionMode": "local_operator",
                        "missionId": "local:map-v1:1:abcd",
                        "activeCheckpointId": "cp_01",
                        "attempt": 1,
                        "phase": "WAITING_VERDICT",
                    }
                }
            }
            receipt = manager.checkpoint_control("continue")
            self.assertTrue(receipt["accepted"])
            control = json.loads(manager.checkpoint_control_path.read_text())
            self.assertEqual(control["action"], "continue")
            self.assertEqual(control["checkpointId"], "cp_01")
            self.assertEqual(control["missionId"], "local:map-v1:1:abcd")

            manager.status = lambda: {
                "runtime": {"checkpoint": {"decisionMode": "platform"}}
            }
            with self.assertRaisesRegex(RuntimeError, "不是本地"):
                manager.checkpoint_control("continue")

    def test_long_operation_returns_receipt_and_is_deduplicated(self):
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "supervisor.sock"
            manager = FakeManager()
            server = NavigationSupervisorServer(socket_path, SupervisorService(manager))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            client = NavigationSupervisorClient(socket_path)
            first = client.start_runtime("map-123456789abc")
            second = client.start_runtime("map-123456789abc")
            self.assertEqual(first["operationId"], second["operationId"])
            self.assertIn(first["state"], {"accepted", "running"})
            deadline = time.time() + 1.0
            operation = None
            while time.time() < deadline:
                operation = client.status()["operations"][0]
                if operation["state"] == "complete":
                    break
                time.sleep(0.01)
            self.assertEqual(operation["state"], "complete")
            self.assertIn("定位是否可用以实时状态为准", operation["message"])
            self.assertEqual(manager.starts, 1)
            server.shutdown()
            server.server_close()

    def test_patrol_operation_does_not_claim_the_route_completed(self):
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "supervisor.sock"
            server = NavigationSupervisorServer(
                socket_path, SupervisorService(FakeManager())
            )
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            client = NavigationSupervisorClient(socket_path)
            client.start_patrol()
            deadline = time.time() + 1.0
            operation = None
            while time.time() < deadline:
                operation = client.status()["operations"][0]
                if operation["state"] == "complete":
                    break
                time.sleep(0.01)
            self.assertEqual(operation["state"], "complete")
            self.assertIn("整条路线结果以巡检状态为准", operation["message"])
            server.shutdown()
            server.server_close()

    def test_platform_selected_patrol_is_version_bound_and_asynchronous(self):
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "supervisor.sock"
            manager = FakeManager()
            server = NavigationSupervisorServer(
                socket_path, SupervisorService(manager)
            )
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            client = NavigationSupervisorClient(socket_path)
            accepted = client.start_selected_patrol(
                expected_map_version="map-v1", expected_route_id="route-v2"
            )
            self.assertIn(accepted["state"], {"accepted", "running", "complete"})
            deadline = time.time() + 1.0
            operation = None
            while time.time() < deadline:
                operation = client.status()["operations"][0]
                if operation["state"] == "complete":
                    break
                time.sleep(0.01)
            self.assertEqual(operation["state"], "complete")
            self.assertEqual(
                operation["result"]["params"],
                {
                    "expected_map_version": "map-v1",
                    "expected_route_id": "route-v2",
                    "mission_plan": None,
                },
            )
            server.shutdown()
            server.server_close()

    def test_local_checkpoint_control_is_synchronous(self):
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "supervisor.sock"
            server = NavigationSupervisorServer(
                socket_path, SupervisorService(FakeManager())
            )
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            client = NavigationSupervisorClient(socket_path)
            self.assertEqual(
                client.checkpoint_control("continue"),
                {"action": "continue", "accepted": True},
            )
            server.shutdown()
            server.server_close()

    def test_stop_lane_remains_available_while_start_lane_is_busy(self):
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "supervisor.sock"
            manager = FakeManager()
            server = NavigationSupervisorServer(
                socket_path, SupervisorService(manager)
            )
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            client = NavigationSupervisorClient(socket_path)
            start = client.start_runtime("map-123456789abc")
            stop = client.stop_runtime()
            self.assertNotEqual(start["operationId"], stop["operationId"])
            self.assertEqual(stop["kind"], "runtime.stop")
            deadline = time.time() + 1.0
            completed = None
            while time.time() < deadline:
                operations = client.status()["operations"]
                completed = next(
                    item for item in operations
                    if item["operationId"] == stop["operationId"]
                )
                if completed["state"] == "complete":
                    break
                time.sleep(0.01)
            self.assertEqual(completed["state"], "complete")
            self.assertIn("遥控权已释放", completed["message"])
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
