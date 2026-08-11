import tempfile
import threading
import time
import unittest
from pathlib import Path

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
    def stop_patrol(self): return self.stop_runtime()
    def reset_localization(self): return {"reset": True}
    def prepare(self, job_id): return {"job_id": job_id}
    def profile(self): return {"schema": "profile"}
    def update_profile(self, profile): return {"profile": profile}
    def rollback_profile(self): return {"rolled_back": True}
    def diagnostics(self): return {"summary": "ok"}


class NavigationSupervisorTest(unittest.TestCase):
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
