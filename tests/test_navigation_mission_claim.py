from __future__ import annotations

import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock

from gogoguard_navigation import NavigationManager


class NavigationMissionClaimTest(unittest.TestCase):
    def manager(
        self,
        root: Path,
        *,
        requested_id: str = "mission-new",
        requested_hash: str = "hash-new",
    ) -> NavigationManager:
        manager = NavigationManager.__new__(NavigationManager)
        manager._lock = threading.Lock()
        manager.mission_path = root / "mission-plan.json"
        manager.mission_claim_path = root / "active-mission-claim.json"
        manager._launched_mission_hash = "hash-old"
        manager._loaded_candidate = lambda: {
            "candidate_id": "map-123456789abc",
            "map_version": "map-123456789abc",
            "route_id": "route-v1",
        }
        mission = {
            "schema": "gogoguard.navigation_mission.v1",
            "missionId": requested_id,
            "missionHash": requested_hash,
            "mapVersion": "map-123456789abc",
            "routeId": "route-v1",
            "checkpoints": [],
        }
        manager._configure_mission = lambda *args, **kwargs: dict(mission)
        manager.status = lambda: {
            "runtime_process": {"running": True},
            "motion_bridge": {"running": True},
            "runtime": {"state": "PATROLLING", "runtimeInstanceId": "runtime-old"},
        }
        manager.stop_runtime = Mock()
        return manager

    @staticmethod
    def write_claim(root: Path, mission_id: str, mission_hash: str) -> None:
        (root / "active-mission-claim.json").write_text(
            json.dumps(
                {
                    "schema": "gogoguard.active_mission_claim.v1",
                    "missionId": mission_id,
                    "missionHash": mission_hash,
                    "state": "active",
                    "remoteControlReleased": False,
                }
            ),
            encoding="utf-8",
        )

    def test_different_active_mission_never_stops_or_restarts_runtime(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write_claim(root, "mission-old", "hash-old")
            manager = self.manager(root)
            with self.assertRaisesRegex(RuntimeError, "MISSION_CONFLICT_ACTIVE"):
                manager.start_selected_patrol()
            manager.stop_runtime.assert_not_called()

    def test_same_mission_with_mutated_hash_is_a_distinct_conflict(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write_claim(root, "mission-new", "hash-old")
            manager = self.manager(root)
            with self.assertRaisesRegex(RuntimeError, "MISSION_CONFLICT_MUTATED"):
                manager.start_selected_patrol()
            manager.stop_runtime.assert_not_called()

    def test_same_mission_and_hash_returns_current_runtime_idempotently(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write_claim(root, "mission-new", "hash-new")
            manager = self.manager(root)
            result = manager.start_selected_patrol()
            self.assertTrue(result["accepted"])
            self.assertTrue(result["duplicate"])
            self.assertTrue(result["runtimeRunning"])
            self.assertEqual(result["runtimeState"], "PATROLLING")
            manager.stop_runtime.assert_not_called()

    def test_plain_idle_runtime_is_reloaded_with_checkpoint_mission(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manager = self.manager(root)
            manager._launched_mission_hash = None
            events = []
            statuses = [
                {
                    "runtime_process": {"running": True},
                    "motion_bridge": {"running": True},
                    "runtime": {
                        "state": "READY",
                        "runtimeInstanceId": "runtime-plain",
                        "motionAuthorized": False,
                        "checkpoint": {
                            "missionId": None,
                            "missionHash": None,
                            "checkpointCount": 0,
                        },
                    },
                },
                {
                    "runtime_process": {"running": False},
                    "motion_bridge": {"running": False},
                    "runtime": {
                        "state": "READY",
                        "runtimeInstanceId": "runtime-plain",
                    },
                },
                {
                    "runtime_process": {"running": True},
                    "motion_bridge": {"running": True},
                    "localization": {"usable": True, "reason": "OK"},
                    "runtime": {
                        "state": "READY",
                        "runtimeInstanceId": "runtime-mission",
                        "costmapHealth": {"healthy": True, "reason": "OK"},
                    },
                },
            ]
            manager.status = lambda: statuses.pop(0)
            manager.stop_runtime = Mock(
                side_effect=lambda: events.append("stop")
                or {"remoteControlReleased": True}
            )
            manager.start_runtime = Mock(
                side_effect=lambda *args, **kwargs: events.append("start") or {}
            )
            manager.start_patrol = Mock(
                side_effect=lambda: events.append("patrol") or {"success": True}
            )

            result = manager.start_selected_patrol(readiness_timeout_s=5.0)

            self.assertTrue(result["accepted"])
            self.assertFalse(result["duplicate"])
            self.assertEqual(events, ["stop", "start", "patrol"])
            manager.start_runtime.assert_called_once()
            self.assertEqual(
                manager.start_runtime.call_args.kwargs["mission"]["missionHash"],
                "hash-new",
            )
            claim = json.loads(manager.mission_claim_path.read_text(encoding="utf-8"))
            self.assertEqual(claim["state"], "active")
            self.assertEqual(claim["missionHash"], "hash-new")

    def test_unproven_or_motion_authorized_runtime_is_never_reloaded(self) -> None:
        base = {
            "runtime_process": {"running": True},
            "runtime": {
                "state": "READY",
                "motionAuthorized": False,
                "checkpoint": {
                    "missionId": None,
                    "missionHash": None,
                    "checkpointCount": 0,
                },
            },
        }
        self.assertTrue(
            NavigationManager._plain_idle_runtime_can_reload_mission(base)
        )
        moving = json.loads(json.dumps(base))
        moving["runtime"]["motionAuthorized"] = True
        self.assertFalse(
            NavigationManager._plain_idle_runtime_can_reload_mission(moving)
        )
        incomplete = json.loads(json.dumps(base))
        incomplete["runtime"]["checkpoint"].pop("checkpointCount")
        self.assertFalse(
            NavigationManager._plain_idle_runtime_can_reload_mission(incomplete)
        )
        patrolling = json.loads(json.dumps(base))
        patrolling["runtime"]["state"] = "PATROLLING"
        self.assertFalse(
            NavigationManager._plain_idle_runtime_can_reload_mission(patrolling)
        )

    def test_explicit_stop_releases_claim_only_after_remote_control_release(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write_claim(root, "mission-old", "hash-old")
            manager = NavigationManager.__new__(NavigationManager)
            manager._lock = threading.Lock()
            manager.mission_claim_path = root / "active-mission-claim.json"
            manager._process = None
            manager._receiver_process = None
            manager._log_handle = None
            manager._receiver_log_handle = None
            manager._last_runtime_exit = None
            manager._last_receiver_exit = None
            manager._launched_mission_hash = "hash-old"
            result = manager.stop_runtime()
            self.assertTrue(result["remoteControlReleased"])
            claim = json.loads(manager.mission_claim_path.read_text(encoding="utf-8"))
            self.assertEqual(claim["state"], "released")
            self.assertTrue(claim["remoteControlReleased"])


if __name__ == "__main__":
    unittest.main()
