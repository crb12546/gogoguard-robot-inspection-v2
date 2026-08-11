from __future__ import annotations

import hashlib
import json
import math
import tempfile
import unittest
from pathlib import Path

from gogoguard_navigation.checkpoint_runtime import (
    CheckpointExecutor,
    load_navigation_mission,
)


class CheckpointRuntimeTest(unittest.TestCase):
    def mission(self, root: Path) -> Path:
        payload = {
            "schema": "gogoguard.navigation_mission.v1",
            "missionId": "mission-1",
            "mapVersion": "map-v1",
            "routeId": "route-v1",
            "checkpoints": [
                {
                    "checkpointId": "point-1",
                    "routeProgressIndex": 10,
                    "action": "body_spin_360",
                    "settleBeforeS": 0.5,
                    "targetYawRad": round(2.0 * math.pi, 9),
                }
            ],
        }
        payload["missionHash"] = hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        path = root / "mission.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_true_stop_spin_and_resume_sequence(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            mission = load_navigation_mission(
                self.mission(Path(temporary)),
                expected_map_version="map-v1",
                expected_route_id="route-v1",
                route_point_count=20,
            )
        executor = CheckpointExecutor(mission)
        self.assertFalse(executor.observe_progress(9))
        self.assertTrue(executor.observe_progress(10))
        executor.route_goal_cancelled()
        self.assertFalse(
            executor.observe_stop(
                1.0,
                motion_authorized=False,
                command_linear_mps=0.0,
                command_angular_rps=0.0,
                robot_linear_mps=0.0,
                robot_angular_rps=0.0,
                route_progress_index=10,
            )
        )
        self.assertTrue(
            executor.observe_stop(
                1.5,
                motion_authorized=False,
                command_linear_mps=0.0,
                command_angular_rps=0.0,
                robot_linear_mps=0.0,
                robot_angular_rps=0.0,
                route_progress_index=10,
            )
        )
        self.assertTrue(executor.last_stop_receipt["stopped"])
        self.assertEqual(
            executor.last_stop_receipt["pauseRequestId"],
            "mission-1:point-1:0",
        )
        self.assertTrue(executor.last_stop_receipt["observedAt"])
        executor.spin_started()
        executor.spin_completed()
        self.assertEqual(executor.status()["completedCheckpointCount"], 1)
        self.assertEqual(executor.phase, "TRAVELING")

    def test_moving_robot_cannot_produce_stop_receipt(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            mission = load_navigation_mission(
                self.mission(Path(temporary)),
                expected_map_version="map-v1",
                expected_route_id="route-v1",
                route_point_count=20,
            )
        executor = CheckpointExecutor(mission)
        executor.observe_progress(10)
        executor.route_goal_cancelled()
        for now in (1.0, 2.0):
            self.assertFalse(
                executor.observe_stop(
                    now,
                    motion_authorized=False,
                    command_linear_mps=0.0,
                    command_angular_rps=0.0,
                    robot_linear_mps=0.03,
                    robot_angular_rps=0.0,
                    route_progress_index=10,
                )
            )
        self.assertIsNone(executor.last_stop_receipt)


if __name__ == "__main__":
    unittest.main()
