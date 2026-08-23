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
from gogoguard_navigation.checkpoint_alignment import (
    plan_checkpoint_view_alignment,
)


class CheckpointRuntimeTest(unittest.TestCase):
    def test_recorded_view_uses_camera_without_turning_body_when_reachable(self) -> None:
        alignment = plan_checkpoint_view_alignment(
            recorded_body_yaw_rad=math.radians(-60.0),
            recorded_camera_pan_deg=0.0,
            current_body_yaw_rad=math.radians(10.0),
            pan_limit_deg=110.0,
        )
        self.assertEqual(alignment.mode, "camera_only")
        self.assertAlmostEqual(alignment.body_turn_rad, 0.0)
        self.assertAlmostEqual(alignment.camera_pan_deg, 70.0)
        self.assertAlmostEqual(
            alignment.target_body_yaw_rad
            - math.radians(alignment.camera_pan_deg),
            alignment.desired_view_yaw_rad,
        )

    def test_view_outside_preferred_pan_uses_minimum_body_turn(self) -> None:
        alignment = plan_checkpoint_view_alignment(
            recorded_body_yaw_rad=math.radians(-150.0),
            recorded_camera_pan_deg=0.0,
            current_body_yaw_rad=0.0,
            pan_limit_deg=110.0,
        )
        self.assertEqual(alignment.mode, "body_plus_camera")
        self.assertAlmostEqual(math.degrees(alignment.body_turn_rad), -40.0)
        self.assertAlmostEqual(alignment.camera_pan_deg, 110.0)

    def test_hard_camera_range_can_finish_a_stalled_body_turn(self) -> None:
        preferred = plan_checkpoint_view_alignment(
            recorded_body_yaw_rad=math.radians(-150.0),
            recorded_camera_pan_deg=0.0,
            current_body_yaw_rad=0.0,
            pan_limit_deg=110.0,
        )
        fallback = plan_checkpoint_view_alignment(
            recorded_body_yaw_rad=math.radians(-150.0),
            recorded_camera_pan_deg=0.0,
            current_body_yaw_rad=math.radians(-10.0),
            pan_limit_deg=140.0,
        )
        self.assertLess(preferred.body_turn_rad, 0.0)
        self.assertAlmostEqual(fallback.body_turn_rad, 0.0)
        self.assertAlmostEqual(fallback.camera_pan_deg, 140.0)

    def test_positive_z1pro_pan_is_camera_right(self) -> None:
        alignment = plan_checkpoint_view_alignment(
            recorded_body_yaw_rad=0.0,
            recorded_camera_pan_deg=45.0,
            current_body_yaw_rad=0.0,
            pan_limit_deg=110.0,
        )
        self.assertAlmostEqual(
            math.degrees(alignment.desired_view_yaw_rad), -45.0
        )
        self.assertAlmostEqual(alignment.camera_pan_deg, 45.0)

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

    def test_platform_checkpoint_waits_for_verdict_and_deduplicates_control(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            payload = {
                "schema": "gogoguard.navigation_mission.v1",
                "missionId": "mission-2",
                "mapVersion": "map-v1",
                "routeId": "route-v1",
                "verdictTimeoutSec": 15,
                "maxRetakeAttempts": 2,
                "evidenceTransactionVersion": 1,
                "checkpoints": [
                    {
                        "checkpointId": "cp_01",
                        "routeProgressIndex": 10,
                        "action": "platform_checkpoint",
                        "settleBeforeS": 0.5,
                        "bodyYawRad": 1.2,
                        "camera": {"pan": -15, "tilt": 22.5, "roll": 0},
                        "spin": True,
                        "dwellSec": 3,
                    }
                ],
            }
            payload["missionHash"] = hashlib.sha256(
                json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest()
            path = Path(temporary) / "mission.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            mission = load_navigation_mission(
                path,
                expected_map_version="map-v1",
                expected_route_id="route-v1",
                route_point_count=20,
            )
        executor = CheckpointExecutor(mission)
        self.assertEqual(executor.status()["evidenceTransactionVersion"], 1)
        executor.observe_progress(10)
        executor.route_goal_cancelled()
        executor.observe_stop(
            1.0, motion_authorized=False, command_linear_mps=0,
            command_angular_rps=0, robot_linear_mps=0, robot_angular_rps=0,
            route_progress_index=10,
        )
        executor.observe_stop(
            1.5, motion_authorized=False, command_linear_mps=0,
            command_angular_rps=0, robot_linear_mps=0, robot_angular_rps=0,
            route_progress_index=10,
        )
        self.assertEqual(executor.phase, "POSE_REQUESTED")
        executor.pose_completed()
        failed_control = {
            "controlId": "cc-capture-failed",
            "missionId": "mission-2",
            "checkpointId": "cp_01",
            "attempt": 1,
            "action": "capture_failed",
        }
        self.assertEqual(
            executor.apply_platform_control(failed_control), "capture_failed"
        )
        self.assertEqual(executor.phase, "WAITING_VERDICT")
        self.assertEqual(
            executor.apply_platform_control(
                {**failed_control, "controlId": "cc-retake", "action": "retake"}
            ),
            "retake",
        )
        self.assertEqual(executor.phase, "WAITING_PLATFORM")
        control = {
            "controlId": "cc-capture",
            "missionId": "mission-2",
            "checkpointId": "cp_01",
            "attempt": 2,
            "action": "capture",
        }
        self.assertEqual(executor.apply_platform_control(control), "capture")
        self.assertEqual(executor.apply_platform_control(control), "duplicate")
        executor.spin_started()
        executor.spin_completed()
        self.assertEqual(executor.phase, "WAITING_VERDICT")
        self.assertEqual(
            executor.apply_platform_control(
                {**control, "controlId": "cc-continue", "action": "continue"}
            ),
            "continue",
        )
        self.assertEqual(executor.phase, "TRAVELING")

    def test_platform_checkpoint_rejects_non_finite_camera_angle(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            payload = {
                "schema": "gogoguard.navigation_mission.v1",
                "missionId": "mission-2",
                "mapVersion": "map-v1",
                "routeId": "route-v1",
                "checkpoints": [
                    {
                        "checkpointId": "cp_01",
                        "routeProgressIndex": 10,
                        "action": "platform_checkpoint",
                        "settleBeforeS": 0.5,
                        "bodyYawRad": 1.2,
                        "camera": {"pan": math.nan, "tilt": 0, "roll": 0},
                    }
                ],
            }
            payload["missionHash"] = hashlib.sha256(
                json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest()
            path = Path(temporary) / "mission.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "camera angle"):
                load_navigation_mission(
                    path,
                    expected_map_version="map-v1",
                    expected_route_id="route-v1",
                    route_point_count=20,
                )

    def test_local_operator_mode_is_explicit_in_runtime_status(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            payload = {
                "schema": "gogoguard.navigation_mission.v1",
                "missionId": "local:map-v1:1:abcd",
                "mapVersion": "map-v1",
                "routeId": "route-v1",
                "decisionMode": "local_operator",
                "checkpoints": [],
            }
            payload["missionHash"] = hashlib.sha256(
                json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest()
            path = Path(temporary) / "mission.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            mission = load_navigation_mission(
                path,
                expected_map_version="map-v1",
                expected_route_id="route-v1",
                route_point_count=20,
            )
        self.assertEqual(mission.decision_mode, "local_operator")
        self.assertEqual(
            CheckpointExecutor(mission).status()["decisionMode"], "local_operator"
        )


if __name__ == "__main__":
    unittest.main()
