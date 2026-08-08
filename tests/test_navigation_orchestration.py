import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "modules/navigation"))
sys.path.insert(0, str(ROOT / "modules/route"))

from gogoguard_navigation.orchestration import (
    ControllerMode,
    ControllerSuccessAction,
    FailureClass,
    FailureEvidence,
    MotionEvidenceTracker,
    MotionSnapshot,
    RecoveryAction,
    controller_success_action,
    decide_controller_failure,
    route_obstruction_evidence,
)


class Grid:
    def __init__(self, width=20, height=20, resolution=0.1):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.values = [0] * (width * height)

    def cell(self, x, y):
        return int(x / self.resolution), int(y / self.resolution)

    def inside(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def cost(self, cell):
        return self.values[cell[1] * self.width + cell[0]]

    def occupy(self, x, y, cost=100):
        cell = self.cell(x, y)
        self.values[cell[1] * self.width + cell[0]] = cost


def motion(translation=0.5, linear=0.4, rotation=0.0, angular=0.0):
    return MotionSnapshot(
        window_s=2.5,
        translation_m=translation,
        rotation_rad=rotation,
        mean_linear_command_mps=linear,
        mean_angular_command_rps=angular,
    )


class NavigationOrchestrationTest(unittest.TestCase):
    def test_clear_route_abort_retries_mppi_instead_of_detour(self):
        decision = decide_controller_failure(
            FailureEvidence(
                controller=ControllerMode.MPPI,
                localization_usable=True,
                route_obstructed=False,
                motion=motion(),
                mppi_retry_count=0,
            )
        )
        self.assertEqual(decision.failure_class, FailureClass.TRANSIENT_CONTROL)
        self.assertEqual(decision.action, RecoveryAction.RETRY_MPPI)

    def test_verified_route_obstruction_authorizes_bounded_detour(self):
        decision = decide_controller_failure(
            FailureEvidence(
                controller=ControllerMode.MPPI,
                localization_usable=True,
                route_obstructed=True,
                motion=motion(),
                mppi_retry_count=0,
            )
        )
        self.assertEqual(decision.failure_class, FailureClass.PATH_OBSTRUCTED)
        self.assertEqual(decision.action, RecoveryAction.START_DETOUR)

    def test_obstruction_sequence_returns_controller_ownership_to_mppi(self):
        failure = decide_controller_failure(
            FailureEvidence(
                controller=ControllerMode.MPPI,
                localization_usable=True,
                route_obstructed=True,
                motion=motion(),
                mppi_retry_count=0,
            )
        )
        self.assertEqual(failure.action, RecoveryAction.START_DETOUR)
        self.assertEqual(
            controller_success_action(ControllerMode.DETOUR_RPP),
            ControllerSuccessAction.RESUME_MPPI_SUFFIX,
        )
        self.assertEqual(
            controller_success_action(ControllerMode.MPPI),
            ControllerSuccessAction.COMPLETE_ROUTE,
        )

    def test_map_8dd_incident_is_actuation_stall_not_path_blocked(self):
        decision = decide_controller_failure(
            FailureEvidence(
                controller=ControllerMode.DETOUR_RPP,
                localization_usable=True,
                route_obstructed=False,
                motion=motion(translation=0.051, linear=0.24),
                mppi_retry_count=0,
            )
        )
        self.assertEqual(decision.failure_class, FailureClass.ACTUATION_STALL)
        self.assertEqual(decision.action, RecoveryAction.STOP_FAULT)
        self.assertEqual(decision.reason, "ACTUATION_STALL")

    def test_detour_failure_never_recursively_starts_another_detour(self):
        decision = decide_controller_failure(
            FailureEvidence(
                controller=ControllerMode.DETOUR_RPP,
                localization_usable=True,
                route_obstructed=True,
                motion=motion(translation=0.3, linear=0.35),
                mppi_retry_count=0,
            )
        )
        self.assertEqual(decision.action, RecoveryAction.STOP_BLOCKED)

    def test_retry_budget_exhaustion_is_controller_failure(self):
        decision = decide_controller_failure(
            FailureEvidence(
                controller=ControllerMode.MPPI,
                localization_usable=True,
                route_obstructed=False,
                motion=motion(),
                mppi_retry_count=2,
            )
        )
        self.assertEqual(decision.failure_class, FailureClass.CONTROLLER_FAILED)
        self.assertEqual(decision.action, RecoveryAction.STOP_FAULT)

    def test_pure_turn_progress_is_not_actuation_stall(self):
        snapshot = motion(
            translation=0.01,
            linear=0.0,
            rotation=0.3,
            angular=0.4,
        )
        self.assertFalse(snapshot.has_actuation_stall())

    def test_motion_tracker_reproduces_command_and_pose_window(self):
        tracker = MotionEvidenceTracker()
        for index in range(6):
            at = index * 0.5
            tracker.record_pose(at, index * 0.02, 0.0, index * 0.01)
            tracker.record_command(at, 0.24, 0.0, 0.04)
        snapshot = tracker.snapshot(2.5, window_s=2.5)
        self.assertAlmostEqual(snapshot.translation_m, 0.10)
        self.assertAlmostEqual(snapshot.rotation_rad, 0.05)
        self.assertAlmostEqual(snapshot.mean_linear_command_mps, 0.24)
        self.assertTrue(snapshot.has_actuation_stall())

    def test_route_obstruction_requires_consecutive_inflated_cells(self):
        grid = Grid()
        route = [(0.2, 0.5), (1.5, 0.5)]
        grid.occupy(0.8, 0.5)
        self.assertFalse(route_obstruction_evidence(grid, route))
        grid.occupy(0.9, 0.5)
        self.assertTrue(route_obstruction_evidence(grid, route))

    def test_out_of_window_route_is_not_obstacle_evidence(self):
        grid = Grid()
        self.assertFalse(
            route_obstruction_evidence(grid, [(3.0, 3.0), (4.0, 4.0)])
        )


if __name__ == "__main__":
    unittest.main()
