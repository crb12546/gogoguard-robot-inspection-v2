from __future__ import annotations

import unittest
from dataclasses import replace

from gogoguard_contracts import (
    CheckpointPlan,
    InspectionViewPlan,
    MissionPlan,
    MissionState,
    NavigationStopReceipt,
)
from gogoguard_mission import MissionCoordinator, validate_mission_plan


def mission_plan() -> MissionPlan:
    return MissionPlan(
        mission_id="mission-001",
        map_version="map-6855ba54ae11",
        route_id="route-r7",
        offline_continue_after_evidence=True,
        checkpoints=(
            CheckpointPlan(
                checkpoint_id="fire-extinguisher-1",
                route_progress_index=10,
                views=(
                    InspectionViewPlan(
                        view_id="left",
                        pan_body_deg=-60.0,
                        tilt_euler_deg=-5.0,
                    ),
                ),
            ),
        ),
    )


class MissionCoordinatorTest(unittest.TestCase):
    def test_frozen_platform_unicode_mission_id_is_accepted(self) -> None:
        plan = replace(
            mission_plan(), mission_id="mission-20260811-143022-RG-狗02"
        )
        self.assertEqual(validate_mission_plan(plan).mission_id, plan.mission_id)

    def test_checkpoint_requires_correlated_true_stop_then_resumes_suffix(self) -> None:
        coordinator = MissionCoordinator(mission_plan())
        self.assertEqual(coordinator.start().action, "navigate")
        self.assertEqual(coordinator.observe_route_progress(9).action, "none")

        pause = coordinator.observe_route_progress(10)
        self.assertEqual(pause.status.state, MissionState.PAUSING)
        self.assertEqual(pause.action, "pause_navigation")
        pause_request_id = pause.status.pause_request_id
        self.assertTrue(pause_request_id)

        wrong = NavigationStopReceipt(
            mission_id="mission-001",
            checkpoint_id="fire-extinguisher-1",
            pause_request_id="stale-pause",
            map_version="map-6855ba54ae11",
            route_id="route-r7",
            route_progress_index=10,
            stopped=True,
            motion_authorized=False,
            stable_for_s=1.0,
        )
        with self.assertRaisesRegex(ValueError, "active pause request"):
            coordinator.confirm_stopped(wrong)

        stopped = NavigationStopReceipt(
            mission_id="mission-001",
            checkpoint_id="fire-extinguisher-1",
            pause_request_id=str(pause_request_id),
            map_version="map-6855ba54ae11",
            route_id="route-r7",
            route_progress_index=10,
            stopped=True,
            motion_authorized=False,
            linear_speed_mps=0.01,
            angular_speed_rps=0.01,
            stable_for_s=0.8,
        )
        self.assertEqual(coordinator.confirm_stopped(stopped).status.state, MissionState.PAUSED)
        self.assertEqual(coordinator.begin_inspection().action, "inspect")
        self.assertEqual(coordinator.mark_inspection_complete().action, "wait_platform_continue")
        self.assertEqual(coordinator.request_continue().action, "resume_navigation")
        resumed = coordinator.confirm_resumed(10)
        self.assertEqual(resumed.status.state, MissionState.TRAVELING)
        self.assertEqual(resumed.status.resume_route_progress_index, 10)
        self.assertEqual(coordinator.route_completed().status.state, MissionState.COMPLETED)

    def test_stop_receipt_rejects_motion_and_non_finite_measurements(self) -> None:
        coordinator = MissionCoordinator(mission_plan())
        coordinator.start()
        pause = coordinator.observe_route_progress(10)
        base = dict(
            mission_id="mission-001",
            checkpoint_id="fire-extinguisher-1",
            pause_request_id=str(pause.status.pause_request_id),
            map_version="map-6855ba54ae11",
            route_id="route-r7",
            route_progress_index=10,
            stopped=True,
            motion_authorized=False,
            stable_for_s=0.8,
        )
        with self.assertRaisesRegex(ValueError, "stable"):
            coordinator.confirm_stopped(NavigationStopReceipt(**base, linear_speed_mps=0.03))
        with self.assertRaisesRegex(ValueError, "invalid motion"):
            coordinator.confirm_stopped(NavigationStopReceipt(**base, linear_speed_mps=float("nan")))

    def test_resume_after_inspection_interruption_reconfirms_stop(self) -> None:
        coordinator = MissionCoordinator(mission_plan())
        coordinator.start()
        pause = coordinator.observe_route_progress(10)
        coordinator.confirm_stopped(
            NavigationStopReceipt(
                mission_id="mission-001",
                checkpoint_id="fire-extinguisher-1",
                pause_request_id=str(pause.status.pause_request_id),
                map_version="map-6855ba54ae11",
                route_id="route-r7",
                route_progress_index=10,
                stopped=True,
                motion_authorized=False,
                stable_for_s=1.0,
            )
        )
        coordinator.begin_inspection()
        coordinator.interrupt("CAMERA_UNAVAILABLE", resumable=True)
        resumed = coordinator.resume_interrupted()
        self.assertEqual(resumed.status.state, MissionState.PAUSING)
        self.assertEqual(resumed.action, "pause_navigation")

    def test_offline_continue_requires_pre_authorization_and_durable_evidence(self) -> None:
        coordinator = MissionCoordinator(mission_plan())
        coordinator.start()
        pause = coordinator.observe_route_progress(10)
        coordinator.confirm_stopped(
            NavigationStopReceipt(
                mission_id="mission-001",
                checkpoint_id="fire-extinguisher-1",
                pause_request_id=str(pause.status.pause_request_id),
                map_version="map-6855ba54ae11",
                route_id="route-r7",
                route_progress_index=10,
                stopped=True,
                motion_authorized=False,
                stable_for_s=1.0,
            )
        )
        coordinator.begin_inspection()
        coordinator.mark_inspection_complete()
        with self.assertRaisesRegex(RuntimeError, "durably buffered"):
            coordinator.request_offline_continue(evidence_durable=False)
        self.assertEqual(coordinator.status().state, MissionState.WAITING_CONTINUE)
        transition = coordinator.request_offline_continue(evidence_durable=True)
        self.assertEqual(transition.status.state, MissionState.RESUMING)
        self.assertEqual(transition.status.reason, "OFFLINE_EVIDENCE_BUFFERED_CONTINUE")

    def test_plan_rejects_ambiguous_route_indexes(self) -> None:
        invalid = MissionPlan(
            mission_id="mission-001",
            map_version="map-1",
            route_id="route-1",
            checkpoints=(CheckpointPlan("point-1", True),),
        )
        with self.assertRaisesRegex(ValueError, "non-negative integer"):
            validate_mission_plan(invalid)


if __name__ == "__main__":
    unittest.main()
