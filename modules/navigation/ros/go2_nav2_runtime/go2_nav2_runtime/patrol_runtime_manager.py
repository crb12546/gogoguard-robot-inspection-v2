#!/usr/bin/env python3
"""Fast, version-pinned start/stop orchestration around Nav2 FollowPath."""

from __future__ import annotations

import hashlib
import json
import math
import time
import uuid
from pathlib import Path
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from go2_nav2_interfaces.srv import StartPatrol
from nav2_msgs.action import ComputePathToPose, FollowPath, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from unitree_go.msg import SportModeState

from go2_site_ops.map_store import MapVersionStore
from gogoguard_navigation.orchestration import (
    ControllerMode,
    ControllerSuccessAction,
    FailureEvidence,
    MotionEvidenceTracker,
    ObstructionEvidenceTracker,
    RecoveryAction,
    controller_success_action,
    decide_controller_failure,
    evaluate_costmap_health,
    route_obstruction_evidence,
)
from gogoguard_navigation.checkpoint_runtime import (
    CheckpointExecutor,
    load_navigation_mission,
)
from gogoguard_navigation.checkpoint_alignment import (
    plan_checkpoint_view_alignment,
)

from .runtime_core import (
    FastLioHealthTracker,
    PatrolReadiness,
    RuntimeArtifactGuard,
    evaluate_readiness,
    evaluate_runtime_trace_gate,
    evaluate_runtime_gate,
    load_candidate_runtime_bundle,
    load_runtime_bundle,
    monotonic_age,
    operator_message_for_reason,
    requested_runtime_identity_reason,
    sample_route,
)
from .start_timing import StartAttemptTimeline
from .local_detour import GridView, route_rejoin_index


RECOVERABLE_RUNTIME_GATES = frozenset(
    {
        "LOCALIZATION_STATUS_MISSING",
        "LOCALIZATION_STATUS_STALE",
        "LOCALIZATION_NOT_TRACKING",
        "LOCALIZATION_NOT_USABLE",
        "LOCALIZATION_POSE_MISSING",
        "LOCALIZATION_POSE_STALE",
        "FASTLIO_ODOMETRY_MISSING",
        "FASTLIO_ODOMETRY_STALE",
        "FASTLIO_ODOMETRY_INVALID",
        "FASTLIO_TIME_RESET",
        "FASTLIO_FRAME_GAP",
        "FASTLIO_WAITING_FOR_STATIONARY",
        "FASTLIO_SETTLING",
        "FASTLIO_STATIONARY_DRIFT",
        "NAV2_FOLLOW_PATH_UNAVAILABLE",
        "NAV2_PLANNER_UNAVAILABLE",
        "ROBOT_STATE_MISSING",
        "ROBOT_STATE_STALE",
        "ROBOT_POSTURE_NOT_READY",
        "ROBOT_ACTION_IN_PROGRESS",
        "COSTMAP_MISSING",
        "COSTMAP_STALE",
        "COSTMAP_ROBOT_POSE_MISSING",
        "COSTMAP_ROBOT_OUTSIDE",
        "COSTMAP_ROBOT_OCCUPIED",
    }
)


def _yaw_from_quaternion(quaternion) -> float:
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def _quaternion_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion

    result = Quaternion()
    result.z = math.sin(yaw * 0.5)
    result.w = math.cos(yaw * 0.5)
    return result


class PatrolRuntimeManager(Node):
    """Keep expensive runtime nodes warm and make startPatrol a short gate check."""

    def __init__(self):
        super().__init__("patrol_runtime_manager")
        self.declare_parameter("map_store_root", "")
        self.declare_parameter("site_id", "")
        self.declare_parameter("expected_map_version", "")
        self.declare_parameter("runtime_source", "active")
        self.declare_parameter("mission_plan_path", "")
        self.declare_parameter(
            "checkpoint_control_path",
            "/var/lib/gogoguard/platform/checkpoint-control.json",
        )
        self.declare_parameter("nav2_profile_hash", "")
        self.declare_parameter("candidate.localization_map", "")
        self.declare_parameter("candidate.route", "")
        self.declare_parameter("candidate.runtime_profile", "")
        self.declare_parameter("candidate.allowed_area_mask", "")
        self.declare_parameter("candidate.allowed_area_mask_image", "")
        self.declare_parameter("candidate.navigation_map", "")
        self.declare_parameter("candidate.navigation_map_image", "")
        self.declare_parameter("candidate.localization_map_hash", "")
        self.declare_parameter("candidate.route_hash", "")
        self.declare_parameter("candidate.runtime_profile_hash", "")
        self.declare_parameter("candidate.allowed_area_mask_hash", "")
        self.declare_parameter("candidate.allowed_area_mask_image_hash", "")
        self.declare_parameter("candidate.navigation_map_hash", "")
        self.declare_parameter("candidate.navigation_map_image_hash", "")
        self.declare_parameter("follow_path_action", "/follow_path")
        self.declare_parameter("compute_path_action", "/compute_path_to_pose")
        self.declare_parameter("localization_status_topic", "/localization/status")
        self.declare_parameter("localization_pose_topic", "/localization/pose")
        self.declare_parameter("localization_timeout_s", 0.20)
        self.declare_parameter("localization_dropout_grace_s", 1.0)
        self.declare_parameter("localization_recovery_stable_s", 0.5)
        self.declare_parameter("checkpoint_camera_preferred_pan_limit_deg", 110.0)
        self.declare_parameter("checkpoint_camera_hard_pan_limit_deg", 140.0)
        self.declare_parameter("checkpoint_body_yaw_tolerance_rad", 0.03)
        self.declare_parameter("progress_timeout_s", 5.0)
        self.declare_parameter("replan_interval_s", 0.75)
        self.declare_parameter("obstruction_cost_threshold", 65)
        self.declare_parameter("obstruction_min_samples", 2)
        self.declare_parameter("obstruction_confirmation_s", 0.30)
        self.declare_parameter("costmap_timeout_s", 1.0)
        self.declare_parameter("rejoin_lookahead_m", 2.0)
        self.declare_parameter("planner_rejoin_lookahead_m", 8.0)
        self.declare_parameter(
            "runtime_trace_status_topic", "/go2/runtime/trace_status"
        )
        self.declare_parameter("runtime_trace_status_timeout_s", 0.50)
        self.declare_parameter("pose_timeout_s", 1.0)
        self.declare_parameter("active_map_check_period_s", 0.10)
        self.declare_parameter("full_map_audit_period_s", 60.0)
        self.declare_parameter("binding_shutdown_delay_s", 1.0)
        self.declare_parameter("robot_state_topic", "/lf/sportmodestate")
        self.declare_parameter("robot_state_timeout_s", 0.20)
        self.declare_parameter("start_stationary_speed_mps", 0.08)
        self.declare_parameter("start_stationary_yaw_rate_rps", 0.12)
        self.declare_parameter("final_cmd_topic", "/cmd_vel")
        self.declare_parameter("final_cmd_nonzero_epsilon", 0.001)
        self.declare_parameter("fastlio_odom_topic", "/Odometry")
        self.declare_parameter("fastlio_health.settle_window_s", 5.0)
        self.declare_parameter(
            "fastlio_health.max_stationary_translation_m", 0.05
        )
        self.declare_parameter(
            "fastlio_health.max_stationary_rotation_deg", 1.0
        )
        self.declare_parameter("fastlio_health.odom_timeout_s", 0.5)
        self.declare_parameter("fastlio_health.max_frame_gap_s", 0.5)
        self.declare_parameter("fastlio_health.minimum_samples", 30)

        self.map_store_root = str(self.get_parameter("map_store_root").value).strip()
        self.site_id = str(self.get_parameter("site_id").value).strip()
        expected = str(self.get_parameter("expected_map_version").value).strip()
        source = str(self.get_parameter("runtime_source").value).strip()
        self.runtime_source = source
        self.nav2_profile_hash = str(
            self.get_parameter("nav2_profile_hash").value
        ).strip()
        self.map_store = None
        self.runtime_guard = None
        if not self.site_id or not expected:
            raise RuntimeError("site_id and expected_map_version are required")
        if source == "active":
            if not self.map_store_root:
                raise RuntimeError("map_store_root is required for active runtime")
            self.map_store = MapVersionStore(Path(self.map_store_root))
            self.bundle = load_runtime_bundle(
                self.map_store_root,
                self.site_id,
                expected,
                store=self.map_store,
            )
            self.runtime_guard = RuntimeArtifactGuard.capture(
                {
                    "active_pointer": self.map_store.sites_root
                    / self.site_id
                    / "active.json",
                    "manifest": self.map_store.versions_root
                    / self.bundle.version_id
                    / "manifest.json",
                    "localization_map": self.bundle.localization_map_path,
                    "route": self.bundle.route_path,
                    "runtime_profile": self.bundle.runtime_profile_path,
                    "allowed_area_mask": self.bundle.allowed_area_mask_path,
                    "allowed_area_mask_image": self.bundle.allowed_area_mask_image_path,
                    "navigation_map": self.bundle.navigation_map_path,
                    "navigation_map_image": self.bundle.navigation_map_image_path,
                    "calibration_bundle": self.bundle.calibration_bundle_path,
                }
            )
        elif source == "candidate":
            candidate_paths = {
                "localization_map": self.get_parameter(
                    "candidate.localization_map"
                ).value,
                "route": self.get_parameter("candidate.route").value,
                "runtime_profile": self.get_parameter(
                    "candidate.runtime_profile"
                ).value,
                "allowed_area_mask": self.get_parameter(
                    "candidate.allowed_area_mask"
                ).value,
                "allowed_area_mask_image": self.get_parameter(
                    "candidate.allowed_area_mask_image"
                ).value,
                "navigation_map": self.get_parameter(
                    "candidate.navigation_map"
                ).value,
                "navigation_map_image": self.get_parameter(
                    "candidate.navigation_map_image"
                ).value,
            }
            self.bundle = load_candidate_runtime_bundle(
                site_id=self.site_id,
                version_id=expected,
                localization_map_path=candidate_paths["localization_map"],
                route_path=candidate_paths["route"],
                runtime_profile_path=candidate_paths["runtime_profile"],
                allowed_area_mask_path=candidate_paths["allowed_area_mask"],
                allowed_area_mask_image_path=candidate_paths[
                    "allowed_area_mask_image"
                ],
                navigation_map_path=candidate_paths["navigation_map"],
                navigation_map_image_path=candidate_paths["navigation_map_image"],
                localization_map_hash=self.get_parameter(
                    "candidate.localization_map_hash"
                ).value,
                route_hash=self.get_parameter("candidate.route_hash").value,
                runtime_profile_hash=self.get_parameter(
                    "candidate.runtime_profile_hash"
                ).value,
                allowed_area_mask_hash=self.get_parameter(
                    "candidate.allowed_area_mask_hash"
                ).value,
                allowed_area_mask_image_hash=self.get_parameter(
                    "candidate.allowed_area_mask_image_hash"
                ).value,
                navigation_map_hash=self.get_parameter(
                    "candidate.navigation_map_hash"
                ).value,
                navigation_map_image_hash=self.get_parameter(
                    "candidate.navigation_map_image_hash"
                ).value,
            )
            self.runtime_guard = RuntimeArtifactGuard.capture(
                {
                    "localization_map": self.bundle.localization_map_path,
                    "route": self.bundle.route_path,
                    "runtime_profile": self.bundle.runtime_profile_path,
                    "allowed_area_mask": self.bundle.allowed_area_mask_path,
                    "allowed_area_mask_image": self.bundle.allowed_area_mask_image_path,
                    "navigation_map": self.bundle.navigation_map_path,
                    "navigation_map_image": self.bundle.navigation_map_image_path,
                }
            )
        else:
            raise RuntimeError("runtime_source must be active or candidate")
        self.runtime_profile_hash = hashlib.sha256(
            self.bundle.runtime_profile_path.read_bytes()
        ).hexdigest()
        if len(self.nav2_profile_hash) != 64:
            raise RuntimeError("nav2_profile_hash is required")
        self.localization_timeout_s = float(
            self.get_parameter("localization_timeout_s").value
        )
        self.localization_dropout_grace_s = float(
            self.get_parameter("localization_dropout_grace_s").value
        )
        self.localization_recovery_stable_s = float(
            self.get_parameter("localization_recovery_stable_s").value
        )
        self.checkpoint_camera_preferred_pan_limit_deg = float(
            self.get_parameter("checkpoint_camera_preferred_pan_limit_deg").value
        )
        self.checkpoint_camera_hard_pan_limit_deg = float(
            self.get_parameter("checkpoint_camera_hard_pan_limit_deg").value
        )
        self.checkpoint_body_yaw_tolerance_rad = float(
            self.get_parameter("checkpoint_body_yaw_tolerance_rad").value
        )
        self.progress_timeout_s = float(
            self.get_parameter("progress_timeout_s").value
        )
        self.replan_interval_s = float(
            self.get_parameter("replan_interval_s").value
        )
        self.obstruction_cost_threshold = int(
            self.get_parameter("obstruction_cost_threshold").value
        )
        self.obstruction_min_samples = int(
            self.get_parameter("obstruction_min_samples").value
        )
        self.obstruction_confirmation_s = float(
            self.get_parameter("obstruction_confirmation_s").value
        )
        self.costmap_timeout_s = float(
            self.get_parameter("costmap_timeout_s").value
        )
        self.rejoin_lookahead_m = float(
            self.get_parameter("rejoin_lookahead_m").value
        )
        self.planner_rejoin_lookahead_m = float(
            self.get_parameter("planner_rejoin_lookahead_m").value
        )
        self.runtime_trace_status_timeout_s = float(
            self.get_parameter("runtime_trace_status_timeout_s").value
        )
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self.active_map_check_period_s = float(
            self.get_parameter("active_map_check_period_s").value
        )
        self.full_map_audit_period_s = float(
            self.get_parameter("full_map_audit_period_s").value
        )
        self.binding_shutdown_delay_s = float(
            self.get_parameter("binding_shutdown_delay_s").value
        )
        self.robot_state_timeout_s = float(
            self.get_parameter("robot_state_timeout_s").value
        )
        self.start_stationary_speed_mps = float(
            self.get_parameter("start_stationary_speed_mps").value
        )
        self.start_stationary_yaw_rate_rps = float(
            self.get_parameter("start_stationary_yaw_rate_rps").value
        )
        self.final_cmd_nonzero_epsilon = float(
            self.get_parameter("final_cmd_nonzero_epsilon").value
        )
        self.require_robot_state = source == "active"
        self.require_fastlio_health = source == "active"
        if not 0.20 <= self.localization_timeout_s <= 2.0:
            raise RuntimeError(
                "localization_timeout_s must be between 0.20 and 2.0 seconds"
            )
        if not 0.20 <= self.localization_dropout_grace_s <= 5.0:
            raise RuntimeError("localization_dropout_grace_s is invalid")
        if not 0.20 <= self.localization_recovery_stable_s <= 3.0:
            raise RuntimeError("localization_recovery_stable_s is invalid")
        if not (
            30.0 <= self.checkpoint_camera_preferred_pan_limit_deg <= 120.0
            and self.checkpoint_camera_preferred_pan_limit_deg
            <= self.checkpoint_camera_hard_pan_limit_deg
            <= 140.0
        ):
            raise RuntimeError("checkpoint camera pan limits are invalid")
        if not 0.01 <= self.checkpoint_body_yaw_tolerance_rad <= 0.10:
            raise RuntimeError("checkpoint body yaw tolerance is invalid")
        if not 2.0 <= self.progress_timeout_s <= 12.0:
            raise RuntimeError("progress_timeout_s is invalid")
        if not 0.25 <= self.replan_interval_s <= 3.0:
            raise RuntimeError("replan_interval_s is invalid")
        if not 1 <= self.obstruction_cost_threshold <= 100:
            raise RuntimeError("obstruction_cost_threshold is invalid")
        if not 1 <= self.obstruction_min_samples <= 10:
            raise RuntimeError("obstruction_min_samples is invalid")
        if not 0.20 <= self.obstruction_confirmation_s <= 2.0:
            raise RuntimeError("obstruction_confirmation_s is invalid")
        if not 0.30 <= self.costmap_timeout_s <= 2.0:
            raise RuntimeError("costmap_timeout_s is invalid")
        if not 0.5 <= self.rejoin_lookahead_m <= 6.0:
            raise RuntimeError("rejoin_lookahead_m is invalid")
        if not 6.0 <= self.planner_rejoin_lookahead_m <= 10.0:
            raise RuntimeError("planner_rejoin_lookahead_m is invalid")
        if not 0.30 <= self.runtime_trace_status_timeout_s <= 2.0:
            raise RuntimeError(
                "runtime_trace_status_timeout_s must be between 0.30 and 2.0 seconds"
            )
        if not 0.05 <= self.active_map_check_period_s <= 0.10:
            raise RuntimeError(
                "active_map_check_period_s must be between 0.05 and 0.10 seconds"
            )
        if self.full_map_audit_period_s < self.active_map_check_period_s:
            raise RuntimeError(
                "full_map_audit_period_s must not be shorter than the runtime binding check"
            )
        if not 0.5 <= self.binding_shutdown_delay_s <= 10.0:
            raise RuntimeError("binding_shutdown_delay_s must be between 0.5 and 10 seconds")
        if not 0.1 <= self.robot_state_timeout_s <= 0.20:
            raise RuntimeError(
                "robot_state_timeout_s must be between 0.1 and 0.20 seconds"
            )
        if not 0.0 < self.start_stationary_speed_mps <= 0.25:
            raise RuntimeError("start_stationary_speed_mps is invalid")
        if not 0.0 < self.start_stationary_yaw_rate_rps <= 0.5:
            raise RuntimeError("start_stationary_yaw_rate_rps is invalid")
        if not 0.0 < self.final_cmd_nonzero_epsilon <= 0.05:
            raise RuntimeError("final_cmd_nonzero_epsilon is invalid")
        self.fastlio_health = FastLioHealthTracker(
            settle_window_s=float(
                self.get_parameter("fastlio_health.settle_window_s").value
            ),
            maximum_translation_excursion_m=float(
                self.get_parameter(
                    "fastlio_health.max_stationary_translation_m"
                ).value
            ),
            maximum_rotation_excursion_deg=float(
                self.get_parameter(
                    "fastlio_health.max_stationary_rotation_deg"
                ).value
            ),
            odom_timeout_s=float(
                self.get_parameter("fastlio_health.odom_timeout_s").value
            ),
            maximum_frame_gap_s=float(
                self.get_parameter("fastlio_health.max_frame_gap_s").value
            ),
            minimum_samples=int(
                self.get_parameter("fastlio_health.minimum_samples").value
            ),
        )

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, "/go2/runtime/status", latched
        )
        self.authorization_publisher = self.create_publisher(
            Bool, "/go2/runtime/motion_authorized", latched
        )
        self.route_publisher = self.create_publisher(
            NavPath, "/go2/runtime/route", latched
        )
        self.planner_diagnostics_publisher = self.create_publisher(
            String, "/go2/runtime/planner_diagnostics", latched
        )

        self.localization_status = None
        self.localization_received_at = 0.0
        self.runtime_trace_status = None
        self.runtime_trace_received_at = 0.0
        self.pose = None
        self.pose_frame = ""
        self.pose_received_at = 0.0
        self.robot_motion_status = None
        self.robot_motion_received_at = 0.0
        self.last_runtime_binding_check = 0.0
        self.last_full_map_audit = time.monotonic()
        self.runtime_binding_valid = True
        self.runtime_binding_reason = "OK"
        self.runtime_binding_fault_at = None
        self.shutdown_requested = False
        self.exit_code = 0
        self.runtime_instance_id = uuid.uuid4().hex
        self.status_sequence = 0
        self.active_start_attempt = None
        self.last_start_attempt = None
        self.current_motion_authorized = False
        self.goal_handle = None
        self.goal_request_pending = False
        self.stop_requested = False
        self.gate_cancel_reason = ""
        self.resume_pending = False
        self.resume_count = 0
        self.runtime_state = "BOOTING"
        self.runtime_reason = "WAITING_FOR_LOCALIZATION"
        self.last_feedback_distance = None
        self.path = self._make_path()
        self.route_progress_index = 0
        self.mission = load_navigation_mission(
            Path(str(self.get_parameter("mission_plan_path").value).strip()),
            expected_map_version=self.bundle.version_id,
            expected_route_id=self.bundle.route.route_id,
            route_point_count=len(self.path.poses),
        )
        self.checkpoints = CheckpointExecutor(self.mission)
        self.checkpoint_control_path = Path(
            str(self.get_parameter("checkpoint_control_path").value)
        )
        self.checkpoint_phase_started_at = time.monotonic()
        self.checkpoint_observed_phase = self.checkpoints.phase
        self.spin_goal_handle = None
        self.spin_request_pending = False
        self.spin_operation = ""
        self.last_final_command = None
        self.last_final_command_at = 0.0
        self.localization_gate_failed_at = None
        self.localization_recovered_at = None
        self.checkpoint_gate_failed_at = None
        self.checkpoint_gate_recovered_at = None
        self.patrol_started_at = None
        self.local_costmap = None
        self.local_costmap_frame = ""
        self.local_costmap_received_at = 0.0
        self.costmap_sequence = 0
        self.last_obstruction_costmap_sequence = -1
        self.costmap_refresh_baseline_sequence = None
        self.costmap_refresh_started_at = None
        self.costmap_refresh_future = None
        self.detour_attempt_count = 0
        self.last_detour_compute_ms = None
        self.last_rejoin_index = None
        self.planner_completion = ""
        self.active_controller = ControllerMode.MPPI
        self.mppi_retry_count = 0
        self.recovery_pending = ""
        self.recovery_requested_at = None
        self.recovery_reason = ""
        self.last_controller_result = None
        self.last_failure_class = None
        self.last_route_obstructed = False
        self.obstruction_evidence = ObstructionEvidenceTracker(
            confirmation_s=self.obstruction_confirmation_s
        )
        self.motion_evidence = MotionEvidenceTracker(
            retention_s=max(12.0, self.progress_timeout_s + 2.0)
        )

        self.action_client = ActionClient(
            self,
            FollowPath,
            str(self.get_parameter("follow_path_action").value),
        )
        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            str(self.get_parameter("compute_path_action").value),
        )
        self.spin_client = ActionClient(self, Spin, "/spin")
        self.planner_goal_handle = None
        self.clear_costmap_client = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            String,
            str(self.get_parameter("localization_status_topic").value),
            self._localization_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("runtime_trace_status_topic").value),
            self._runtime_trace_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("localization_pose_topic").value),
            self._pose_callback,
            10,
        )
        if self.require_robot_state:
            self.create_subscription(
                SportModeState,
                str(self.get_parameter("robot_state_topic").value),
                self._robot_state_callback,
                qos_profile_sensor_data,
            )
        if self.require_fastlio_health:
            self.create_subscription(
                Odometry,
                str(self.get_parameter("fastlio_odom_topic").value),
                self._fastlio_odom_callback,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Twist,
            str(self.get_parameter("final_cmd_topic").value),
            self._final_cmd_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._costmap_callback, 5
        )
        self.create_service(StartPatrol, "/go2/patrol/start", self._start_callback)
        self.create_service(Trigger, "/go2/patrol/stop", self._stop_callback)
        self.create_timer(0.1, self._tick)

        self.route_publisher.publish(self.path)
        self._publish_status(False)
        self.get_logger().info(
            "runtime pinned: site=%s map=%s route_sha256=%s length=%.1fm"
            % (
                self.bundle.site_id,
                self.bundle.version_id,
                self.bundle.route.source_hash,
                self.bundle.route.length_m,
            )
        )

    def _make_path(self) -> NavPath:
        result = NavPath()
        result.header.frame_id = "map"
        for waypoint in sample_route(
            self.bundle.route, self.bundle.patrol.path_sample_spacing_m
        ):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.orientation = _quaternion_from_yaw(waypoint.yaw)
            result.poses.append(pose)
        return result

    def _localization_callback(self, message: String) -> None:
        received = time.monotonic()
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self.localization_status = payload
        self.localization_received_at = received

    def _runtime_trace_callback(self, message: String) -> None:
        received = time.monotonic()
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = None
        self.runtime_trace_status = payload
        self.runtime_trace_received_at = received

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        received = time.monotonic()
        self.pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            _yaw_from_quaternion(message.pose.pose.orientation),
        )
        self.pose_frame = message.header.frame_id
        self.pose_received_at = received
        self.motion_evidence.record_pose(received, *self.pose)
        if (
            (self.goal_handle is not None or self.goal_request_pending or self.resume_pending)
            and self.active_controller != ControllerMode.GOAL_MPPI
        ):
            self.route_progress_index = max(
                self.route_progress_index, self._nearest_path_index(self.pose[0], self.pose[1])
            )

    def _nearest_path_index(self, x: float, y: float) -> int:
        if not self.path.poses:
            return 0
        return min(
            range(len(self.path.poses)),
            key=lambda index: math.hypot(
                self.path.poses[index].pose.position.x - x,
                self.path.poses[index].pose.position.y - y,
            ),
        )

    def _remaining_path(self) -> NavPath:
        result = NavPath()
        result.header.frame_id = "map"
        # Keep two points behind the projection so MPPI sees the route tangent,
        # but never resend the completed prefix or an empty path.
        start = max(0, min(self.route_progress_index - 2, len(self.path.poses) - 2))
        result.poses = list(self.path.poses[start:])
        return result

    def _costmap_callback(self, message: OccupancyGrid) -> None:
        self.costmap_sequence += 1
        orientation = message.info.origin.orientation
        if not message.header.frame_id or abs(_yaw_from_quaternion(orientation)) > 1.0e-3:
            self.local_costmap = None
            self.local_costmap_frame = ""
            self.local_costmap_received_at = time.monotonic()
            return
        self.local_costmap = GridView(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            costs=tuple(int(value) for value in message.data),
        )
        self.local_costmap_frame = message.header.frame_id
        self.local_costmap_received_at = time.monotonic()

    def _costmap_health(self):
        robot_xy = None
        if self.pose is not None and self.pose_frame == "map":
            robot_xy = self.pose[:2]
        return evaluate_costmap_health(
            self.local_costmap,
            frame_id=self.local_costmap_frame,
            expected_frame="map",
            age_s=monotonic_age(self.local_costmap_received_at),
            robot_xy=robot_xy,
            timeout_s=self.costmap_timeout_s,
        )

    def _reset_costmap_refresh(self) -> None:
        self.costmap_refresh_baseline_sequence = None
        self.costmap_refresh_started_at = None
        self.costmap_refresh_future = None

    def _resume_costmap_refresh(self, now: float) -> PatrolReadiness:
        """Clear once and wait for a newer healthy map without blocking ROS callbacks."""

        if self.costmap_refresh_started_at is None:
            self.costmap_refresh_started_at = float(now)
            self.costmap_refresh_baseline_sequence = self.costmap_sequence
        elapsed = float(now) - self.costmap_refresh_started_at
        if self.costmap_refresh_future is None:
            if not self.clear_costmap_client.service_is_ready():
                if elapsed >= 4.0:
                    return PatrolReadiness(
                        False, "FAULT", "COSTMAP_CLEAR_SERVICE_UNAVAILABLE"
                    )
                return PatrolReadiness(False, "RESUMING", "COSTMAP_CLEAR_WAITING")
            self.costmap_refresh_future = self.clear_costmap_client.call_async(
                ClearEntireCostmap.Request()
            )
            return PatrolReadiness(False, "RESUMING", "COSTMAP_CLEARING")
        if not self.costmap_refresh_future.done():
            if elapsed >= 4.0:
                return PatrolReadiness(False, "FAULT", "COSTMAP_CLEAR_TIMEOUT")
            return PatrolReadiness(False, "RESUMING", "COSTMAP_CLEARING")
        try:
            self.costmap_refresh_future.result()
        except Exception:
            return PatrolReadiness(False, "FAULT", "COSTMAP_CLEAR_FAILED")
        baseline = int(self.costmap_refresh_baseline_sequence or 0)
        health = self._costmap_health()
        if self.costmap_sequence > baseline and health.healthy:
            return PatrolReadiness(True, "READY", "OK")
        if elapsed >= 4.0:
            return PatrolReadiness(False, "FAULT", health.reason)
        return PatrolReadiness(False, "RESUMING", "COSTMAP_REFRESHING")

    def _transform_xy(self, x: float, y: float, target: str, source: str):
        if target == source:
            return float(x), float(y)
        try:
            transform = self.tf_buffer.lookup_transform(target, source, Time())
        except Exception:
            return None
        translation = transform.transform.translation
        yaw = _yaw_from_quaternion(transform.transform.rotation)
        return (
            translation.x + math.cos(yaw) * x - math.sin(yaw) * y,
            translation.y + math.sin(yaw) * x + math.cos(yaw) * y,
        )

    def _route_and_rejoin(self, lookahead_m: Optional[float] = None):
        route_xy = [
            (item.pose.position.x, item.pose.position.y) for item in self.path.poses
        ]
        if len(route_xy) < 2:
            return route_xy, None
        rejoin_index = route_rejoin_index(
            route_xy,
            self.route_progress_index,
            self.rejoin_lookahead_m if lookahead_m is None else float(lookahead_m),
        )
        if rejoin_index <= self.route_progress_index or rejoin_index >= len(route_xy):
            return route_xy, None
        return route_xy, rejoin_index

    def _route_obstructed_sample(self) -> bool:
        """Read one spatial obstruction sample from a healthy costmap."""
        health = self._costmap_health()
        if (
            not health.healthy
            or self.pose is None
            or self.local_costmap is None
        ):
            return False
        route_xy, rejoin_index = self._route_and_rejoin()
        if rejoin_index is None:
            return False
        start_index = max(0, min(self.route_progress_index, rejoin_index - 1))
        segment_in_grid = [
            self._transform_xy(
                point[0], point[1], self.local_costmap_frame, "map"
            )
            for point in route_xy[start_index : rejoin_index + 1]
        ]
        if any(point is None for point in segment_in_grid):
            return False
        return route_obstruction_evidence(
            self.local_costmap,
            segment_in_grid,
            occupied_threshold=self.obstruction_cost_threshold,
            minimum_consecutive_samples=self.obstruction_min_samples,
        )

    def _update_obstruction_evidence(self, now: float) -> None:
        health = self._costmap_health()
        if not health.healthy:
            self.obstruction_evidence.reset()
            self.last_route_obstructed = False
            return
        if self.costmap_sequence == self.last_obstruction_costmap_sequence:
            return
        self.last_obstruction_costmap_sequence = self.costmap_sequence
        self.last_route_obstructed = self.obstruction_evidence.update(
            now,
            blocked=self._route_obstructed_sample(),
            source_healthy=True,
        )

    def _route_obstructed(self) -> bool:
        """Return only temporally confirmed obstruction evidence."""
        return bool(self.last_route_obstructed)

    def _try_global_replan(self, *, forced: bool = False) -> bool:
        if (
            self.pose is None
            or (not forced and not self._route_obstructed())
            or not self.planner_client.server_is_ready()
        ):
            return False
        if forced:
            route_xy = [
                (item.pose.position.x, item.pose.position.y)
                for item in self.path.poses
            ]
            if not route_xy:
                return False
            next_checkpoint = (
                self.mission.checkpoints[self.checkpoints.cursor]
                if self.checkpoints.cursor < len(self.mission.checkpoints)
                else None
            )
            rejoin_index = (
                next_checkpoint.route_progress_index
                if next_checkpoint is not None
                else len(route_xy) - 1
            )
        else:
            route_xy, rejoin_index = self._route_and_rejoin(
                self.planner_rejoin_lookahead_m
            )
        if rejoin_index is None:
            return False
        stamp = self.get_clock().now().to_msg()
        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = stamp
        start.pose.position.x = float(self.pose[0])
        start.pose.position.y = float(self.pose[1])
        start.pose.orientation = _quaternion_from_yaw(float(self.pose[2]))
        target = PoseStamped()
        target.header.frame_id = "map"
        target.header.stamp = stamp
        target.pose.position.x = float(route_xy[rejoin_index][0])
        target.pose.position.y = float(route_xy[rejoin_index][1])
        target.pose.orientation = self.path.poses[rejoin_index].pose.orientation
        goal = ComputePathToPose.Goal()
        goal.start = start
        goal.goal = target
        goal.use_start = True
        goal.planner_id = "GridBased"
        self.goal_request_pending = True
        self.detour_attempt_count += 1
        self.last_rejoin_index = rejoin_index
        self.planner_completion = "GOAL" if forced else "REJOIN"
        self.runtime_state = "REPLANNING"
        self.runtime_reason = (
            "NAV2_GLOBAL_PLAN_TO_PATROL_GOAL"
            if forced
            else "NAV2_GLOBAL_PLAN_TO_ROUTE_REJOIN"
        )
        self._planner_requested_at = time.monotonic()
        try:
            future = self.planner_client.send_goal_async(goal)
        except Exception as exc:
            self.goal_request_pending = False
            self.runtime_reason = "NAV2_GLOBAL_PLAN_TRANSPORT: %s" % exc
            return False
        future.add_done_callback(self._planner_goal_response_callback)
        return True

    def _planner_goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            goal_handle = None
            self.runtime_reason = "NAV2_GLOBAL_PLAN_TRANSPORT: %s" % exc
        if goal_handle is None or not goal_handle.accepted:
            self.goal_request_pending = False
            self.last_failure_class = "PLANNER_FAILED"
            if not self.runtime_reason.startswith("NAV2_GLOBAL_PLAN_TRANSPORT"):
                self.runtime_reason = "NAV2_GLOBAL_PLAN_REJECTED"
            self._schedule_recovery(
                "GOAL" if self.planner_completion == "GOAL" else "DETOUR",
                self.runtime_reason,
            )
            return
        self.planner_goal_handle = goal_handle
        if self.stop_requested or self.gate_cancel_reason:
            goal_handle.cancel_goal_async()
        goal_handle.get_result_async().add_done_callback(self._planner_result_callback)

    def _planner_result_callback(self, future) -> None:
        self.goal_request_pending = False
        self.planner_goal_handle = None
        try:
            wrapped = future.result()
            status = wrapped.status
            result = wrapped.result
            path = result.path
        except Exception as exc:
            status = None
            path = None
            self.runtime_reason = "NAV2_GLOBAL_PLAN_RESULT: %s" % exc
        self.last_detour_compute_ms = round(
            (time.monotonic() - getattr(self, "_planner_requested_at", time.monotonic()))
            * 1000.0,
            2,
        )
        diagnostic = String()
        diagnostic.data = json.dumps(
            {
                "schema": "gogoguard.nav2_planner_receipt.v1",
                "planner": "SmacPlanner2D",
                "status": None if status is None else int(status),
                "computeMs": self.last_detour_compute_ms,
                "rejoinRouteIndex": self.last_rejoin_index,
                "routeObstructed": self._route_obstructed(),
                "planPurpose": self.planner_completion,
                "pathPoints": len(path.poses) if path is not None else 0,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.planner_diagnostics_publisher.publish(diagnostic)
        if self.stop_requested:
            self.runtime_state = "READY"
            self.runtime_reason = "STOPPED"
            self._finish_active_start_attempt("STOPPED", "STOPPED")
            return
        if self.gate_cancel_reason:
            cancel_reason = self.gate_cancel_reason
            self.gate_cancel_reason = ""
            self.resume_pending = cancel_reason in RECOVERABLE_RUNTIME_GATES
            self.runtime_state = "HOLDING" if self.resume_pending else "FAULT"
            self.runtime_reason = cancel_reason
            if not self.resume_pending:
                self._finish_active_start_attempt("FAILED", cancel_reason)
            return
        if status != GoalStatus.STATUS_SUCCEEDED or path is None or len(path.poses) < 2:
            self.last_failure_class = "PLANNER_FAILED"
            self._schedule_recovery(
                "GOAL" if self.planner_completion == "GOAL" else "DETOUR",
                "SEARCHING_FOR_PATH",
            )
            return
        now = self.get_clock().now().to_msg()
        path.header.stamp = now
        for pose in path.poses:
            pose.header.stamp = now
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "route_goal_checker"
        self.goal_request_pending = True
        self.active_controller = (
            ControllerMode.GOAL_MPPI
            if self.planner_completion == "GOAL"
            else ControllerMode.DETOUR_MPPI
        )
        self.runtime_state = "PLANNING_TO_GOAL" if self.planner_completion == "GOAL" else "DETOURING"
        self.runtime_reason = (
            "NAV2_GLOBAL_PLAN_TO_PATROL_GOAL_USING_MPPI"
            if self.planner_completion == "GOAL"
            else "NAV2_GLOBAL_PATH_USING_MPPI"
        )
        try:
            request = self.action_client.send_goal_async(
                goal, feedback_callback=self._feedback_callback
            )
        except Exception as exc:
            self.goal_request_pending = False
            self.runtime_reason = (
                "NAV2_GOAL_MPPI_TRANSPORT: %s" % exc
                if self.active_controller == ControllerMode.GOAL_MPPI
                else "NAV2_DETOUR_MPPI_TRANSPORT: %s" % exc
            )
            self._schedule_recovery(
                "GOAL" if self.planner_completion == "GOAL" else "DETOUR",
                self.runtime_reason,
            )
            return
        request.add_done_callback(self._replan_goal_response_callback)

    def _replan_goal_response_callback(self, future) -> None:
        self.goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            goal_handle = None
            self.runtime_reason = (
                "NAV2_GOAL_MPPI_TRANSPORT: %s" % exc
                if self.active_controller == ControllerMode.GOAL_MPPI
                else "NAV2_DETOUR_MPPI_TRANSPORT: %s" % exc
            )
        if goal_handle is None or not goal_handle.accepted:
            self.last_failure_class = "CONTROLLER_FAILED"
            if "_TRANSPORT:" not in self.runtime_reason:
                self.runtime_reason = (
                    "NAV2_GOAL_MPPI_REJECTED"
                    if self.active_controller == ControllerMode.GOAL_MPPI
                    else "NAV2_DETOUR_MPPI_REJECTED"
                )
            self._schedule_recovery(
                "GOAL" if self.active_controller == ControllerMode.GOAL_MPPI else "DETOUR",
                self.runtime_reason,
            )
            return
        self.goal_handle = goal_handle
        if (
            self.active_controller == ControllerMode.GOAL_MPPI
            and self.active_start_attempt is not None
            and self.active_start_attempt.goal_decided_at_monotonic is None
        ):
            self.active_start_attempt.mark_goal_decision(True, reason="OK")
        if self.stop_requested:
            self.runtime_state = "STOPPING"
            self.runtime_reason = "STOP_REQUESTED"
            goal_handle.cancel_goal_async()
            goal_handle.get_result_async().add_done_callback(self._result_callback)
            return
        self.runtime_state = "PATROLLING"
        self.runtime_reason = (
            "NAV2_GOAL_MPPI_ACCEPTED"
            if self.active_controller == ControllerMode.GOAL_MPPI
            else "NAV2_DETOUR_MPPI_ACCEPTED"
        )
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _request_mppi_suffix(self, reason: str) -> bool:
        path = self._remaining_path()
        if len(path.poses) < 2:
            self.runtime_state = "COMPLETED"
            self.runtime_reason = "ROUTE_COMPLETE"
            self._finish_active_start_attempt("COMPLETED", self.runtime_reason)
            return True
        now = self.get_clock().now().to_msg()
        path.header.stamp = now
        for pose in path.poses:
            pose.header.stamp = now
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "route_goal_checker"
        self.goal_request_pending = True
        self.active_controller = ControllerMode.MPPI
        self.runtime_state = "REJOINING" if reason.startswith("DETOUR_") else "RETRYING"
        self.runtime_reason = reason
        try:
            future = self.action_client.send_goal_async(
                goal, feedback_callback=self._feedback_callback
            )
        except Exception as exc:
            self.goal_request_pending = False
            self.runtime_reason = "NAV2_CONTINUATION_TRANSPORT: %s" % exc
            self._schedule_recovery("MPPI", self.runtime_reason)
            return False
        future.add_done_callback(
            lambda completed, accepted_reason=reason: (
                self._continuation_goal_response_callback(
                    completed, accepted_reason
                )
            )
        )
        return True

    def _schedule_recovery(self, mode: str, reason: str) -> None:
        """Keep the patrol alive while waiting for fresh planning evidence."""

        # A late planner/controller callback must never resurrect a patrol
        # after the independent operator-stop lane has revoked it.
        if self.stop_requested:
            self.recovery_pending = ""
            self.recovery_requested_at = None
            self.recovery_reason = ""
            self.runtime_state = "READY"
            self.runtime_reason = "STOPPED"
            self._finish_active_start_attempt("STOPPED", "STOPPED")
            return
        self.recovery_pending = str(mode)
        self.recovery_requested_at = time.monotonic()
        self.recovery_reason = str(reason)
        self._reset_costmap_refresh()
        if mode in {"DETOUR", "GOAL"}:
            self.runtime_state = "SEARCHING_PATH"
            self.runtime_reason = "SEARCHING_FOR_PATH"
        else:
            self.runtime_state = "RECOVERING"
            self.runtime_reason = str(reason)

    def _continuation_goal_response_callback(self, future, accepted_reason: str) -> None:
        self.goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            goal_handle = None
            self.runtime_reason = "NAV2_CONTINUATION_TRANSPORT: %s" % exc
        if goal_handle is None or not goal_handle.accepted:
            self.last_failure_class = "CONTROLLER_FAILED"
            if not self.runtime_reason.startswith("NAV2_CONTINUATION_TRANSPORT"):
                self.runtime_reason = "NAV2_CONTINUATION_REJECTED"
            self._schedule_recovery("MPPI", self.runtime_reason)
            return
        self.goal_handle = goal_handle
        if self.stop_requested:
            self.runtime_state = "STOPPING"
            self.runtime_reason = "STOP_REQUESTED"
            goal_handle.cancel_goal_async()
            goal_handle.get_result_async().add_done_callback(self._result_callback)
            return
        self.runtime_state = "PATROLLING"
        self.runtime_reason = accepted_reason
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _robot_state_callback(self, message: SportModeState) -> None:
        received = time.monotonic()
        try:
            self.robot_motion_status = {
                "errorCode": int(message.error_code),
                "mode": int(message.mode),
                "progress": float(message.progress),
                "vx": float(message.velocity[0]),
                "vy": float(message.velocity[1]),
                "yawSpeed": float(message.yaw_speed),
            }
        except (AttributeError, IndexError, TypeError, ValueError):
            self.robot_motion_status = None
        self.robot_motion_received_at = received
        status = self.robot_motion_status
        stationary = bool(
            isinstance(status, dict)
            and status.get("errorCode") in {0, 1001}
            and status.get("mode") in {0, 1, 3}
            and status.get("progress", 1.0) <= 0.01
            and math.hypot(status.get("vx", 1.0), status.get("vy", 1.0))
            <= self.start_stationary_speed_mps
            and abs(status.get("yawSpeed", 1.0))
            <= self.start_stationary_yaw_rate_rps
        )
        self.fastlio_health.set_stationary(stationary)

    def _fastlio_odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        stamp = message.header.stamp
        self.fastlio_health.update_odom(
            received_at=time.monotonic(),
            sensor_stamp_s=float(stamp.sec) + float(stamp.nanosec) * 1.0e-9,
            position=(pose.position.x, pose.position.y, pose.position.z),
            quaternion_xyzw=(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def _final_cmd_callback(self, message: Twist) -> None:
        """Observe, but never influence, the final safety-filtered command."""
        received_at = time.monotonic()
        linear_speed = math.hypot(float(message.linear.x), float(message.linear.y))
        angular_speed = abs(float(message.angular.z))
        self.last_final_command = (linear_speed, angular_speed)
        self.last_final_command_at = received_at
        self.motion_evidence.record_command(
            received_at,
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        attempt = self.active_start_attempt
        if attempt is None or not self.current_motion_authorized:
            return
        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.linear.z),
            float(message.angular.x),
            float(message.angular.y),
            float(message.angular.z),
        )
        if any(abs(value) > self.final_cmd_nonzero_epsilon for value in values):
            try:
                attempt.mark_first_final_nonzero()
            except RuntimeError as exc:
                self.get_logger().error(
                    "invalid start timing transition: %s" % exc
                )

    def _new_start_attempt(self, request) -> StartAttemptTimeline:
        return StartAttemptTimeline.create(
            requested_map_version=request.expected_map_version,
            requested_route_id=request.expected_route_id,
            attempt_id="%s-%s" % (
                self.runtime_instance_id[:12],
                uuid.uuid4().hex,
            ),
        )

    def _reject_start(self, attempt, response, reason, readiness=None):
        attempt.mark_service_response(success=False, reason=reason)
        self.last_start_attempt = attempt
        response.success = False
        response.message = self._response_json(reason, readiness, attempt=attempt)
        return response

    def _finish_active_start_attempt(self, outcome: str, reason: str) -> None:
        attempt = self.active_start_attempt
        if attempt is None:
            return
        attempt.finish(outcome=outcome, reason=reason)
        self.last_start_attempt = attempt
        self.active_start_attempt = None

    def _runtime_gate(self) -> PatrolReadiness:
        if not self.runtime_binding_valid:
            return PatrolReadiness(False, "FAULT", self.runtime_binding_reason)
        if self.require_fastlio_health:
            fastlio = self.fastlio_health.assess()
            if not fastlio.ready:
                return PatrolReadiness(False, "DEGRADED", fastlio.reason)
        readiness = evaluate_runtime_gate(
            expected_version=self.bundle.version_id,
            localization_status=self.localization_status,
            localization_age_s=monotonic_age(self.localization_received_at),
            localization_timeout_s=self.localization_timeout_s,
            pose_xy_yaw=self.pose,
            pose_frame=self.pose_frame,
            pose_age_s=monotonic_age(self.pose_received_at),
            pose_timeout_s=self.pose_timeout_s,
            action_server_ready=self.action_client.server_is_ready(),
            require_robot_state=self.require_robot_state,
            robot_motion_status=self.robot_motion_status,
            robot_motion_age_s=monotonic_age(self.robot_motion_received_at),
            robot_motion_timeout_s=self.robot_state_timeout_s,
        )
        if not readiness.ready:
            return readiness
        if not self.planner_client.server_is_ready():
            return PatrolReadiness(False, "DEGRADED", "NAV2_PLANNER_UNAVAILABLE")
        if self.mission.checkpoints and not self.spin_client.server_is_ready():
            return PatrolReadiness(False, "DEGRADED", "NAV2_SPIN_UNAVAILABLE")
        costmap = self._costmap_health()
        if not costmap.healthy:
            return PatrolReadiness(False, "FAULT", costmap.reason)
        return readiness

    def _start_readiness(self) -> PatrolReadiness:
        if not self.runtime_binding_valid:
            return PatrolReadiness(False, "FAULT", self.runtime_binding_reason)
        if self.require_fastlio_health:
            fastlio = self.fastlio_health.assess()
            if not fastlio.ready:
                return PatrolReadiness(False, "POSITIONING", fastlio.reason)
        settings = self.bundle.patrol
        readiness = evaluate_readiness(
            expected_version=self.bundle.version_id,
            localization_status=self.localization_status,
            localization_age_s=monotonic_age(self.localization_received_at),
            localization_timeout_s=self.localization_timeout_s,
            pose_xy_yaw=self.pose,
            pose_frame=self.pose_frame,
            pose_age_s=monotonic_age(self.pose_received_at),
            pose_timeout_s=self.pose_timeout_s,
            route=self.bundle.route,
            start_max_distance_m=settings.start_max_distance_m,
            start_max_yaw_deg=settings.start_max_yaw_deg,
            action_server_ready=self.action_client.server_is_ready(),
            require_robot_state=self.require_robot_state,
            robot_motion_status=self.robot_motion_status,
            robot_motion_age_s=monotonic_age(self.robot_motion_received_at),
            robot_motion_timeout_s=self.robot_state_timeout_s,
            maximum_stationary_speed_mps=self.start_stationary_speed_mps,
            maximum_stationary_yaw_rate_rps=(
                self.start_stationary_yaw_rate_rps
            ),
        )
        if not readiness.ready:
            return readiness
        if not self.planner_client.server_is_ready():
            return PatrolReadiness(False, "POSITIONING", "NAV2_PLANNER_UNAVAILABLE")
        if self.mission.checkpoints and not self.spin_client.server_is_ready():
            return PatrolReadiness(False, "POSITIONING", "NAV2_SPIN_UNAVAILABLE")
        costmap = self._costmap_health()
        if not costmap.healthy:
            return PatrolReadiness(
                False,
                # A local costmap is created asynchronously after Nav2 enters
                # the active lifecycle state.  While the runtime is idle this
                # is recoverable startup readiness, not a terminal patrol
                # fault.  The active-motion gate above remains fail-closed and
                # still turns a lost/stale costmap into FAULT while driving.
                "POSITIONING",
                costmap.reason,
                readiness.start_distance_m,
                readiness.start_yaw_error_deg,
            )
        return readiness

    def _runtime_trace_gate(self) -> PatrolReadiness:
        return evaluate_runtime_trace_gate(
            self.runtime_trace_status,
            receive_age_s=monotonic_age(self.runtime_trace_received_at),
            timeout_s=self.runtime_trace_status_timeout_s,
            expected_map_version=self.bundle.version_id,
            expected_manifest_hash=self.bundle.manifest_hash,
        )

    def _start_callback(self, request, response):
        attempt = self._new_start_attempt(request)
        identity_reason = requested_runtime_identity_reason(
            request.expected_map_version,
            request.expected_route_id,
            self.bundle.version_id,
            self.bundle.route.route_id,
        )
        if identity_reason != "OK":
            return self._reject_start(attempt, response, identity_reason)
        if self.goal_handle is not None or self.goal_request_pending:
            return self._reject_start(
                attempt,
                response,
                "ALREADY_PATROLLING",
            )
        readiness = self._start_readiness()
        if not readiness.ready:
            return self._reject_start(
                attempt,
                response,
                readiness.reason,
                readiness,
            )

        self.route_progress_index = 0
        self.checkpoints.reset()
        self.spin_goal_handle = None
        self.spin_request_pending = False
        self.localization_gate_failed_at = None
        self.localization_recovered_at = None
        self.checkpoint_gate_failed_at = None
        self.checkpoint_gate_recovered_at = None
        self.detour_attempt_count = 0
        self.last_detour_compute_ms = None
        self.last_rejoin_index = None
        self.planner_completion = ""
        self.active_controller = ControllerMode.MPPI
        self.mppi_retry_count = 0
        self.recovery_pending = ""
        self.recovery_requested_at = None
        self.recovery_reason = ""
        self.last_controller_result = None
        self.last_failure_class = None
        self.last_route_obstructed = False
        self.obstruction_evidence.reset()
        self.last_obstruction_costmap_sequence = self.costmap_sequence
        self._reset_costmap_refresh()
        self.motion_evidence = MotionEvidenceTracker(
            retention_s=max(12.0, self.progress_timeout_s + 2.0)
        )

        attempt.mark_goal_requested()
        self.active_start_attempt = attempt
        self.last_start_attempt = attempt
        self.goal_request_pending = True
        self.stop_requested = False
        self.runtime_state = "STARTING"
        self.runtime_reason = "NAV2_GLOBAL_PLAN_REQUESTED"
        self.patrol_started_at = time.monotonic()
        if not self._try_global_replan(forced=True):
            self.goal_request_pending = False
            self.runtime_state = "FAULT"
            self.runtime_reason = "NAV2_INITIAL_GLOBAL_PLAN_UNAVAILABLE"
            attempt.mark_goal_decision(False, reason=self.runtime_reason)
            attempt.mark_service_response(
                success=False,
                reason=self.runtime_reason,
            )
            self.last_start_attempt = attempt
            self.active_start_attempt = None
            response.success = False
            response.message = self._response_json(
                self.runtime_reason,
                readiness,
                attempt=attempt,
            )
            return response
        attempt.mark_service_response(
            success=True,
            reason="NAV2_GLOBAL_PLAN_REQUESTED",
        )
        response.success = True
        response.message = self._response_json(
            "NAV2_GLOBAL_PLAN_REQUESTED",
            readiness,
            attempt=attempt,
        )
        return response

    def _goal_response_callback(self, future, attempt_id: str) -> None:
        self.goal_request_pending = False
        attempt = self.active_start_attempt
        if attempt is None or attempt.attempt_id != attempt_id:
            self.runtime_state = "FAULT"
            self.runtime_reason = "START_ATTEMPT_ID_MISMATCH"
            self._finish_active_start_attempt("FAILED", self.runtime_reason)
            return
        try:
            goal_handle = future.result()
        except Exception as exc:  # ROS action transport failure
            if self.stop_requested:
                attempt.mark_goal_decision(False, reason="STOPPED")
                self.runtime_state = "READY"
                self.runtime_reason = "STOPPED"
                self._finish_active_start_attempt("STOPPED", "STOPPED")
                return
            self.runtime_state = "FAULT"
            self.runtime_reason = "NAV2_GOAL_TRANSPORT: %s" % exc
            attempt.mark_goal_decision(False, reason=self.runtime_reason)
            self.last_start_attempt = attempt
            self.active_start_attempt = None
            return
        if not goal_handle.accepted:
            if self.stop_requested:
                attempt.mark_goal_decision(False, reason="STOPPED")
                self.runtime_state = "READY"
                self.runtime_reason = "STOPPED"
                self._finish_active_start_attempt("STOPPED", "STOPPED")
                return
            self.runtime_state = "FAULT"
            self.runtime_reason = "NAV2_GOAL_REJECTED"
            attempt.mark_goal_decision(False, reason=self.runtime_reason)
            self.last_start_attempt = attempt
            self.active_start_attempt = None
            return
        attempt.mark_goal_decision(True, reason="OK")
        self.goal_handle = goal_handle
        if self.stop_requested:
            self.runtime_state = "STOPPING"
            self.runtime_reason = "STOP_REQUESTED"
            goal_handle.cancel_goal_async()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._result_callback)
            return
        self.runtime_state = "PATROLLING"
        self.runtime_reason = "OK"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback) -> None:
        self.last_feedback_distance = float(feedback.feedback.distance_to_goal)
        if self.pose is not None and self.active_controller != ControllerMode.GOAL_MPPI:
            self.route_progress_index = max(
                self.route_progress_index,
                self._nearest_path_index(self.pose[0], self.pose[1]),
            )
        if self.checkpoints.observe_progress(self.route_progress_index):
            self.runtime_state = "CHECKPOINT_PAUSING"
            self.runtime_reason = "CHECKPOINT_REACHED_CANCELLING_ROUTE"
            self.resume_pending = False
            self.recovery_pending = ""
            self.recovery_requested_at = None
            self.recovery_reason = ""
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()

    def _result_callback(self, future) -> None:
        completed_controller = self.active_controller
        result_message = None
        try:
            wrapped = future.result()
            status = wrapped.status
            result_message = getattr(wrapped, "result", None)
            self.last_controller_result = {
                "controller": completed_controller.value,
                "status": int(status),
                "errorCode": getattr(result_message, "error_code", None),
                "errorMessage": getattr(result_message, "error_msg", None),
                "receivedAt": time.time(),
            }
        except Exception as exc:
            status = None
            self.runtime_reason = "NAV2_RESULT_TRANSPORT: %s" % exc
            self.last_controller_result = {
                "controller": completed_controller.value,
                "status": None,
                "transportError": str(exc),
                "receivedAt": time.time(),
            }
        self.goal_handle = None
        self.goal_request_pending = False
        if (
            completed_controller == ControllerMode.GOAL_MPPI
            and status == GoalStatus.STATUS_SUCCEEDED
            and self.last_rejoin_index is not None
        ):
            self.route_progress_index = max(
                self.route_progress_index, self.last_rejoin_index
            )
            self.checkpoints.observe_progress(self.route_progress_index)
        if self.checkpoints.phase == "PAUSING":
            self.checkpoints.route_goal_cancelled()
            self.runtime_state = "CHECKPOINT_SETTLING"
            self.runtime_reason = "CHECKPOINT_ROUTE_CANCELLED_WAITING_TRUE_STOP"
            return
        if self.gate_cancel_reason:
            cancel_reason = self.gate_cancel_reason
            self.runtime_state = (
                "HOLDING"
                if cancel_reason in RECOVERABLE_RUNTIME_GATES
                else "FAULT"
            )
            self.runtime_reason = cancel_reason
            self.gate_cancel_reason = ""
            if self.resume_pending:
                # The final safety chain is already at zero. Keep the same
                # patrol attempt alive and wait for map localization to become
                # usable; the fixed route will then be resent and Nav2 will
                # continue from the closest route point, never from the start.
                return
            attempt_outcome = "FAILED"
        elif self.stop_requested:
            self.runtime_state = "READY"
            self.runtime_reason = "STOPPED"
            attempt_outcome = "STOPPED"
        elif status == GoalStatus.STATUS_SUCCEEDED:
            success_action = controller_success_action(completed_controller)
            if success_action == ControllerSuccessAction.RESUME_MPPI_SUFFIX:
                if self.last_rejoin_index is not None:
                    self.route_progress_index = max(
                        self.route_progress_index, self.last_rejoin_index
                    )
                self.mppi_retry_count = 0
                self._request_mppi_suffix("DETOUR_REJOINED_RESUMING_ROUTE")
                return
            if success_action == ControllerSuccessAction.COMPLETE_PLANNED_GOAL:
                # A planned checkpoint goal is handled by the PAUSING branch
                # above. Reaching the last goal with no checkpoint completes
                # the mission without replaying the recorded polyline.
                self.runtime_state = "COMPLETED"
                self.runtime_reason = "ROUTE_COMPLETE"
                attempt_outcome = "COMPLETED"
                self._finish_active_start_attempt(
                    attempt_outcome, self.runtime_reason
                )
                return
            self.runtime_state = "COMPLETED"
            self.runtime_reason = "ROUTE_COMPLETE"
            attempt_outcome = "COMPLETED"
        else:
            gate = self._runtime_gate()
            if not gate.ready and gate.reason not in RECOVERABLE_RUNTIME_GATES:
                self.runtime_state = "FAULT"
                self.runtime_reason = gate.reason
                self.last_failure_class = "RUNTIME_GATE_FAILED"
                attempt_outcome = "FAILED"
                self._finish_active_start_attempt(
                    attempt_outcome, self.runtime_reason
                )
                return
            route_obstructed = self._route_obstructed()
            costmap_health = self._costmap_health()
            decision = decide_controller_failure(
                FailureEvidence(
                    controller=completed_controller,
                    localization_usable=(
                        gate.ready
                        or not gate.reason.startswith("LOCALIZATION_")
                        and not gate.reason.startswith("FASTLIO_")
                    ),
                    costmap_healthy=costmap_health.healthy,
                    costmap_reason=costmap_health.reason,
                    route_obstructed=route_obstructed,
                    motion=self.motion_evidence.snapshot(
                        time.monotonic(), window_s=self.progress_timeout_s
                    ),
                )
            )
            self.last_failure_class = decision.failure_class.value
            if decision.action == RecoveryAction.HOLD_LOCALIZATION:
                self.resume_pending = True
                self.runtime_state = "HOLDING"
                self.runtime_reason = gate.reason
                return
            if decision.action in {
                RecoveryAction.RETRY_MPPI,
                RecoveryAction.WAIT_COSTMAP,
                RecoveryAction.RETRY_ACTUATION,
            }:
                self.mppi_retry_count += 1
                self._schedule_recovery("MPPI", decision.reason)
                return
            if decision.action == RecoveryAction.START_DETOUR:
                if completed_controller == ControllerMode.GOAL_MPPI:
                    self._schedule_recovery("GOAL", "SEARCHING_FOR_PATH")
                    return
                if self._try_global_replan():
                    return
                self._schedule_recovery("DETOUR", "SEARCHING_FOR_PATH")
                return
            if decision.action == RecoveryAction.SEARCH_PATH:
                self._schedule_recovery(
                    "GOAL" if completed_controller == ControllerMode.GOAL_MPPI else "DETOUR",
                    decision.reason,
                )
                return
            self._schedule_recovery("MPPI", decision.reason)
            return
        self._finish_active_start_attempt(
            attempt_outcome,
            self.runtime_reason,
        )

    def _checkpoint_view_alignment(self, pan_limit_deg: float):
        checkpoint = self.checkpoints.active_checkpoint
        if checkpoint is None or self.pose is None:
            return None
        return plan_checkpoint_view_alignment(
            recorded_body_yaw_rad=float(checkpoint.body_yaw_rad),
            recorded_camera_pan_deg=float(checkpoint.camera_pan_deg),
            current_body_yaw_rad=float(self.pose[2]),
            pan_limit_deg=float(pan_limit_deg),
            body_tolerance_rad=self.checkpoint_body_yaw_tolerance_rad,
        )

    def _complete_checkpoint_pose_with_camera(
        self,
        *,
        pan_limit_deg: float,
        mode: str,
        reason: str,
    ) -> bool:
        if self.checkpoints.phase not in {"POSE_REQUESTED", "POSING"}:
            return False
        alignment = self._checkpoint_view_alignment(pan_limit_deg)
        if (
            alignment is None
            or abs(alignment.body_turn_rad) > self.checkpoint_body_yaw_tolerance_rad
        ):
            return False
        self.checkpoints.set_view_alignment(alignment.with_mode(mode))
        self.checkpoints.pose_completed()
        self.spin_operation = ""
        self.checkpoint_gate_failed_at = None
        self.checkpoint_gate_recovered_at = None
        self.runtime_state = "PAUSED"
        self.runtime_reason = reason
        return True

    def _request_checkpoint_spin(self) -> None:
        checkpoint = self.checkpoints.active_checkpoint
        if checkpoint is None or self.spin_request_pending or self.spin_goal_handle is not None:
            return
        operation = (
            "pose" if self.checkpoints.phase == "POSE_REQUESTED" else "capture"
        )
        if operation == "pose":
            if self.pose is None:
                return
            alignment = self._checkpoint_view_alignment(
                self.checkpoint_camera_preferred_pan_limit_deg
            )
            if alignment is None:
                return
            self.checkpoints.set_view_alignment(alignment)
            target_yaw = alignment.body_turn_rad
            if abs(target_yaw) <= self.checkpoint_body_yaw_tolerance_rad:
                self.checkpoints.pose_completed()
                self.runtime_state = "PAUSED"
                self.runtime_reason = "CHECKPOINT_CAMERA_ALIGNMENT_READY"
                return
        else:
            target_yaw = 2.0 * math.pi
        if not self.spin_client.server_is_ready():
            if operation == "pose" and self._complete_checkpoint_pose_with_camera(
                pan_limit_deg=self.checkpoint_camera_hard_pan_limit_deg,
                mode="camera_fallback",
                reason="CHECKPOINT_CAMERA_FALLBACK_READY",
            ):
                return
            self.checkpoints.fail("NAV2_SPIN_UNAVAILABLE")
            self.runtime_state = "BLOCKED"
            self.runtime_reason = "CHECKPOINT_SPIN_UNAVAILABLE"
            return
        goal = Spin.Goal()
        goal.target_yaw = float(target_yaw)
        goal.time_allowance.sec = 30
        goal.time_allowance.nanosec = 0
        self.spin_request_pending = True
        self.spin_operation = operation
        self.runtime_state = "INSPECTING"
        self.runtime_reason = (
            "CHECKPOINT_POSE_REQUESTED"
            if operation == "pose"
            else "CHECKPOINT_SPIN_REQUESTED"
        )
        try:
            future = self.spin_client.send_goal_async(goal)
        except Exception as exc:
            self.spin_request_pending = False
            if operation == "pose" and self._complete_checkpoint_pose_with_camera(
                pan_limit_deg=self.checkpoint_camera_hard_pan_limit_deg,
                mode="camera_fallback",
                reason="CHECKPOINT_CAMERA_FALLBACK_READY",
            ):
                self.get_logger().warning(
                    "checkpoint body pose transport failed; camera fallback selected: %s"
                    % exc
                )
                return
            self.checkpoints.fail("NAV2_SPIN_TRANSPORT: %s" % exc)
            self.runtime_state = "BLOCKED"
            self.runtime_reason = "CHECKPOINT_SPIN_TRANSPORT"
            return
        future.add_done_callback(self._checkpoint_spin_response_callback)

    def _checkpoint_spin_response_callback(self, future) -> None:
        self.spin_request_pending = False
        transport_error = None
        try:
            goal_handle = future.result()
        except Exception as exc:
            goal_handle = None
            transport_error = exc
        if goal_handle is None or not goal_handle.accepted:
            if self.spin_operation == "pose" and self._complete_checkpoint_pose_with_camera(
                pan_limit_deg=self.checkpoint_camera_hard_pan_limit_deg,
                mode="camera_fallback",
                reason="CHECKPOINT_CAMERA_FALLBACK_READY",
            ):
                self.get_logger().warning(
                    "checkpoint body pose was unavailable; camera fallback selected"
                )
                return
            if self.checkpoints.phase != "FAILED":
                self.checkpoints.fail(
                    "NAV2_SPIN_TRANSPORT: %s" % transport_error
                    if transport_error is not None
                    else "NAV2_SPIN_REJECTED"
                )
            self.runtime_state = "BLOCKED"
            self.runtime_reason = self.checkpoints.failure_reason or "NAV2_SPIN_REJECTED"
            return
        self.spin_goal_handle = goal_handle
        if self.stop_requested:
            goal_handle.cancel_goal_async()
            goal_handle.get_result_async().add_done_callback(
                self._checkpoint_spin_result_callback
            )
            return
        if self.spin_operation == "pose":
            self.checkpoints.pose_started()
        else:
            self.checkpoints.spin_started()
        self.runtime_state = "INSPECTING"
        self.runtime_reason = (
            "CHECKPOINT_POSING_ACTIVE"
            if self.spin_operation == "pose"
            else "CHECKPOINT_SPIN_ACTIVE"
        )
        goal_handle.get_result_async().add_done_callback(
            self._checkpoint_spin_result_callback
        )

    def _checkpoint_spin_result_callback(self, future) -> None:
        self.spin_goal_handle = None
        result_error = None
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:
            status = None
            result_error = exc
        if self.stop_requested:
            self.checkpoints.fail("STOPPED")
            self.runtime_state = "READY"
            self.runtime_reason = "STOPPED"
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            if self.spin_operation == "pose" and self._complete_checkpoint_pose_with_camera(
                pan_limit_deg=self.checkpoint_camera_hard_pan_limit_deg,
                mode="camera_fallback",
                reason="CHECKPOINT_CAMERA_FALLBACK_READY",
            ):
                self.get_logger().warning(
                    "checkpoint body pose did not complete; remaining yaw moved to camera"
                )
                return
            if self.checkpoints.phase != "FAILED":
                self.checkpoints.fail(
                    "NAV2_SPIN_RESULT_TRANSPORT: %s" % result_error
                    if result_error is not None
                    else "CHECKPOINT_VIEW_UNREACHABLE_AFTER_BODY_TURN"
                )
            self.runtime_state = "BLOCKED"
            self.runtime_reason = self.checkpoints.failure_reason or "NAV2_SPIN_FAILED"
            return
        operation = self.spin_operation
        self.spin_operation = ""
        if operation == "pose":
            previous = self.checkpoints.view_alignment
            if previous is None:
                self.checkpoints.fail("CHECKPOINT_VIEW_UNREACHABLE_AFTER_BODY_TURN")
                self.runtime_state = "BLOCKED"
                self.runtime_reason = self.checkpoints.failure_reason
                return
            # Nav2 has accepted the body yaw goal as complete. Keep the
            # original body/camera allocation instead of recomputing from a
            # localization sample that may trail the action result callback.
            self.checkpoints.set_view_alignment(previous.with_mode("body_plus_camera"))
            self.checkpoints.pose_completed()
            self.checkpoint_gate_failed_at = None
            self.checkpoint_gate_recovered_at = None
            self.runtime_state = "PAUSED"
            self.runtime_reason = "CHECKPOINT_POSE_READY_WAITING_PLATFORM"
            return
        self.checkpoints.spin_completed()
        if self.checkpoints.phase == "TRAVELING":
            self._resume_after_checkpoint()
        else:
            self.runtime_state = "PAUSED"
            self.runtime_reason = "CHECKPOINT_CAPTURE_COMPLETE_WAITING_VERDICT"

    def _resume_after_checkpoint(self) -> None:
        # Route motion is not reauthorized until Nav2 accepts the retained suffix.
        self.resume_pending = True
        self.localization_recovered_at = None
        self._reset_costmap_refresh()
        self.runtime_state = "HOLDING"
        self.runtime_reason = "CHECKPOINT_COMPLETE_WAITING_ROUTE_SUFFIX"

    def _apply_checkpoint_control(self) -> str:
        try:
            control = json.loads(self.checkpoint_control_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, OSError, ValueError):
            return "unavailable"
        before = self.checkpoints.phase
        result = self.checkpoints.apply_platform_control(control)
        if before == "WAITING_VERDICT" and self.checkpoints.phase == "TRAVELING":
            self._resume_after_checkpoint()
        return result

    def _tick_checkpoint(self, now: float, gate: PatrolReadiness) -> bool:
        phase = self.checkpoints.phase
        if phase != self.checkpoint_observed_phase:
            self.checkpoint_observed_phase = phase
            self.checkpoint_phase_started_at = now
            if phase not in {"POSING", "SPINNING"}:
                self.checkpoint_gate_failed_at = None
                self.checkpoint_gate_recovered_at = None
        if phase == "PAUSING":
            self.runtime_state = "CHECKPOINT_PAUSING"
            self.runtime_reason = "CHECKPOINT_REACHED_CANCELLING_ROUTE"
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            return False
        if phase == "SETTLING":
            self.runtime_state = "CHECKPOINT_SETTLING"
            self.runtime_reason = "CHECKPOINT_WAITING_TRUE_STOP"
            command_fresh = now - self.last_final_command_at <= 0.5
            command = self.last_final_command if command_fresh else None
            command_linear = float(command[0]) if command is not None else math.inf
            command_angular = float(command[1]) if command is not None else math.inf
            robot = self.robot_motion_status if isinstance(self.robot_motion_status, dict) else None
            robot_fresh = (
                not self.require_robot_state
                or now - self.robot_motion_received_at <= self.robot_state_timeout_s
            )
            if not self.require_robot_state:
                robot_linear = 0.0
                robot_angular = 0.0
            elif robot is not None and robot_fresh:
                robot_linear = math.hypot(float(robot["vx"]), float(robot["vy"]))
                robot_angular = abs(float(robot["yawSpeed"]))
            else:
                robot_linear = math.inf
                robot_angular = math.inf
            ready = self.checkpoints.observe_stop(
                now,
                motion_authorized=False,
                command_linear_mps=command_linear,
                command_angular_rps=command_angular,
                robot_linear_mps=robot_linear,
                robot_angular_rps=robot_angular,
                route_progress_index=self.route_progress_index,
            )
            if ready:
                self._request_checkpoint_spin()
            return False
        if phase == "POSE_REQUESTED":
            self.runtime_state = "INSPECTING"
            self.runtime_reason = "CHECKPOINT_POSE_REQUESTED"
            if gate.ready:
                self._request_checkpoint_spin()
            else:
                self.runtime_state = "HOLDING"
                self.runtime_reason = "CHECKPOINT_POSE_WAITING_%s" % gate.reason
            return False
        if phase == "POSING":
            if not gate.ready:
                if gate.reason in RECOVERABLE_RUNTIME_GATES:
                    if self.checkpoint_gate_failed_at is None:
                        self.checkpoint_gate_failed_at = now
                    self.checkpoint_gate_recovered_at = None
                    self.runtime_state = "HOLDING"
                    self.runtime_reason = "CHECKPOINT_POSE_HOLD_%s" % gate.reason
                    return False
                if self.spin_goal_handle is not None:
                    self.spin_goal_handle.cancel_goal_async()
                self.checkpoints.fail("CHECKPOINT_POSE_RUNTIME_GATE: %s" % gate.reason)
                self.runtime_state = "BLOCKED"
                self.runtime_reason = self.checkpoints.failure_reason
                return False
            if self.checkpoint_gate_failed_at is not None:
                if self.checkpoint_gate_recovered_at is None:
                    self.checkpoint_gate_recovered_at = now
                if (
                    now - self.checkpoint_gate_recovered_at
                    < self.localization_recovery_stable_s
                ):
                    self.runtime_state = "HOLDING"
                    self.runtime_reason = "CHECKPOINT_POSE_WAITING_STABLE_LOCALIZATION"
                    return False
                self.checkpoint_gate_failed_at = None
                self.checkpoint_gate_recovered_at = None
            self.runtime_state = "INSPECTING"
            self.runtime_reason = "CHECKPOINT_POSING_ACTIVE"
            return self.spin_goal_handle is not None
        if phase == "WAITING_PLATFORM":
            self.runtime_state = "PAUSED"
            self.runtime_reason = "CHECKPOINT_POSE_READY_WAITING_PLATFORM"
            result = self._apply_checkpoint_control()
            if self.checkpoints.phase == "SPIN_REQUESTED":
                self._request_checkpoint_spin()
            elif (
                self.mission.decision_mode == "platform"
                and now - self.checkpoint_phase_started_at >= (
                25.0
                + float(
                    self.checkpoints.active_checkpoint.dwell_s
                    if self.checkpoints.active_checkpoint is not None else 0.0
                )
                )
            ):
                # Platform/announcement failure must not strand the patrol.
                self.checkpoints.request_spin()
                if self.checkpoints.phase == "SPIN_REQUESTED":
                    self._request_checkpoint_spin()
            return False
        if phase == "SPIN_REQUESTED":
            self.runtime_state = "INSPECTING"
            self.runtime_reason = "CHECKPOINT_SPIN_REQUESTED"
            return False
        if phase == "SPINNING":
            if not gate.ready:
                if gate.reason in RECOVERABLE_RUNTIME_GATES:
                    if self.checkpoint_gate_failed_at is None:
                        self.checkpoint_gate_failed_at = now
                    self.checkpoint_gate_recovered_at = None
                    self.runtime_state = "HOLDING"
                    self.runtime_reason = "CHECKPOINT_SPIN_HOLD_%s" % gate.reason
                    return False
                if self.spin_goal_handle is not None:
                    self.spin_goal_handle.cancel_goal_async()
                self.checkpoints.fail("CHECKPOINT_SPIN_RUNTIME_GATE: %s" % gate.reason)
                self.runtime_state = "BLOCKED"
                self.runtime_reason = self.checkpoints.failure_reason
                return False
            if self.checkpoint_gate_failed_at is not None:
                if self.checkpoint_gate_recovered_at is None:
                    self.checkpoint_gate_recovered_at = now
                if (
                    now - self.checkpoint_gate_recovered_at
                    < self.localization_recovery_stable_s
                ):
                    self.runtime_state = "HOLDING"
                    self.runtime_reason = "CHECKPOINT_SPIN_WAITING_STABLE_LOCALIZATION"
                    return False
                self.checkpoint_gate_failed_at = None
                self.checkpoint_gate_recovered_at = None
            self.runtime_state = "INSPECTING"
            self.runtime_reason = "CHECKPOINT_SPIN_ACTIVE"
            return self.spin_goal_handle is not None
        if phase == "WAITING_VERDICT":
            self.runtime_state = "PAUSED"
            self.runtime_reason = "CHECKPOINT_WAITING_PLATFORM_VERDICT"
            self._apply_checkpoint_control()
            if (
                self.mission.decision_mode == "platform"
                and self.checkpoints.phase == "WAITING_VERDICT"
                and now - self.checkpoint_phase_started_at
                >= float(self.mission.verdict_timeout_s) + 2.0
            ):
                self.checkpoints.complete_checkpoint()
                self._resume_after_checkpoint()
            return False
        return False

    def _stop_callback(self, request, response):
        del request
        self.stop_requested = True
        self.resume_pending = False
        self.recovery_pending = ""
        self.recovery_requested_at = None
        self.recovery_reason = ""
        self._reset_costmap_refresh()
        self.runtime_state = "STOPPING"
        self.runtime_reason = "STOP_REQUESTED"
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.planner_goal_handle is not None:
            self.planner_goal_handle.cancel_goal_async()
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()
        if self.checkpoints.active:
            self.checkpoints.fail("STOPPED")
        elif not self.goal_request_pending:
            self._finish_active_start_attempt("STOPPED", "STOP_REQUESTED")
        response.success = True
        response.message = self._response_json("STOP_REQUESTED")
        return response

    def _cancel_for_gate(self, reason: str) -> None:
        if self.gate_cancel_reason:
            return
        self.runtime_state = "DEGRADED"
        self.runtime_reason = reason
        self.gate_cancel_reason = reason
        if self.goal_handle is not None:
            self.resume_pending = reason in RECOVERABLE_RUNTIME_GATES
            if self.resume_pending:
                self._reset_costmap_refresh()
        if self.goal_handle is not None:
            self.stop_requested = False
            self.goal_handle.cancel_goal_async()
        if self.planner_goal_handle is not None:
            self.planner_goal_handle.cancel_goal_async()

    def _request_resume(self) -> None:
        if self.goal_handle is not None or self.goal_request_pending:
            return
        if self.bundle.navigation_map_path is not None:
            self.resume_pending = False
            if not self._try_global_replan(forced=True):
                self.resume_pending = True
                self.localization_recovered_at = None
                self.runtime_state = "HOLDING"
                self.runtime_reason = "NAV2_NEXT_GOAL_PLAN_WAITING"
            return
        resume_path = self._remaining_path()
        if len(resume_path.poses) < 2:
            self.resume_pending = False
            self.runtime_state = "COMPLETED"
            self.runtime_reason = "ROUTE_COMPLETE"
            self._finish_active_start_attempt("COMPLETED", self.runtime_reason)
            return
        now = self.get_clock().now().to_msg()
        resume_path.header.stamp = now
        for pose in resume_path.poses:
            pose.header.stamp = now
        goal = FollowPath.Goal()
        goal.path = resume_path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "route_goal_checker"
        self.active_controller = ControllerMode.MPPI
        self._reset_costmap_refresh()
        self.goal_request_pending = True
        self.runtime_state = "RESUMING"
        self.runtime_reason = "LOCALIZATION_RECOVERED_RESUMING_ROUTE"
        try:
            future = self.action_client.send_goal_async(
                goal,
                feedback_callback=self._feedback_callback,
            )
        except Exception as exc:
            self.goal_request_pending = False
            self.runtime_reason = "NAV2_RESUME_TRANSPORT: %s" % exc
            self.resume_pending = True
            self.localization_recovered_at = None
            self._reset_costmap_refresh()
            return
        future.add_done_callback(self._resume_goal_response_callback)

    def _resume_goal_response_callback(self, future) -> None:
        self.goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            goal_handle = None
            self.runtime_reason = "NAV2_RESUME_TRANSPORT: %s" % exc
        if goal_handle is None or not goal_handle.accepted:
            if not self.runtime_reason.startswith("NAV2_RESUME_TRANSPORT"):
                self.runtime_reason = "NAV2_RESUME_REJECTED"
            self.resume_pending = True
            self.localization_recovered_at = None
            self._reset_costmap_refresh()
            self.runtime_state = "HOLDING"
            return
        self.goal_handle = goal_handle
        if self.stop_requested:
            self.runtime_state = "STOPPING"
            self.runtime_reason = "STOP_REQUESTED"
            goal_handle.cancel_goal_async()
            goal_handle.get_result_async().add_done_callback(self._result_callback)
            return
        self.resume_pending = False
        self.resume_count += 1
        self.runtime_state = "PATROLLING"
        self.runtime_reason = "LOCALIZATION_RECOVERED_ROUTE_RESUMED"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _verify_runtime_binding(self) -> None:
        now = time.monotonic()
        if now - self.last_runtime_binding_check < self.active_map_check_period_s:
            return
        self.last_runtime_binding_check = now
        # A changed immutable binding is a latched fault.  Restoring a file or
        # pointer cannot silently re-authorize motion; the runtime must restart
        # and complete the full startup verification again.
        if not self.runtime_binding_valid:
            return
        try:
            valid, reason = self.runtime_guard.verify()
            if not valid:
                raise RuntimeError(reason)
            if (
                self.runtime_source == "active"
                and now - self.last_full_map_audit >= self.full_map_audit_period_s
            ):
                self.last_full_map_audit = now
                load_runtime_bundle(
                    self.map_store_root,
                    self.site_id,
                    self.bundle.version_id,
                    store=self.map_store,
                )
            self.runtime_binding_valid = True
            self.runtime_binding_reason = "OK"
        except Exception as exc:
            self.runtime_binding_valid = False
            self.runtime_binding_reason = "RUNTIME_BINDING_INVALID: %s" % exc
            self.runtime_binding_fault_at = now

    def _tick(self) -> None:
        now = time.monotonic()
        self._verify_runtime_binding()
        self._update_obstruction_evidence(now)
        gate = self._runtime_gate()
        if self.checkpoints.active:
            motion_authorized = self._tick_checkpoint(now, gate)
            self.current_motion_authorized = bool(motion_authorized)
            self._publish_status(motion_authorized)
            return
        accepted_goal = self.goal_handle is not None
        motion_authorized = accepted_goal and gate.ready and self.runtime_state == "PATROLLING"
        if (accepted_goal or self.goal_request_pending) and not gate.ready:
            if gate.reason in RECOVERABLE_RUNTIME_GATES:
                if self.localization_gate_failed_at is None:
                    self.localization_gate_failed_at = now
                elapsed = now - self.localization_gate_failed_at
                self.runtime_state = "HOLDING"
                self.runtime_reason = gate.reason
                if elapsed >= self.localization_dropout_grace_s:
                    self._cancel_for_gate(gate.reason)
            else:
                self._cancel_for_gate(gate.reason)
            motion_authorized = False
        elif (
            accepted_goal
            and gate.ready
            and self.runtime_state == "HOLDING"
            and self.localization_gate_failed_at is not None
            and not self.gate_cancel_reason
        ):
            # A brief status/TF/costmap gap does not require canceling the Nav2
            # goal. Restore the exact same goal once the runtime gate is fresh.
            self.localization_gate_failed_at = None
            self.localization_recovered_at = None
            self.runtime_state = "PATROLLING"
            self.runtime_reason = "RUNTIME_TRANSIENT_RECOVERED"
            motion_authorized = True
        elif (
            self.recovery_pending
            and not accepted_goal
            and not self.goal_request_pending
        ):
            motion_authorized = False
            if not gate.ready:
                if gate.reason in RECOVERABLE_RUNTIME_GATES:
                    self.runtime_state = (
                        "SEARCHING_PATH"
                        if self.recovery_pending in {"DETOUR", "GOAL"}
                        else "RECOVERING"
                    )
                    self.runtime_reason = gate.reason
                else:
                    self.recovery_pending = ""
                    self.recovery_requested_at = None
                    self.runtime_state = "FAULT"
                    self.runtime_reason = gate.reason
                    self._finish_active_start_attempt("FAILED", gate.reason)
            elif (
                self.recovery_requested_at is not None
                and now - self.recovery_requested_at >= self.replan_interval_s
            ):
                mode = self.recovery_pending
                reason = self.recovery_reason or "TRANSIENT_CONTROL_RETRY"
                if mode == "GOAL":
                    self.recovery_pending = ""
                    self.recovery_requested_at = None
                    self.recovery_reason = ""
                    if not self._try_global_replan(forced=True):
                        self._schedule_recovery("GOAL", "SEARCHING_FOR_PATH")
                elif mode == "DETOUR":
                    if not self._route_obstructed():
                        self._schedule_recovery(
                            "MPPI", "OBSTRUCTION_CLEARED_RESUMING_ROUTE"
                        )
                    else:
                        self.recovery_pending = ""
                        self.recovery_requested_at = None
                        self.recovery_reason = ""
                        if not self._try_global_replan():
                            self._schedule_recovery(
                                "DETOUR", "SEARCHING_FOR_PATH"
                            )
                else:
                    refresh = self._resume_costmap_refresh(now)
                    if refresh.ready:
                        self.recovery_pending = ""
                        self.recovery_requested_at = None
                        self.recovery_reason = ""
                        self._request_mppi_suffix(reason)
                    elif refresh.state == "FAULT":
                        # Clearing and fresh-map acquisition are also transient
                        # operations. Keep retrying at the configured cadence.
                        self._reset_costmap_refresh()
                        self.recovery_requested_at = now
                        self.runtime_state = "RECOVERING"
                        self.runtime_reason = refresh.reason
                    else:
                        self.runtime_state = "RECOVERING"
                        self.runtime_reason = refresh.reason
        elif self.resume_pending and not accepted_goal and not self.goal_request_pending:
            costmap_only_gate = gate.reason.startswith("COSTMAP_")
            if gate.ready or costmap_only_gate:
                if self.localization_recovered_at is None:
                    self.localization_recovered_at = time.monotonic()
                if time.monotonic() - self.localization_recovered_at >= self.localization_recovery_stable_s:
                    refresh = self._resume_costmap_refresh(now)
                    if refresh.ready:
                        self._request_resume()
                    elif refresh.state == "FAULT":
                        self._reset_costmap_refresh()
                        self.localization_recovered_at = now
                        self.runtime_state = "HOLDING"
                        self.runtime_reason = refresh.reason
                    else:
                        self.runtime_state = refresh.state
                        self.runtime_reason = refresh.reason
            elif gate.reason not in RECOVERABLE_RUNTIME_GATES:
                self.resume_pending = False
                self.runtime_state = "FAULT"
                self.runtime_reason = gate.reason
                self._finish_active_start_attempt("FAILED", gate.reason)
            else:
                self.localization_recovered_at = None
                self.runtime_state = gate.state
                self.runtime_reason = gate.reason
            motion_authorized = False
        else:
            self.localization_gate_failed_at = None
        if not accepted_goal and not self.goal_request_pending and not self.resume_pending and self.runtime_state not in {
            "COMPLETED",
            "FAULT",
            "BLOCKED",
            "STOPPING",
        } and not self.recovery_pending:
            readiness = self._start_readiness()
            self.runtime_state = readiness.state
            self.runtime_reason = readiness.reason
        if motion_authorized and not self.current_motion_authorized:
            attempt = self.active_start_attempt
            if attempt is not None:
                try:
                    attempt.mark_motion_authorized()
                except RuntimeError as exc:
                    motion_authorized = False
                    self._cancel_for_gate("START_TIMING_INVALID: %s" % exc)
        self.current_motion_authorized = bool(motion_authorized)
        self._publish_status(motion_authorized)
        if (
            not self.shutdown_requested
            and self.runtime_binding_fault_at is not None
            and time.monotonic() - self.runtime_binding_fault_at
            >= self.binding_shutdown_delay_s
        ):
            # The launch file treats this process as the owner of the complete
            # localization/navigation generation.  Leaving after a short
            # zero-command drain forces the whole generation to restart and
            # bind one coherent active release; no node may hot-swap alone.
            self.shutdown_requested = True
            self.exit_code = 75
            self.get_logger().error(
                "runtime binding changed; stopping the complete runtime generation"
            )
            if rclpy.ok():
                rclpy.shutdown()

    def _publish_status(self, motion_authorized: bool) -> None:
        self.status_sequence += 1
        fastlio_health = self.fastlio_health.assess().as_dict()
        fastlio_health["required"] = self.require_fastlio_health
        trace_health = self._runtime_trace_gate()
        costmap_health = self._costmap_health()
        motion_evidence = self.motion_evidence.snapshot(
            time.monotonic(), window_s=self.progress_timeout_s
        )
        authorization = Bool()
        authorization.data = bool(motion_authorized)
        self.authorization_publisher.publish(authorization)
        status = String()
        status_payload = {
            "schema": "go2.runtime_status.v1",
            "runtimeInstanceId": self.runtime_instance_id,
            "statusSequence": self.status_sequence,
            "publishedAt": time.time(),
            "siteId": self.bundle.site_id,
            "mapVersion": self.bundle.version_id,
            "manifestHash": self.bundle.manifest_hash,
            "state": self.runtime_state,
            "reason": self.runtime_reason,
            "operatorMessage": operator_message_for_reason(self.runtime_reason),
            "motionAuthorized": bool(motion_authorized),
            "routeHash": self.bundle.route.source_hash,
            "runtimeProfileHash": self.runtime_profile_hash,
            "nav2ProfileHash": self.nav2_profile_hash,
            "controllerProfileId": "go2-nav2-mppi-omni-v1",
            "collisionProfileId": "go2-mid360-collision-v1",
            "calibrationHash": self.bundle.calibration_hash,
            "robotId": self.bundle.robot_id,
            "sensorId": self.bundle.sensor_id,
            "routeId": self.bundle.route.route_id,
            "routeLengthM": self.bundle.route.length_m,
            "routePointCount": len(self.path.poses),
            "distanceToGoalM": self.last_feedback_distance,
            "routeProgressIndex": self.route_progress_index,
            "routeProgressPercent": round(
                100.0 * self.route_progress_index / max(1, len(self.path.poses) - 1), 1
            ),
            "remainingRoutePointCount": max(0, len(self.path.poses) - self.route_progress_index),
            "activeController": (
                self.active_controller.value
                if self.goal_handle is not None
                else None
            ),
            "selectedController": self.active_controller.value,
            "controllerOwnership": (
                "NONE"
                if self.goal_handle is None
                else (
                    "MPPI_NAV2_GLOBAL_DETOUR_TO_REJOIN"
                    if self.active_controller == ControllerMode.DETOUR_MPPI
                    else "MPPI_RECORDED_ROUTE"
                )
            ),
            "failureClass": self.last_failure_class,
            "lastControllerResult": self.last_controller_result,
            "mppiRetryCount": self.mppi_retry_count,
            "replanIntervalS": self.replan_interval_s,
            "recoveryPending": self.recovery_pending or None,
            "recoveryReason": self.recovery_reason or None,
            "routeObstructed": self.last_route_obstructed,
            "costmapHealth": {
                "healthy": costmap_health.healthy,
                "reason": costmap_health.reason,
                "sequence": self.costmap_sequence,
                "frameId": costmap_health.frame_id,
                "ageS": costmap_health.age_s,
                "robotCost": costmap_health.robot_cost,
            },
            "costmapRefresh": {
                "active": self.costmap_refresh_started_at is not None,
                "baselineSequence": self.costmap_refresh_baseline_sequence,
                "elapsedS": (
                    monotonic_age(self.costmap_refresh_started_at)
                    if self.costmap_refresh_started_at is not None
                    else None
                ),
                "clearResponseReceived": (
                    self.costmap_refresh_future.done()
                    if self.costmap_refresh_future is not None
                    else False
                ),
            },
            "motionEvidence": {
                "windowS": motion_evidence.window_s,
                "observedDurationS": motion_evidence.observed_duration_s,
                "translationM": motion_evidence.translation_m,
                "rotationRad": motion_evidence.rotation_rad,
                "meanLinearCommandMps": motion_evidence.mean_linear_command_mps,
                "meanAngularCommandRps": motion_evidence.mean_angular_command_rps,
                "linearCommandActiveRatio": motion_evidence.linear_command_active_ratio,
                "angularCommandActiveRatio": motion_evidence.angular_command_active_ratio,
            },
            "detourAttemptCount": self.detour_attempt_count,
            "detourComputeMs": self.last_detour_compute_ms,
            "rejoinRouteIndex": self.last_rejoin_index,
            "resumePending": self.resume_pending,
            "resumeCount": self.resume_count,
            "checkpoint": self.checkpoints.status(),
            "robotStateRequired": self.require_robot_state,
            "robotMode": (
                self.robot_motion_status.get("mode")
                if isinstance(self.robot_motion_status, dict)
                else None
            ),
            "robotErrorCode": (
                self.robot_motion_status.get("errorCode")
                if isinstance(self.robot_motion_status, dict)
                else None
            ),
            "robotStateAgeS": (
                monotonic_age(self.robot_motion_received_at)
                if self.require_robot_state
                else None
            ),
            "fastlioHealth": fastlio_health,
            "runtimeTraceHealth": {
                "ready": trace_health.ready,
                "reason": trace_health.reason,
                "ageS": monotonic_age(self.runtime_trace_received_at),
                "writable": (
                    self.runtime_trace_status.get("writable")
                    if isinstance(self.runtime_trace_status, dict)
                    else None
                ),
                "path": (
                    self.runtime_trace_status.get("path")
                    if isinstance(self.runtime_trace_status, dict)
                    else None
                ),
                "recordsWritten": (
                    self.runtime_trace_status.get("recordsWritten")
                    if isinstance(self.runtime_trace_status, dict)
                    else None
                ),
            },
            "startTiming": (
                self.active_start_attempt.snapshot()
                if self.active_start_attempt is not None
                else (
                    self.last_start_attempt.snapshot()
                    if self.last_start_attempt is not None
                    else None
                )
            ),
            "activeStartAttempt": (
                self.active_start_attempt.snapshot()
                if self.active_start_attempt is not None
                else None
            ),
            "lastStartAttempt": (
                self.last_start_attempt.snapshot()
                if self.last_start_attempt is not None
                else None
            ),
        }
        status.data = json.dumps(
            status_payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_publisher.publish(status)

    def _response_json(
        self,
        reason: str,
        readiness: Optional[PatrolReadiness] = None,
        attempt: Optional[StartAttemptTimeline] = None,
    ) -> str:
        payload = {
            "schema": "go2.patrol_response.v1",
            "mapVersion": self.bundle.version_id,
            "routeId": self.bundle.route.route_id,
            "state": self.runtime_state,
            "reason": reason,
            "operatorMessage": operator_message_for_reason(
                reason,
                start_distance_m=(
                    readiness.start_distance_m if readiness is not None else None
                ),
                start_yaw_error_deg=(
                    readiness.start_yaw_error_deg if readiness is not None else None
                ),
            ),
            "stage": attempt.stage if attempt is not None else None,
            "nav2GoalAccepted": (
                attempt.nav2_goal_accepted if attempt is not None else None
            ),
            "startTiming": attempt.snapshot() if attempt is not None else None,
        }
        if readiness is not None:
            payload["startDistanceM"] = readiness.start_distance_m
            payload["startYawErrorDeg"] = readiness.start_yaw_error_deg
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = PatrolRuntimeManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            exit_code = node.exit_code
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
