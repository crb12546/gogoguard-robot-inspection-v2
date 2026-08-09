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
from nav2_msgs.action import FollowPath
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
from gogoguard_navigation.detour_diagnostics import plan_detour_diagnostic
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


RECOVERABLE_LOCALIZATION_GATES = frozenset(
    {
        "LOCALIZATION_STATUS_STALE",
        "LOCALIZATION_NOT_TRACKING",
        "LOCALIZATION_NOT_USABLE",
        "LOCALIZATION_POSE_MISSING",
        "LOCALIZATION_POSE_STALE",
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
        self.declare_parameter("nav2_profile_hash", "")
        self.declare_parameter("candidate.localization_map", "")
        self.declare_parameter("candidate.route", "")
        self.declare_parameter("candidate.runtime_profile", "")
        self.declare_parameter("candidate.localization_map_hash", "")
        self.declare_parameter("candidate.route_hash", "")
        self.declare_parameter("candidate.runtime_profile_hash", "")
        self.declare_parameter("follow_path_action", "/follow_path")
        self.declare_parameter("localization_status_topic", "/localization/status")
        self.declare_parameter("localization_pose_topic", "/localization/pose")
        self.declare_parameter("localization_timeout_s", 0.20)
        self.declare_parameter("localization_dropout_grace_s", 1.0)
        self.declare_parameter("localization_recovery_stable_s", 0.5)
        self.declare_parameter("progress_timeout_s", 5.0)
        self.declare_parameter("mppi_retry_limit", 2)
        self.declare_parameter("detour_attempt_limit", 2)
        self.declare_parameter("obstruction_cost_threshold", 65)
        self.declare_parameter("obstruction_min_samples", 2)
        self.declare_parameter("obstruction_confirmation_s", 0.30)
        self.declare_parameter("costmap_timeout_s", 1.0)
        self.declare_parameter("rejoin_lookahead_m", 2.0)
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
            }
            self.bundle = load_candidate_runtime_bundle(
                site_id=self.site_id,
                version_id=expected,
                localization_map_path=candidate_paths["localization_map"],
                route_path=candidate_paths["route"],
                runtime_profile_path=candidate_paths["runtime_profile"],
                localization_map_hash=self.get_parameter(
                    "candidate.localization_map_hash"
                ).value,
                route_hash=self.get_parameter("candidate.route_hash").value,
                runtime_profile_hash=self.get_parameter(
                    "candidate.runtime_profile_hash"
                ).value,
            )
            self.runtime_guard = RuntimeArtifactGuard.capture(
                {
                    "localization_map": self.bundle.localization_map_path,
                    "route": self.bundle.route_path,
                    "runtime_profile": self.bundle.runtime_profile_path,
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
        self.progress_timeout_s = float(
            self.get_parameter("progress_timeout_s").value
        )
        self.mppi_retry_limit = int(self.get_parameter("mppi_retry_limit").value)
        self.detour_attempt_limit = int(
            self.get_parameter("detour_attempt_limit").value
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
        if not 2.0 <= self.progress_timeout_s <= 12.0:
            raise RuntimeError("progress_timeout_s is invalid")
        if not 0 <= self.mppi_retry_limit <= 3:
            raise RuntimeError("mppi_retry_limit is invalid")
        if not 1 <= self.detour_attempt_limit <= 3:
            raise RuntimeError("detour_attempt_limit is invalid")
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
        self.localization_gate_failed_at = None
        self.localization_recovered_at = None
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
        self.active_controller = ControllerMode.MPPI
        self.mppi_retry_count = 0
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
        if self.goal_handle is not None or self.goal_request_pending or self.resume_pending:
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

    def _route_and_rejoin(self):
        route_xy = [
            (item.pose.position.x, item.pose.position.y) for item in self.path.poses
        ]
        if len(route_xy) < 2:
            return route_xy, None
        rejoin_index = route_rejoin_index(
            route_xy, self.route_progress_index, self.rejoin_lookahead_m
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

    def _nav_path_from_points(self, points) -> Optional[NavPath]:
        if len(points) < 2:
            return None
        path = NavPath()
        path.header.frame_id = "map"
        now = self.get_clock().now().to_msg()
        for index, point in enumerate(points):
            following = points[index + 1] if index + 1 < len(points) else points[index - 1]
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation = _quaternion_from_yaw(
                math.atan2(following[1] - point[1], following[0] - point[0])
            )
            path.poses.append(pose)
        return path

    def _try_local_replan(self) -> bool:
        if (
            self.pose is None
            or self.local_costmap is None
            or not self._costmap_health().healthy
            or self.detour_attempt_count >= self.detour_attempt_limit
            or not self._route_obstructed()
        ):
            return False
        route_xy, rejoin_index = self._route_and_rejoin()
        if rejoin_index is None:
            return False
        start_in_grid = self._transform_xy(
            self.pose[0], self.pose[1], self.local_costmap_frame, "map"
        )
        goal_in_grid = self._transform_xy(
            route_xy[rejoin_index][0],
            route_xy[rejoin_index][1],
            self.local_costmap_frame,
            "map",
        )
        if start_in_grid is None or goal_in_grid is None:
            return False
        started = time.monotonic()
        detour, planner_diagnostics = plan_detour_diagnostic(
            self.local_costmap,
            start_in_grid,
            goal_in_grid,
            occupied_threshold=self.obstruction_cost_threshold,
        )
        self.last_detour_compute_ms = round(
            (time.monotonic() - started) * 1000.0, 2
        )
        planner_diagnostics["computeMs"] = self.last_detour_compute_ms
        planner_diagnostics["rejoinRouteIndex"] = rejoin_index
        planner_diagnostics["routeObstructed"] = True
        planner_message = String()
        planner_message.data = json.dumps(
            planner_diagnostics, ensure_ascii=False, separators=(",", ":")
        )
        self.planner_diagnostics_publisher.publish(planner_message)
        if not detour:
            return False
        detour_in_map = [
            self._transform_xy(point[0], point[1], "map", self.local_costmap_frame)
            for point in detour
        ]
        if any(point is None for point in detour_in_map):
            return False
        # RPP receives only current -> rejoin. It must never inherit the
        # remaining recorded route; successful rejoin explicitly returns
        # ownership to MPPI in _result_callback.
        path = self._nav_path_from_points(detour_in_map)
        if path is None:
            return False
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "DetourPath"
        goal.goal_checker_id = "route_goal_checker"
        self.goal_request_pending = True
        self.active_controller = ControllerMode.DETOUR_RPP
        self.detour_attempt_count += 1
        self.last_rejoin_index = rejoin_index
        self.runtime_state = "DETOURING"
        self.runtime_reason = "LOCAL_DETOUR_TO_ROUTE_REJOIN"
        try:
            future = self.action_client.send_goal_async(
                goal, feedback_callback=self._feedback_callback
            )
        except Exception:
            self.goal_request_pending = False
            return False
        future.add_done_callback(self._replan_goal_response_callback)
        return True

    def _replan_goal_response_callback(self, future) -> None:
        self.goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            goal_handle = None
            self.runtime_reason = "LOCAL_DETOUR_TRANSPORT: %s" % exc
        if goal_handle is None or not goal_handle.accepted:
            self.runtime_state = "FAULT"
            self.last_failure_class = "CONTROLLER_FAILED"
            if not self.runtime_reason.startswith("LOCAL_DETOUR_TRANSPORT"):
                self.runtime_reason = "LOCAL_DETOUR_REJECTED"
            self._finish_active_start_attempt("FAILED", self.runtime_reason)
            return
        self.goal_handle = goal_handle
        if self.stop_requested:
            self.runtime_state = "STOPPING"
            self.runtime_reason = "STOP_REQUESTED"
            goal_handle.cancel_goal_async()
            goal_handle.get_result_async().add_done_callback(self._result_callback)
            return
        self.runtime_state = "PATROLLING"
        self.runtime_reason = "LOCAL_DETOUR_ACCEPTED"
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
            self.runtime_state = "FAULT"
            self.runtime_reason = "NAV2_CONTINUATION_TRANSPORT: %s" % exc
            self._finish_active_start_attempt("FAILED", self.runtime_reason)
            return False
        future.add_done_callback(
            lambda completed, accepted_reason=reason: (
                self._continuation_goal_response_callback(
                    completed, accepted_reason
                )
            )
        )
        return True

    def _continuation_goal_response_callback(self, future, accepted_reason: str) -> None:
        self.goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            goal_handle = None
            self.runtime_reason = "NAV2_CONTINUATION_TRANSPORT: %s" % exc
        if goal_handle is None or not goal_handle.accepted:
            self.runtime_state = "FAULT"
            self.last_failure_class = "CONTROLLER_FAILED"
            if not self.runtime_reason.startswith("NAV2_CONTINUATION_TRANSPORT"):
                self.runtime_reason = "NAV2_CONTINUATION_REJECTED"
            self._finish_active_start_attempt("FAILED", self.runtime_reason)
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
        self.motion_evidence.record_command(
            time.monotonic(),
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
        trace = self._runtime_trace_gate()
        if not trace.ready:
            return trace
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
        costmap = self._costmap_health()
        if not costmap.healthy:
            return PatrolReadiness(False, "FAULT", costmap.reason)
        return readiness

    def _start_readiness(self) -> PatrolReadiness:
        if not self.runtime_binding_valid:
            return PatrolReadiness(False, "FAULT", self.runtime_binding_reason)
        trace = self._runtime_trace_gate()
        if not trace.ready:
            return trace
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
        self.localization_gate_failed_at = None
        self.localization_recovered_at = None
        self.detour_attempt_count = 0
        self.last_detour_compute_ms = None
        self.last_rejoin_index = None
        self.active_controller = ControllerMode.MPPI
        self.mppi_retry_count = 0
        self.last_failure_class = None
        self.last_route_obstructed = False
        self.obstruction_evidence.reset()
        self.last_obstruction_costmap_sequence = self.costmap_sequence
        self._reset_costmap_refresh()
        self.motion_evidence = MotionEvidenceTracker(
            retention_s=max(12.0, self.progress_timeout_s + 2.0)
        )

        now = self.get_clock().now().to_msg()
        self.path.header.stamp = now
        for pose in self.path.poses:
            pose.header.stamp = now
        goal = FollowPath.Goal()
        goal.path = self.path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "route_goal_checker"
        attempt.mark_goal_requested()
        self.active_start_attempt = attempt
        self.last_start_attempt = attempt
        self.goal_request_pending = True
        self.stop_requested = False
        self.runtime_state = "STARTING"
        self.runtime_reason = "NAV2_GOAL_REQUESTED"
        self.patrol_started_at = time.monotonic()
        try:
            future = self.action_client.send_goal_async(
                goal,
                feedback_callback=self._feedback_callback,
            )
        except Exception as exc:
            self.goal_request_pending = False
            self.runtime_state = "FAULT"
            self.runtime_reason = "NAV2_GOAL_TRANSPORT: %s" % exc
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
        future.add_done_callback(
            lambda completed, attempt_id=attempt.attempt_id: (
                self._goal_response_callback(completed, attempt_id)
            )
        )
        attempt.mark_service_response(
            success=True,
            reason="NAV2_GOAL_REQUESTED",
        )
        response.success = True
        response.message = self._response_json(
            "NAV2_GOAL_REQUESTED",
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
            self.runtime_state = "FAULT"
            self.runtime_reason = "NAV2_GOAL_TRANSPORT: %s" % exc
            attempt.mark_goal_decision(False, reason=self.runtime_reason)
            self.last_start_attempt = attempt
            self.active_start_attempt = None
            return
        if not goal_handle.accepted:
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
        if self.pose is not None:
            self.route_progress_index = max(
                self.route_progress_index,
                self._nearest_path_index(self.pose[0], self.pose[1]),
            )

    def _result_callback(self, future) -> None:
        completed_controller = self.active_controller
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:
            status = None
            self.runtime_reason = "NAV2_RESULT_TRANSPORT: %s" % exc
        self.goal_handle = None
        self.goal_request_pending = False
        if self.gate_cancel_reason:
            cancel_reason = self.gate_cancel_reason
            self.runtime_state = (
                "HOLDING"
                if cancel_reason in RECOVERABLE_LOCALIZATION_GATES
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
            self.runtime_state = "COMPLETED"
            self.runtime_reason = "ROUTE_COMPLETE"
            attempt_outcome = "COMPLETED"
        else:
            gate = self._runtime_gate()
            if not gate.ready and gate.reason not in RECOVERABLE_LOCALIZATION_GATES:
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
                        or gate.reason not in RECOVERABLE_LOCALIZATION_GATES
                    ),
                    costmap_healthy=costmap_health.healthy,
                    costmap_reason=costmap_health.reason,
                    route_obstructed=route_obstructed,
                    motion=self.motion_evidence.snapshot(
                        time.monotonic(), window_s=self.progress_timeout_s
                    ),
                    mppi_retry_count=self.mppi_retry_count,
                    mppi_retry_limit=self.mppi_retry_limit,
                )
            )
            self.last_failure_class = decision.failure_class.value
            if decision.action == RecoveryAction.HOLD_LOCALIZATION:
                self.resume_pending = True
                self.runtime_state = "HOLDING"
                self.runtime_reason = gate.reason
                return
            if decision.action == RecoveryAction.RETRY_MPPI:
                self.mppi_retry_count += 1
                self._request_mppi_suffix(decision.reason)
                return
            if decision.action == RecoveryAction.START_DETOUR:
                if self._try_local_replan():
                    return
                self.runtime_state = "BLOCKED"
                self.runtime_reason = "PATH_OBSTRUCTED"
                attempt_outcome = "FAILED"
            elif decision.action == RecoveryAction.STOP_BLOCKED:
                self.runtime_state = "BLOCKED"
                self.runtime_reason = decision.reason
                attempt_outcome = "FAILED"
            else:
                self.runtime_state = "FAULT"
                self.runtime_reason = decision.reason
                attempt_outcome = "FAILED"
        self._finish_active_start_attempt(
            attempt_outcome,
            self.runtime_reason,
        )

    def _stop_callback(self, request, response):
        del request
        self.stop_requested = True
        self.resume_pending = False
        self._reset_costmap_refresh()
        self.runtime_state = "STOPPING"
        self.runtime_reason = "STOP_REQUESTED"
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
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
            self.resume_pending = reason in RECOVERABLE_LOCALIZATION_GATES
            if self.resume_pending:
                self._reset_costmap_refresh()
        if self.goal_handle is not None:
            self.stop_requested = False
            self.goal_handle.cancel_goal_async()

    def _request_resume(self) -> None:
        if self.goal_handle is not None or self.goal_request_pending:
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
            self.resume_pending = False
            self.runtime_state = "FAULT"
            self.runtime_reason = "NAV2_RESUME_TRANSPORT: %s" % exc
            self._finish_active_start_attempt("FAILED", self.runtime_reason)
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
            self.resume_pending = False
            self.runtime_state = "FAULT"
            if not self.runtime_reason.startswith("NAV2_RESUME_TRANSPORT"):
                self.runtime_reason = "NAV2_RESUME_REJECTED"
            self._finish_active_start_attempt("FAILED", self.runtime_reason)
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
        accepted_goal = self.goal_handle is not None
        motion_authorized = accepted_goal and gate.ready and self.runtime_state == "PATROLLING"
        if (accepted_goal or self.goal_request_pending) and not gate.ready:
            if gate.reason in RECOVERABLE_LOCALIZATION_GATES:
                if self.localization_gate_failed_at is None:
                    self.localization_gate_failed_at = time.monotonic()
                elapsed = time.monotonic() - self.localization_gate_failed_at
                self.runtime_state = "HOLDING"
                self.runtime_reason = gate.reason
                if elapsed >= self.localization_dropout_grace_s:
                    self._cancel_for_gate(gate.reason)
            else:
                self._cancel_for_gate(gate.reason)
            motion_authorized = False
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
                        self.resume_pending = False
                        self.runtime_state = "FAULT"
                        self.runtime_reason = refresh.reason
                        self._finish_active_start_attempt("FAILED", refresh.reason)
                    else:
                        self.runtime_state = refresh.state
                        self.runtime_reason = refresh.reason
            elif gate.reason not in RECOVERABLE_LOCALIZATION_GATES:
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
        }:
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
                    "RPP_LOCAL_DETOUR_TO_REJOIN"
                    if self.active_controller == ControllerMode.DETOUR_RPP
                    else "MPPI_RECORDED_ROUTE"
                )
            ),
            "failureClass": self.last_failure_class,
            "mppiRetryCount": self.mppi_retry_count,
            "mppiRetryLimit": self.mppi_retry_limit,
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
