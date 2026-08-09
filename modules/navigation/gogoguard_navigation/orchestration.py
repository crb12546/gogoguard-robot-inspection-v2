"""Pure navigation orchestration policy shared by ROS runtime and replay tests.

The policy classifies evidence; it never publishes velocity commands and has
no ROS dependency.  This keeps controller switching testable with recorded
field incidents instead of encoding it in callback branches.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Deque, Iterable, Optional, Protocol, Sequence


class ControllerMode(str, Enum):
    MPPI = "MPPI"
    DETOUR_RPP = "DETOUR_RPP"


class FailureClass(str, Enum):
    TRANSIENT_CONTROL = "TRANSIENT_CONTROL"
    LOCALIZATION_LOST = "LOCALIZATION_LOST"
    COSTMAP_UNHEALTHY = "COSTMAP_UNHEALTHY"
    PATH_OBSTRUCTED = "PATH_OBSTRUCTED"
    CONTROLLER_FAILED = "CONTROLLER_FAILED"
    ACTUATION_STALL = "ACTUATION_STALL"


class RecoveryAction(str, Enum):
    HOLD_LOCALIZATION = "HOLD_LOCALIZATION"
    WAIT_COSTMAP = "WAIT_COSTMAP"
    RETRY_MPPI = "RETRY_MPPI"
    START_DETOUR = "START_DETOUR"
    SEARCH_PATH = "SEARCH_PATH"
    RETRY_ACTUATION = "RETRY_ACTUATION"


class ControllerSuccessAction(str, Enum):
    COMPLETE_ROUTE = "COMPLETE_ROUTE"
    RESUME_MPPI_SUFFIX = "RESUME_MPPI_SUFFIX"


@dataclass(frozen=True)
class MotionSnapshot:
    window_s: float
    observed_duration_s: Optional[float]
    translation_m: Optional[float]
    rotation_rad: Optional[float]
    mean_linear_command_mps: Optional[float]
    mean_angular_command_rps: Optional[float]
    linear_command_active_ratio: Optional[float] = None
    angular_command_active_ratio: Optional[float] = None

    def has_actuation_stall(
        self,
        *,
        minimum_linear_command_mps: float = 0.20,
        minimum_angular_command_rps: float = 0.15,
        required_translation_m: float = 0.15,
        required_rotation_rad: float = 0.15,
    ) -> bool:
        """Return true only when commands exist and neither commanded axis moved."""

        # The configured window is a minimum observation period, not a label
        # that can be attached to a sub-second sample. This prevents an early
        # controller abort from being misreported as a hardware gait stall.
        if (
            self.observed_duration_s is None
            or self.observed_duration_s + 0.05 < self.window_s
        ):
            return False

        commanded_linear = bool(
            self.mean_linear_command_mps is not None
            and self.mean_linear_command_mps >= minimum_linear_command_mps
            and (
                self.linear_command_active_ratio is None
                or self.linear_command_active_ratio >= 0.5
            )
        )
        commanded_angular = bool(
            self.mean_angular_command_rps is not None
            and self.mean_angular_command_rps >= minimum_angular_command_rps
            and (
                self.angular_command_active_ratio is None
                or self.angular_command_active_ratio >= 0.5
            )
        )
        if not commanded_linear and not commanded_angular:
            return False
        linear_progress = bool(
            commanded_linear
            and self.translation_m is not None
            and self.translation_m >= required_translation_m
        )
        angular_progress = bool(
            commanded_angular
            and self.rotation_rad is not None
            and self.rotation_rad >= required_rotation_rad
        )
        return not linear_progress and not angular_progress


@dataclass(frozen=True)
class FailureEvidence:
    controller: ControllerMode
    localization_usable: bool
    costmap_healthy: bool
    costmap_reason: str
    route_obstructed: bool
    motion: MotionSnapshot


@dataclass(frozen=True)
class FailureDecision:
    failure_class: FailureClass
    action: RecoveryAction
    reason: str


def decide_controller_failure(evidence: FailureEvidence) -> FailureDecision:
    """Classify a controller abort before selecting a recovery action.

    A clear costmap can never authorize a detour. A failed search or controller
    attempt is recoverable: counters remain diagnostics and never terminate an
    attended patrol. Humble's FollowPath result carries no detailed error code,
    so a fresh costmap and rate-limited retry provide the next evidence sample.
    """

    if not evidence.localization_usable:
        return FailureDecision(
            FailureClass.LOCALIZATION_LOST,
            RecoveryAction.HOLD_LOCALIZATION,
            "LOCALIZATION_LOST",
        )
    if not evidence.costmap_healthy:
        return FailureDecision(
            FailureClass.COSTMAP_UNHEALTHY,
            RecoveryAction.WAIT_COSTMAP,
            evidence.costmap_reason or "COSTMAP_UNHEALTHY",
        )
    if evidence.route_obstructed:
        if evidence.controller == ControllerMode.DETOUR_RPP:
            return FailureDecision(
                FailureClass.PATH_OBSTRUCTED,
                RecoveryAction.SEARCH_PATH,
                "SEARCHING_FOR_PATH",
            )
        return FailureDecision(
            FailureClass.PATH_OBSTRUCTED,
            RecoveryAction.START_DETOUR,
            "PATH_OBSTRUCTED",
        )
    if evidence.motion.has_actuation_stall():
        return FailureDecision(
            FailureClass.ACTUATION_STALL,
            RecoveryAction.RETRY_ACTUATION,
            "ACTUATION_RECOVERY",
        )
    if evidence.controller == ControllerMode.DETOUR_RPP:
        return FailureDecision(
            FailureClass.CONTROLLER_FAILED,
            RecoveryAction.SEARCH_PATH,
            "SEARCHING_FOR_PATH",
        )
    return FailureDecision(
        FailureClass.TRANSIENT_CONTROL,
        RecoveryAction.RETRY_MPPI,
        "TRANSIENT_CONTROL_RETRY",
    )


def controller_success_action(
    controller: ControllerMode,
) -> ControllerSuccessAction:
    """Make the controller ownership handoff explicit and replayable."""

    if controller == ControllerMode.DETOUR_RPP:
        return ControllerSuccessAction.RESUME_MPPI_SUFFIX
    return ControllerSuccessAction.COMPLETE_ROUTE


class MotionEvidenceTracker:
    """Bounded command/pose history used to distinguish stall from obstruction."""

    def __init__(self, *, retention_s: float = 12.0) -> None:
        self.retention_s = float(retention_s)
        self._poses: Deque[tuple[float, float, float, float]] = deque()
        self._commands: Deque[tuple[float, float, float]] = deque()

    def _trim(self, now: float) -> None:
        cutoff = float(now) - self.retention_s
        while self._poses and self._poses[0][0] < cutoff:
            self._poses.popleft()
        while self._commands and self._commands[0][0] < cutoff:
            self._commands.popleft()

    def record_pose(self, at: float, x: float, y: float, yaw: float) -> None:
        self._poses.append((float(at), float(x), float(y), float(yaw)))
        self._trim(float(at))

    def record_command(
        self,
        at: float,
        linear_x: float,
        linear_y: float,
        angular_z: float,
    ) -> None:
        self._commands.append(
            (
                float(at),
                math.hypot(float(linear_x), float(linear_y)),
                abs(float(angular_z)),
            )
        )
        self._trim(float(at))

    def snapshot(self, now: float, *, window_s: float) -> MotionSnapshot:
        now = float(now)
        window_s = float(window_s)
        cutoff = now - window_s
        poses = [value for value in self._poses if value[0] >= cutoff]
        commands = [value for value in self._commands if value[0] >= cutoff]
        sample_times = [value[0] for value in poses] + [
            value[0] for value in commands
        ]
        observed_duration = (
            max(0.0, now - min(sample_times)) if sample_times else 0.0
        )
        translation = None
        rotation = None
        if len(poses) >= 2:
            translation = math.hypot(
                poses[-1][1] - poses[0][1],
                poses[-1][2] - poses[0][2],
            )
            delta_yaw = math.atan2(
                math.sin(poses[-1][3] - poses[0][3]),
                math.cos(poses[-1][3] - poses[0][3]),
            )
            rotation = abs(delta_yaw)
        mean_linear = None
        mean_angular = None
        linear_active_ratio = None
        angular_active_ratio = None
        if commands:
            active_linear = [value[1] for value in commands if value[1] >= 0.01]
            active_angular = [value[2] for value in commands if value[2] >= 0.01]
            linear_active_ratio = len(active_linear) / len(commands)
            angular_active_ratio = len(active_angular) / len(commands)
            if active_linear:
                mean_linear = sum(active_linear) / len(active_linear)
            if active_angular:
                mean_angular = sum(active_angular) / len(active_angular)
        return MotionSnapshot(
            window_s=window_s,
            observed_duration_s=observed_duration,
            translation_m=translation,
            rotation_rad=rotation,
            mean_linear_command_mps=mean_linear,
            mean_angular_command_rps=mean_angular,
            linear_command_active_ratio=linear_active_ratio,
            angular_command_active_ratio=angular_active_ratio,
        )


class CostGrid(Protocol):
    resolution: float

    def cell(self, x: float, y: float) -> tuple[int, int]: ...

    def inside(self, cell: tuple[int, int]) -> bool: ...

    def cost(self, cell: tuple[int, int]) -> int: ...


@dataclass(frozen=True)
class CostmapHealth:
    healthy: bool
    reason: str
    frame_id: str
    age_s: Optional[float]
    robot_cost: Optional[int]


def evaluate_costmap_health(
    grid: Optional[CostGrid],
    *,
    frame_id: str,
    expected_frame: str,
    age_s: Optional[float],
    robot_xy: Optional[Sequence[float]],
    timeout_s: float = 1.0,
    lethal_cost: int = 253,
) -> CostmapHealth:
    """Validate the 2D navigation observation before it may own motion."""

    if grid is None:
        return CostmapHealth(False, "COSTMAP_MISSING", frame_id, age_s, None)
    if frame_id != expected_frame:
        return CostmapHealth(False, "COSTMAP_FRAME_INVALID", frame_id, age_s, None)
    if age_s is None or not math.isfinite(age_s) or age_s > timeout_s:
        return CostmapHealth(False, "COSTMAP_STALE", frame_id, age_s, None)
    if robot_xy is None or len(robot_xy) < 2:
        return CostmapHealth(False, "COSTMAP_ROBOT_POSE_MISSING", frame_id, age_s, None)
    cell = grid.cell(float(robot_xy[0]), float(robot_xy[1]))
    if not grid.inside(cell):
        return CostmapHealth(False, "COSTMAP_ROBOT_OUTSIDE", frame_id, age_s, None)
    robot_cost = int(grid.cost(cell))
    if robot_cost >= int(lethal_cost):
        return CostmapHealth(
            False, "COSTMAP_ROBOT_OCCUPIED", frame_id, age_s, robot_cost
        )
    return CostmapHealth(True, "OK", frame_id, age_s, robot_cost)


class ObstructionEvidenceTracker:
    """Require obstruction to persist in healthy, fresh costmap frames."""

    def __init__(self, *, confirmation_s: float = 0.30) -> None:
        if confirmation_s <= 0.0:
            raise ValueError("confirmation_s must be positive")
        self.confirmation_s = float(confirmation_s)
        self._blocked_since: Optional[float] = None

    def reset(self) -> None:
        self._blocked_since = None

    def update(self, now: float, *, blocked: bool, source_healthy: bool) -> bool:
        now = float(now)
        if not source_healthy or not blocked:
            self.reset()
            return False
        if self._blocked_since is None:
            self._blocked_since = now
            return False
        return now - self._blocked_since >= self.confirmation_s


def _segment_samples(
    first: Sequence[float], second: Sequence[float], spacing: float
) -> Iterable[tuple[float, float]]:
    distance = math.hypot(
        float(second[0]) - float(first[0]),
        float(second[1]) - float(first[1]),
    )
    count = max(1, int(math.ceil(distance / max(0.01, spacing))))
    for index in range(count + 1):
        ratio = index / count
        yield (
            float(first[0]) + (float(second[0]) - float(first[0])) * ratio,
            float(first[1]) + (float(second[1]) - float(first[1])) * ratio,
        )


def route_obstruction_evidence(
    grid: CostGrid,
    route_xy: Iterable[Sequence[float]],
    *,
    occupied_threshold: int = 65,
    minimum_consecutive_samples: int = 2,
) -> bool:
    """Prove that the recorded route centreline is blocked in the local grid.

    The grid is already footprint-inflated, so centreline samples are enough.
    Unknown or out-of-window cells are not obstacle evidence.
    """

    points = list(route_xy)
    if len(points) < 2 or grid.resolution <= 0.0:
        return False
    consecutive = 0
    previous_cell = None
    for first, second in zip(points, points[1:]):
        for x, y in _segment_samples(first, second, grid.resolution * 0.5):
            cell = grid.cell(x, y)
            if cell == previous_cell:
                continue
            previous_cell = cell
            occupied = bool(
                grid.inside(cell)
                and grid.cost(cell) >= int(occupied_threshold)
            )
            consecutive = consecutive + 1 if occupied else 0
            if consecutive >= int(minimum_consecutive_samples):
                return True
    return False
