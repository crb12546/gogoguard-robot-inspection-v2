"""Pure contracts for the fixed-map patrol runtime.

This module deliberately has no ROS imports.  It is used by the on-robot node,
launch-time map resolution, and offline tests so all three make the same safety
decision.
"""

from __future__ import annotations

import json
import hashlib
import math
import stat
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Deque, Mapping, Optional, Sequence, Tuple

from go2_site_ops.map_store import MapVersionStore
from go2_site_ops.localization_quality import (
    LocalizationQualityProfile,
    resolve_localization_quality_profile,
)
from go2_site_ops.release_calibration import ReleaseCalibrationBundle
from go2_site_ops.route_contract import (
    Route,
    RouteError,
    angle_distance,
    finite as _finite,
    load_route,
    normalize_angle,
    sample_route as _sample_route,
)


# Kept as a public compatibility export for the runtime manager and offline
# contract tests.  The implementation remains owned by go2_site_ops.
sample_route = _sample_route


class RuntimeContractError(ValueError):
    """Raised when the activated map is not a complete runtime bundle."""


@dataclass(frozen=True)
class RuntimeArtifactGuard:
    """Cheap post-hash identity checks for a warm runtime.

    Candidate files are fully hashed before this guard is created.  Re-hashing
    a large PCD every second would steal CPU and storage bandwidth from
    localization.  Device/inode/size/mtime/ctime/mode plus an 8 KiB sampled
    content signature fail closed on replacement, ordinary edits, chmod,
    deletion, symlink substitution, and same-size overwrites on filesystems
    whose timestamp resolution is too coarse.  A slower full-map audit remains
    responsible for detecting an adversarial middle-only large-file edit.
    """

    fingerprints: Mapping[str, Tuple[Path, Tuple[Any, ...]]]

    @staticmethod
    def _sample_signature(path: Path, size: int) -> str:
        digest = hashlib.sha256()
        digest.update(str(size).encode("ascii"))
        try:
            with path.open("rb") as handle:
                if size <= 8192:
                    digest.update(handle.read())
                else:
                    digest.update(handle.read(4096))
                    handle.seek(-4096, 2)
                    digest.update(handle.read(4096))
        except OSError as exc:
            raise RuntimeContractError("RUNTIME_ARTIFACT_UNAVAILABLE: %s" % exc)
        return digest.hexdigest()

    @staticmethod
    def _fingerprint(path: Path) -> Tuple[Any, ...]:
        path = Path(path)
        if path.is_symlink():
            raise RuntimeContractError("RUNTIME_ARTIFACT_SYMLINK")
        try:
            record = path.stat()
        except OSError as exc:
            raise RuntimeContractError("RUNTIME_ARTIFACT_UNAVAILABLE: %s" % exc)
        if not stat.S_ISREG(record.st_mode):
            raise RuntimeContractError("RUNTIME_ARTIFACT_NOT_REGULAR")
        return (
            int(record.st_dev),
            int(record.st_ino),
            int(record.st_size),
            int(record.st_mtime_ns),
            int(record.st_ctime_ns),
            int(stat.S_IMODE(record.st_mode)),
            RuntimeArtifactGuard._sample_signature(path, int(record.st_size)),
        )

    @classmethod
    def capture(cls, artifacts: Mapping[str, Path]) -> "RuntimeArtifactGuard":
        if not artifacts:
            raise RuntimeContractError("RUNTIME_ARTIFACTS_REQUIRED")
        captured = {}
        for name, path in sorted(artifacts.items()):
            label = str(name).strip()
            if not label:
                raise RuntimeContractError("RUNTIME_ARTIFACT_NAME_REQUIRED")
            absolute = Path(path).absolute()
            captured[label] = (absolute, cls._fingerprint(absolute))
        return cls(fingerprints=captured)

    def verify(self) -> Tuple[bool, str]:
        for name, (path, expected) in self.fingerprints.items():
            try:
                actual = self._fingerprint(path)
            except RuntimeContractError as exc:
                return False, "%s_%s" % (name.upper(), exc)
            if actual != expected:
                return False, "%s_RUNTIME_ARTIFACT_CHANGED" % name.upper()
        return True, "OK"


@dataclass(frozen=True)
class PatrolSettings:
    loop_mode: str
    speed_limit_mps: float
    start_max_distance_m: float
    start_max_yaw_deg: float
    path_sample_spacing_m: float


@dataclass(frozen=True)
class RuntimeBundle:
    site_id: str
    version_id: str
    manifest_hash: str
    route_path: Path
    localization_map_path: Path
    runtime_profile_path: Path
    route: Route
    patrol: PatrolSettings
    initialization_center: Tuple[float, float, float]
    initialization_radius_m: float
    initialization_yaw_rad: float
    initialization_yaw_tolerance_deg: float
    localization_quality: LocalizationQualityProfile
    calibration_bundle_path: Optional[Path] = None
    calibration_hash: str = ""
    robot_id: str = ""
    sensor_id: str = ""


@dataclass(frozen=True)
class PatrolReadiness:
    ready: bool
    state: str
    reason: str
    start_distance_m: Optional[float] = None
    start_yaw_error_deg: Optional[float] = None


def evaluate_runtime_trace_gate(
    status: Optional[Mapping[str, Any]],
    *,
    receive_age_s: float,
    timeout_s: float,
    expected_map_version: str,
    expected_manifest_hash: str,
) -> PatrolReadiness:
    """Require a fresh, writable black-box recorder before authorizing motion."""

    if not isinstance(status, Mapping):
        return PatrolReadiness(False, "DEGRADED", "RUNTIME_TRACE_MISSING")
    try:
        age = float(receive_age_s)
        timeout = float(timeout_s)
    except (TypeError, ValueError):
        age = float("inf")
        timeout = 0.0
    if not math.isfinite(age) or age > max(0.0, timeout):
        return PatrolReadiness(False, "DEGRADED", "RUNTIME_TRACE_STALE")
    binding = status.get("binding")
    if (
        status.get("schema") != "go2.runtime_trace_status.v1"
        or not isinstance(binding, Mapping)
        or not isinstance(status.get("recordsWritten"), int)
        or isinstance(status.get("recordsWritten"), bool)
        or status.get("recordsWritten", 0) <= 0
        or not isinstance(status.get("lastFsyncAt"), (int, float))
        or isinstance(status.get("lastFsyncAt"), bool)
        or not math.isfinite(float(status.get("lastFsyncAt")))
    ):
        return PatrolReadiness(False, "FAULT", "RUNTIME_TRACE_INVALID")
    try:
        fsync_age = float(status.get("lastFsyncAgeS"))
    except (TypeError, ValueError):
        fsync_age = float("inf")
    if not math.isfinite(fsync_age) or fsync_age > 2.0:
        return PatrolReadiness(
            False, "DEGRADED", "RUNTIME_TRACE_NOT_PERSISTING"
        )
    if (
        binding.get("mapVersion") != expected_map_version
        or binding.get("manifestHash") != expected_manifest_hash
    ):
        return PatrolReadiness(
            False, "FAULT", "RUNTIME_TRACE_BINDING_MISMATCH"
        )
    if status.get("writable") is not True:
        return PatrolReadiness(False, "DEGRADED", "RUNTIME_TRACE_UNWRITABLE")
    return PatrolReadiness(True, "READY", "RUNTIME_TRACE_READY")


@dataclass(frozen=True)
class FastLioHealthAssessment:
    """Current, precomputed health of the warm FAST-LIO odometry source."""

    ready: bool
    reason: str
    stationary: bool
    latched_ready: bool
    odom_age_s: Optional[float]
    window_s: float
    sample_count: int
    translation_excursion_m: Optional[float]
    rotation_excursion_deg: Optional[float]

    def as_dict(self) -> dict:
        return {
            "schema": "go2.fastlio_health.v1",
            "ready": self.ready,
            "reason": self.reason,
            "stationary": self.stationary,
            "latchedReady": self.latched_ready,
            "odomAgeS": self.odom_age_s,
            "windowS": self.window_s,
            "sampleCount": self.sample_count,
            "translationExcursionM": self.translation_excursion_m,
            "rotationExcursionDeg": self.rotation_excursion_deg,
        }


@dataclass(frozen=True)
class _FastLioPoseSample:
    received_at: float
    sensor_stamp_s: float
    position: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]


_FASTLIO_RECOVERY_REASONS = frozenset(
    {
        "FASTLIO_ODOMETRY_INVALID",
        "FASTLIO_TIME_RESET",
        "FASTLIO_FRAME_GAP",
        "FASTLIO_STATIONARY_DRIFT",
    }
)


class FastLioHealthTracker:
    """Latch FAST-LIO ready only after a stationary convergence window.

    The tracker is deliberately independent of ROS. The runtime feeds it the
    already-warm raw odometry and Unitree stationary state. ``startPatrol``
    only reads :meth:`assess`; it never waits for this window or restarts the
    sensor stack.
    """

    def __init__(
        self,
        *,
        settle_window_s: float = 5.0,
        maximum_translation_excursion_m: float = 0.05,
        maximum_rotation_excursion_deg: float = 1.0,
        odom_timeout_s: float = 0.5,
        maximum_frame_gap_s: float = 0.5,
        minimum_samples: int = 30,
    ) -> None:
        values = (
            settle_window_s,
            maximum_translation_excursion_m,
            maximum_rotation_excursion_deg,
            odom_timeout_s,
            maximum_frame_gap_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("FAST-LIO health thresholds must be positive")
        if not 2.0 <= settle_window_s <= 30.0:
            raise ValueError("FAST-LIO settle window must be 2..30 seconds")
        if not 5 <= int(minimum_samples) <= 1000:
            raise ValueError("FAST-LIO minimum samples must be 5..1000")
        self.settle_window_s = float(settle_window_s)
        self.maximum_translation_excursion_m = float(
            maximum_translation_excursion_m
        )
        self.maximum_rotation_excursion_deg = float(
            maximum_rotation_excursion_deg
        )
        self.odom_timeout_s = float(odom_timeout_s)
        self.maximum_frame_gap_s = float(maximum_frame_gap_s)
        self.minimum_samples = int(minimum_samples)
        self.samples: Deque[_FastLioPoseSample] = deque()
        self.stationary = False
        self.latched_ready = False
        self.last_odom_received_at: Optional[float] = None
        self.last_sensor_stamp_s: Optional[float] = None
        self.last_reason = "FASTLIO_ODOMETRY_MISSING"
        self.translation_excursion_m: Optional[float] = None
        self.rotation_excursion_deg: Optional[float] = None

    @staticmethod
    def _normalize_quaternion(
        values: Sequence[float],
    ) -> Tuple[float, float, float, float]:
        if len(values) != 4:
            raise ValueError("FAST-LIO quaternion must have four values")
        quaternion = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in quaternion):
            raise ValueError("FAST-LIO quaternion is not finite")
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1.0e-6:
            raise ValueError("FAST-LIO quaternion has zero norm")
        return tuple(value / norm for value in quaternion)

    @staticmethod
    def _translation_distance(
        first: Sequence[float], last: Sequence[float]
    ) -> float:
        return math.sqrt(
            sum((float(last[index]) - float(first[index])) ** 2 for index in range(3))
        )

    @staticmethod
    def _rotation_distance_deg(
        first: Sequence[float], last: Sequence[float]
    ) -> float:
        dot = abs(sum(left * right for left, right in zip(first, last)))
        dot = max(-1.0, min(1.0, dot))
        return math.degrees(2.0 * math.acos(dot))

    def set_stationary(self, stationary: bool) -> None:
        stationary = bool(stationary)
        if stationary != self.stationary:
            self.samples.clear()
            self.translation_excursion_m = None
            self.rotation_excursion_deg = None
        self.stationary = stationary
        if not stationary and not self.latched_ready:
            self.last_reason = "FASTLIO_WAITING_FOR_STATIONARY"

    def _invalidate(self, reason: str) -> None:
        self.samples.clear()
        self.latched_ready = False
        self.translation_excursion_m = None
        self.rotation_excursion_deg = None
        self.last_reason = reason

    def update_odom(
        self,
        *,
        received_at: float,
        sensor_stamp_s: float,
        position: Sequence[float],
        quaternion_xyzw: Sequence[float],
    ) -> None:
        try:
            now = float(received_at)
            stamp = float(sensor_stamp_s)
            pose = tuple(float(value) for value in position)
            if len(pose) != 3 or not all(math.isfinite(value) for value in pose):
                raise ValueError("FAST-LIO position is invalid")
            if not math.isfinite(now) or not math.isfinite(stamp) or stamp <= 0.0:
                raise ValueError("FAST-LIO time is invalid")
            quaternion = self._normalize_quaternion(quaternion_xyzw)
        except (TypeError, ValueError):
            self._invalidate("FASTLIO_ODOMETRY_INVALID")
            return

        if self.last_sensor_stamp_s is not None:
            sensor_gap = stamp - self.last_sensor_stamp_s
            receive_gap = (
                now - self.last_odom_received_at
                if self.last_odom_received_at is not None
                else 0.0
            )
            if sensor_gap <= 0.0:
                self._invalidate("FASTLIO_TIME_RESET")
            elif (
                sensor_gap > self.maximum_frame_gap_s
                or receive_gap > self.maximum_frame_gap_s
            ):
                self._invalidate("FASTLIO_FRAME_GAP")
        self.last_sensor_stamp_s = stamp
        self.last_odom_received_at = now

        if not self.stationary:
            return
        sample = _FastLioPoseSample(now, stamp, pose, quaternion)
        self.samples.append(sample)
        while (
            len(self.samples) >= 2
            and now - self.samples[1].received_at >= self.settle_window_s
        ):
            self.samples.popleft()

        first = self.samples[0]
        window_s = now - first.received_at
        self.translation_excursion_m = max(
            self._translation_distance(first.position, item.position)
            for item in self.samples
        )
        self.rotation_excursion_deg = max(
            self._rotation_distance_deg(
                first.quaternion_xyzw, item.quaternion_xyzw
            )
            for item in self.samples
        )
        exceeds_limit = (
            self.translation_excursion_m
            > self.maximum_translation_excursion_m
            or self.rotation_excursion_deg
            > self.maximum_rotation_excursion_deg
        )
        if self.latched_ready and exceeds_limit:
            # A previously healthy runtime must fail closed as soon as a
            # stationary jump is observable; it must not retain authorization
            # for the remainder of a newly-started five-second window.
            self.latched_ready = False
            self.last_reason = "FASTLIO_STATIONARY_DRIFT"
            return
        if window_s < self.settle_window_s or len(self.samples) < self.minimum_samples:
            if (
                not self.latched_ready
                and self.last_reason not in _FASTLIO_RECOVERY_REASONS
            ):
                self.last_reason = "FASTLIO_SETTLING"
            return
        if not exceeds_limit:
            self.latched_ready = True
            self.last_reason = "OK"
        else:
            self.latched_ready = False
            self.last_reason = "FASTLIO_STATIONARY_DRIFT"

    def assess(self, now: Optional[float] = None) -> FastLioHealthAssessment:
        current = time.monotonic() if now is None else float(now)
        age = (
            None
            if self.last_odom_received_at is None
            else max(0.0, current - self.last_odom_received_at)
        )
        window_s = (
            max(0.0, self.samples[-1].received_at - self.samples[0].received_at)
            if len(self.samples) >= 2
            else 0.0
        )
        if age is None:
            reason = "FASTLIO_ODOMETRY_MISSING"
            ready = False
        elif age > self.odom_timeout_s:
            reason = "FASTLIO_ODOMETRY_STALE"
            ready = False
        elif not self.latched_ready:
            reason = self.last_reason
            ready = False
        else:
            reason = "OK"
            ready = True
        return FastLioHealthAssessment(
            ready=ready,
            reason=reason,
            stationary=self.stationary,
            latched_ready=self.latched_ready,
            odom_age_s=age,
            window_s=window_s,
            sample_count=len(self.samples),
            translation_excursion_m=self.translation_excursion_m,
            rotation_excursion_deg=self.rotation_excursion_deg,
        )


_OPERATOR_REASON_MESSAGES = {
    "OK": "机器狗已经就绪",
    "NAV2_GOAL_REQUESTED": "开始请求已收到，正在等待导航系统确认路线；机器狗尚未获准行走",
    "REQUESTED_MAP_VERSION_IS_NOT_ACTIVE": "平台指定的地图不是机器狗当前已发布地图，本次未开始；请刷新任务或联系工程人员",
    "REQUESTED_ROUTE_IS_NOT_ACTIVE": "平台指定的路线不属于机器狗当前已发布地图，本次未开始；请刷新任务或联系工程人员",
    "ALREADY_PATROLLING": "机器狗已经在巡检中，请不要重复开始",
    "STOP_REQUESTED": "停止指令已接受，机器狗正在停车",
    "STOPPED": "机器狗已停止巡检",
    "ROUTE_COMPLETE": "本次巡检路线已经完成",
    "LOCALIZATION_RECOVERED_RESUMING_ROUTE": "定位已恢复，正从当前进度继续巡检",
    "LOCALIZATION_RECOVERED_ROUTE_RESUMED": "已从当前进度继续巡检",
    "TRANSIENT_CONTROL_RETRY": "导航控制刚才短暂中断，路线前方没有确认障碍，正从当前进度自动重试",
    "PATH_OBSTRUCTED": "局部代价地图已确认原路线前方被占用，正在局部绕行或已停止等待处理",
    "ACTUATION_STALL": "系统持续发出了有效运动指令，但机器狗没有足够位移或转动；已停止，请查看运动桥和机器狗状态",
    "DETOUR_CONTROLLER_FAILED": "局部绕行路径已生成，但绕行控制器没有完成到重入点；机器狗已停止",
    "MPPI_RETRY_EXHAUSTED": "原路线前方没有确认障碍，但 MPPI 多次未能继续；机器狗已停止并保留了诊断记录",
    "DETOUR_REJOINED_RESUMING_ROUTE": "已完成局部绕行并回到原路线，正交还 MPPI 继续巡检",
    "LOCAL_DETOUR_TO_ROUTE_REJOIN": "原路线被阻挡，正在规划局部绕行并接回前方路线",
    "LOCAL_DETOUR_ACCEPTED": "已找到局部绕行路径，正在接回原巡检路线",
    "LOCAL_DETOUR_REJECTED": "局部绕行路径未被导航接受，机器狗已停止",
    "LOCALIZATION_STATUS_MISSING": "正在等待地图定位，请保持机器狗不动并稍候",
    "LOCALIZATION_STATUS_STALE": "地图定位数据暂时中断，系统已禁止行走；请保持机器狗不动并稍候",
    "LOCALIZATION_STATUS_SCHEMA": "地图定位程序版本不匹配，系统已禁止行走；请联系工程人员",
    "LOCALIZATION_MAP_VERSION_MISMATCH": "定位使用的地图不是当前发布地图，系统已禁止行走；请联系工程人员",
    "LOCALIZATION_NOT_TRACKING": "机器狗还没有在地图中稳定找到自己，请保持不动并稍候",
    "LOCALIZATION_NOT_USABLE": "当前定位精度不足，系统已禁止行走；请保持机器狗不动并等待恢复",
    "LOCALIZATION_POSE_MISSING": "正在等待机器狗在地图中的位置，请保持不动并稍候",
    "LOCALIZATION_POSE_STALE": "机器狗在地图中的位置没有及时更新，系统已禁止行走",
    "LOCALIZATION_POSE_FRAME": "定位坐标设置错误，系统已禁止行走；请联系工程人员",
    "LOCALIZATION_POSE_NONFINITE": "定位结果异常，系统已禁止行走；请联系工程人员",
    "FASTLIO_ODOMETRY_MISSING": "正在等待雷达里程计，请保持机器狗不动并稍候",
    "FASTLIO_ODOMETRY_STALE": "雷达里程计数据已中断，系统已禁止行走；请检查雷达连接",
    "FASTLIO_ODOMETRY_INVALID": "雷达里程计数据异常，系统已禁止行走；请联系工程人员",
    "FASTLIO_TIME_RESET": "雷达里程计时间发生回退，系统正在重新建立稳定状态",
    "FASTLIO_FRAME_GAP": "雷达里程计曾发生断流，系统正在重新确认稳定状态",
    "FASTLIO_WAITING_FOR_STATIONARY": "请先让机器狗停稳，系统会在后台确认雷达定位是否稳定",
    "FASTLIO_SETTLING": "雷达定位正在后台稳定，请保持机器狗不动并稍候",
    "FASTLIO_STATIONARY_DRIFT": "机器狗虽然静止，但雷达定位仍在漂移；系统已禁止行走并会自动继续观察",
    "NAV2_FOLLOW_PATH_UNAVAILABLE": "导航服务仍在启动，请稍候再开始巡检",
    "ROBOT_STATE_MISSING": "没有收到机器狗实时状态，请检查狗身主机与运动控制连接",
    "ROBOT_STATE_STALE": "机器狗实时状态已中断，系统已禁止行走；请检查狗身主机与运动控制连接",
    "ROBOT_STATE_INVALID": "机器狗状态数据异常，系统已禁止行走；请联系工程人员",
    "ROBOT_MODE_INVALID": "机器狗运动模式数据异常，系统已禁止行走；请联系工程人员",
    "ROBOT_SPORT_ERROR": "机器狗运动控制报告故障，系统已禁止行走；请在官方 App 中检查故障",
    "ROBOT_POSTURE_NOT_READY": "请先用遥控器或官方 App 让机器狗正常站稳，再开始巡检",
    "ROBOT_ACTION_IN_PROGRESS": "机器狗正在执行其他姿态动作，请等待动作完成并站稳",
    "ROBOT_NOT_STATIONARY": "机器狗还在移动，请等待完全停稳后再开始巡检",
    "START_ATTEMPT_ID_MISMATCH": "开始请求的内部编号不一致，系统已禁止行走；请联系工程人员",
    "RUNTIME_TRACE_MISSING": "运行黑匣尚未就绪，系统不会开始行走；请稍候",
    "RUNTIME_TRACE_STALE": "运行黑匣心跳已中断，系统已禁止行走；请联系工程人员",
    "RUNTIME_TRACE_UNWRITABLE": "运行记录无法写入，系统已禁止行走；请检查存储空间",
    "RUNTIME_TRACE_NOT_PERSISTING": "运行记录已超时未落盘，系统已禁止行走；请检查存储和运行黑匣",
    "RUNTIME_TRACE_INVALID": "运行黑匣状态异常，系统已禁止行走；请联系工程人员",
    "RUNTIME_TRACE_BINDING_MISMATCH": "运行黑匣与当前地图版本不一致，系统已禁止行走；请重新发布",
}


def operator_message_for_reason(
    reason: str,
    *,
    start_distance_m: Optional[float] = None,
    start_yaw_error_deg: Optional[float] = None,
) -> str:
    """Turn stable engineering reason codes into one operator action.

    The reason code remains in the machine contract for support and audit.  A
    delivery operator should never have to understand that code to decide what
    to do next.
    """
    code = str(reason or "UNKNOWN").strip() or "UNKNOWN"
    if code == "TOO_FAR_FROM_ROUTE_START":
        distance = (
            "（当前距起点 %.2f 米）" % start_distance_m
            if start_distance_m is not None and math.isfinite(start_distance_m)
            else ""
        )
        return "机器狗已定位，但不在巡检起点附近%s；请移到操作台地图高亮的起点区域" % distance
    if code == "START_YAW_OUTSIDE_GATE":
        yaw = (
            "（当前相差 %.0f°）" % start_yaw_error_deg
            if start_yaw_error_deg is not None and math.isfinite(start_yaw_error_deg)
            else ""
        )
        return "机器狗已定位，但朝向不符合起步范围%s；请大致朝向路线起步方向，无需人工精确对准" % yaw
    exact = _OPERATOR_REASON_MESSAGES.get(code)
    if exact is not None:
        return exact
    if code.startswith("RUNTIME_BINDING_INVALID") or "RUNTIME_ARTIFACT" in code:
        return "已发布地图或路线文件发生变化，系统已禁止行走；请联系工程人员重新发布"
    if code.startswith("NAV2_GOAL_TRANSPORT"):
        return "导航服务通信失败，机器狗没有开始行走；请联系工程人员"
    if code == "NAV2_GOAL_REJECTED" or code.startswith("NAV2_FOLLOW_PATH_FAILED"):
        return "导航没有接受或完成本次路线，机器狗已停止；请联系工程人员查看运行记录"
    return "巡检暂时不能继续；请保持机器狗停止，并将原因码 %s 提供给工程人员" % code


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_json(payload: Any) -> str:
    encoded = json.dumps(
        payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _parse_profile(
    profile_path: Path,
) -> Tuple[
    PatrolSettings,
    Tuple[float, float, float],
    float,
    float,
    float,
    LocalizationQualityProfile,
]:
    try:
        profile = json.loads(profile_path.read_text(encoding="utf-8"))
        if profile.get("schema") != "go2.runtime_profile.v1":
            raise RuntimeContractError("INVALID_RUNTIME_PROFILE_SCHEMA")
        patrol = profile["patrol"]
        localization = profile["localization"]
        initialization = localization["initializationZone"]
        quality = resolve_localization_quality_profile(
            localization["qualityProfileId"]
        )
        center = initialization["center"]
        settings = PatrolSettings(
            loop_mode=str(patrol["loopMode"]),
            speed_limit_mps=float(patrol["speedLimitMps"]),
            start_max_distance_m=float(patrol["startMaxDistanceM"]),
            start_max_yaw_deg=float(patrol["startMaxYawDeg"]),
            path_sample_spacing_m=float(patrol["pathSampleSpacingM"]),
        )
        initialization_center = (
            float(center["x"]),
            float(center["y"]),
            float(center["z"]),
        )
        initialization_radius = float(initialization["radiusM"])
        initialization_yaw = normalize_angle(float(initialization["expectedYawRad"]))
        initialization_yaw_tolerance = float(initialization["yawToleranceDeg"])
    except RuntimeContractError:
        raise
    except (KeyError, OSError, TypeError, ValueError) as exc:
        raise RuntimeContractError("INVALID_RUNTIME_PROFILE: %s" % exc)
    values = (
        *initialization_center,
        initialization_radius,
        initialization_yaw,
        initialization_yaw_tolerance,
        settings.speed_limit_mps,
        settings.start_max_distance_m,
        settings.start_max_yaw_deg,
        settings.path_sample_spacing_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise RuntimeContractError("INVALID_RUNTIME_PROFILE_NONFINITE")
    if not 0.5 <= initialization_radius <= 30.0:
        raise RuntimeContractError("INVALID_INITIALIZATION_RADIUS")
    if not 5.0 <= initialization_yaw_tolerance <= 90.0:
        raise RuntimeContractError("INVALID_INITIALIZATION_YAW_TOLERANCE")
    if settings.loop_mode != "once":
        raise RuntimeContractError("UNSUPPORTED_PATROL_LOOP_MODE")
    if not 0.10 <= settings.speed_limit_mps <= 0.60:
        raise RuntimeContractError("INVALID_PATROL_SPEED_LIMIT")
    if not 0.50 <= settings.start_max_distance_m <= 5.0:
        raise RuntimeContractError("INVALID_PATROL_START_DISTANCE")
    if not 0.0 <= settings.start_max_yaw_deg <= 90.0:
        raise RuntimeContractError("INVALID_PATROL_START_YAW")
    if not 0.05 <= settings.path_sample_spacing_m <= 0.50:
        raise RuntimeContractError("INVALID_PATROL_PATH_SPACING")
    return (
        settings,
        initialization_center,
        initialization_radius,
        initialization_yaw,
        initialization_yaw_tolerance,
        quality,
    )


def load_runtime_bundle(
    map_store_root: Path,
    site_id: str,
    expected_version: str = "",
    *,
    store: Optional[MapVersionStore] = None,
) -> RuntimeBundle:
    requested_root = Path(map_store_root).resolve()
    if store is None:
        store = MapVersionStore(requested_root)
    elif Path(store.root).resolve() != requested_root:
        raise RuntimeContractError("MAP_STORE_ROOT_MISMATCH")
    activation = store.active(site_id)
    if activation is None:
        raise RuntimeContractError("NO_ACTIVE_MAP")
    version_id = str(activation["versionId"])
    if expected_version and version_id != expected_version:
        raise RuntimeContractError("ACTIVE_MAP_VERSION_CHANGED")
    manifest = store.verify(version_id)
    if manifest.get("purpose") != "release":
        raise RuntimeContractError("ACTIVE_MAP_IS_NOT_RELEASE")
    if manifest.get("manifestHash") != activation.get("manifestHash"):
        raise RuntimeContractError("ACTIVE_MAP_MANIFEST_MISMATCH")

    route_path = store.resolve_artifact(version_id, "route")
    localization_path = store.resolve_artifact(version_id, "localization_map")
    profile_path = store.resolve_artifact(version_id, "runtime_profile")
    calibration_path = store.resolve_artifact(version_id, "calibration_bundle")
    try:
        calibration_payload = json.loads(calibration_path.read_text(encoding="utf-8"))
        calibration = ReleaseCalibrationBundle.from_dict(calibration_payload)
    except (OSError, ValueError) as exc:
        raise RuntimeContractError("INVALID_RELEASE_CALIBRATION: %s" % exc)
    if calibration.digest != manifest.get("calibrationHash"):
        raise RuntimeContractError("RELEASE_CALIBRATION_HASH_MISMATCH")
    if (
        calibration.coordinate_contract.digest
        != manifest.get("coordinateContractHash")
    ):
        raise RuntimeContractError("RELEASE_COORDINATE_CONTRACT_MISMATCH")
    route = load_route(route_path)
    (
        settings,
        initialization_center,
        initialization_radius,
        initialization_yaw,
        initialization_yaw_tolerance,
        localization_quality,
    ) = _parse_profile(profile_path)
    return RuntimeBundle(
        site_id=site_id,
        version_id=version_id,
        manifest_hash=str(activation["manifestHash"]),
        route_path=route_path,
        localization_map_path=localization_path,
        runtime_profile_path=profile_path,
        route=route,
        patrol=settings,
        initialization_center=initialization_center,
        initialization_radius_m=initialization_radius,
        initialization_yaw_rad=initialization_yaw,
        initialization_yaw_tolerance_deg=initialization_yaw_tolerance,
        localization_quality=localization_quality,
        calibration_bundle_path=calibration_path,
        calibration_hash=calibration.digest,
        robot_id=calibration.robot_id,
        sensor_id=calibration.sensor_id,
    )


def load_candidate_runtime_bundle(
    *,
    site_id: str,
    version_id: str,
    localization_map_path: Path,
    route_path: Path,
    runtime_profile_path: Path,
    localization_map_hash: str,
    route_hash: str,
    runtime_profile_hash: str,
) -> RuntimeBundle:
    """Load an unactivated candidate only when all release bindings match."""
    site_id = str(site_id).strip()
    version_id = str(version_id).strip()
    if not site_id or not version_id:
        raise RuntimeContractError("CANDIDATE_SITE_AND_VERSION_REQUIRED")
    raw_paths = [
        Path(localization_map_path),
        Path(route_path),
        Path(runtime_profile_path),
    ]
    if any(path.is_symlink() or not path.is_file() for path in raw_paths):
        raise RuntimeContractError("CANDIDATE_ARTIFACT_UNAVAILABLE")
    paths = [path.resolve() for path in raw_paths]
    map_path, route_path, profile_path = paths
    try:
        route_payload = json.loads(route_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise RuntimeContractError("INVALID_CANDIDATE_ROUTE: %s" % exc)
    actual = {
        "localizationMapHash": _sha256_file(map_path),
        "routeHash": _sha256_json(route_payload),
        "runtimeProfileHash": _sha256_file(profile_path),
    }
    expected = {
        "localizationMapHash": str(localization_map_hash).strip().lower(),
        "routeHash": str(route_hash).strip().lower(),
        "runtimeProfileHash": str(runtime_profile_hash).strip().lower(),
    }
    for name in actual:
        if len(expected[name]) != 64 or actual[name] != expected[name]:
            raise RuntimeContractError("CANDIDATE_%s_MISMATCH" % name.upper())
    route = load_route(route_path)
    (
        settings,
        initialization_center,
        initialization_radius,
        initialization_yaw,
        initialization_yaw_tolerance,
        localization_quality,
    ) = _parse_profile(profile_path)
    binding = hashlib.sha256(
        (
            actual["localizationMapHash"]
            + actual["routeHash"]
            + actual["runtimeProfileHash"]
        ).encode("ascii")
    ).hexdigest()
    return RuntimeBundle(
        site_id=site_id,
        version_id=version_id,
        manifest_hash=binding,
        route_path=route_path,
        localization_map_path=map_path,
        runtime_profile_path=profile_path,
        route=route,
        patrol=settings,
        initialization_center=initialization_center,
        initialization_radius_m=initialization_radius,
        initialization_yaw_rad=initialization_yaw,
        initialization_yaw_tolerance_deg=initialization_yaw_tolerance,
        localization_quality=localization_quality,
    )


def _localization_ok(
    status: Any,
    expected_version: str,
) -> Tuple[bool, str]:
    if not isinstance(status, Mapping):
        return False, "LOCALIZATION_STATUS_MISSING"
    if status.get("schemaVersion") != 1:
        return False, "LOCALIZATION_STATUS_SCHEMA"
    if status.get("mapVersion") != expected_version:
        return False, "LOCALIZATION_MAP_VERSION_MISMATCH"
    if status.get("state") != "TRACKING":
        return False, "LOCALIZATION_NOT_TRACKING"
    if status.get("usable") is not True:
        return False, "LOCALIZATION_NOT_USABLE"
    return True, "OK"


def requested_runtime_identity_reason(
    expected_map_version: str,
    expected_route_id: str,
    active_map_version: str,
    active_route_id: str,
) -> str:
    """Validate cloud identity before the runtime constructs a Nav2 goal."""
    expected_map = str(expected_map_version or "").strip()
    expected_route = str(expected_route_id or "").strip()
    if expected_map and expected_map != active_map_version:
        return "REQUESTED_MAP_VERSION_IS_NOT_ACTIVE"
    if expected_route and expected_route != active_route_id:
        return "REQUESTED_ROUTE_IS_NOT_ACTIVE"
    return "OK"


def evaluate_robot_motion_state(
    status: Any,
    *,
    age_s: float,
    timeout_s: float,
    require_stationary: bool,
    maximum_stationary_speed_mps: float = 0.08,
    maximum_stationary_yaw_rate_rps: float = 0.12,
) -> PatrolReadiness:
    """Validate Unitree's physical sport state before authorizing motion.

    Unitree's official SportModeState contract defines mode 0 as default
    standing, 1 as balance standing and 3 as locomotion.  Other modes include
    pose, lie-down, joint lock, damping, recovery, sit and acrobatics; none may
    be treated as patrol-ready merely because localization is healthy.
    """
    if age_s < 0.0 or age_s > timeout_s:
        return PatrolReadiness(False, "POSITIONING", "ROBOT_STATE_STALE")
    if not isinstance(status, Mapping):
        return PatrolReadiness(False, "POSITIONING", "ROBOT_STATE_MISSING")
    try:
        error_code = int(status["errorCode"])
        mode_value = _finite(status["mode"], "robot_mode")
        progress = _finite(status["progress"], "robot_progress")
        vx = _finite(status["vx"], "robot_vx")
        vy = _finite(status["vy"], "robot_vy")
        yaw_rate = _finite(status["yawSpeed"], "robot_yaw_speed")
    except (KeyError, TypeError, ValueError, RouteError):
        return PatrolReadiness(False, "FAULT", "ROBOT_STATE_INVALID")
    mode = int(mode_value)
    if mode_value != mode:
        return PatrolReadiness(False, "FAULT", "ROBOT_MODE_INVALID")
    # Go2 production firmware reports 1001 on healthy robots; Unitree also
    # defines 1001 as the DAMP sport API id.  Accept only the two observed
    # baseline wire values and keep every other undocumented value fail-closed.
    if error_code not in {0, 1001}:
        return PatrolReadiness(False, "FAULT", "ROBOT_SPORT_ERROR")
    if mode not in {0, 1, 3}:
        return PatrolReadiness(False, "POSITIONING", "ROBOT_POSTURE_NOT_READY")
    if progress > 0.01:
        return PatrolReadiness(False, "POSITIONING", "ROBOT_ACTION_IN_PROGRESS")
    if require_stationary:
        if math.hypot(vx, vy) > maximum_stationary_speed_mps:
            return PatrolReadiness(False, "POSITIONING", "ROBOT_NOT_STATIONARY")
        if abs(yaw_rate) > maximum_stationary_yaw_rate_rps:
            return PatrolReadiness(False, "POSITIONING", "ROBOT_NOT_STATIONARY")
    return PatrolReadiness(True, "READY", "OK")


def evaluate_readiness(
    *,
    expected_version: str,
    localization_status: Any,
    localization_age_s: float,
    localization_timeout_s: float,
    pose_xy_yaw: Optional[Sequence[float]],
    pose_frame: str,
    pose_age_s: float,
    pose_timeout_s: float,
    route: Route,
    start_max_distance_m: float,
    start_max_yaw_deg: float,
    action_server_ready: bool,
    require_robot_state: bool = False,
    robot_motion_status: Any = None,
    robot_motion_age_s: float = float("inf"),
    robot_motion_timeout_s: float = 0.5,
    maximum_stationary_speed_mps: float = 0.08,
    maximum_stationary_yaw_rate_rps: float = 0.12,
) -> PatrolReadiness:
    gate = evaluate_runtime_gate(
        expected_version=expected_version,
        localization_status=localization_status,
        localization_age_s=localization_age_s,
        localization_timeout_s=localization_timeout_s,
        pose_xy_yaw=pose_xy_yaw,
        pose_frame=pose_frame,
        pose_age_s=pose_age_s,
        pose_timeout_s=pose_timeout_s,
        action_server_ready=action_server_ready,
        require_robot_state=require_robot_state,
        robot_motion_status=robot_motion_status,
        robot_motion_age_s=robot_motion_age_s,
        robot_motion_timeout_s=robot_motion_timeout_s,
    )
    if not gate.ready:
        return gate
    if require_robot_state:
        robot_gate = evaluate_robot_motion_state(
            robot_motion_status,
            age_s=robot_motion_age_s,
            timeout_s=robot_motion_timeout_s,
            require_stationary=True,
            maximum_stationary_speed_mps=maximum_stationary_speed_mps,
            maximum_stationary_yaw_rate_rps=maximum_stationary_yaw_rate_rps,
        )
        if not robot_gate.ready:
            return robot_gate
    pose = tuple(float(value) for value in pose_xy_yaw)
    start = route.waypoints[0]
    distance = math.hypot(pose[0] - start.x, pose[1] - start.y)
    yaw_error_deg = math.degrees(angle_distance(pose[2], start.yaw))
    if distance > start_max_distance_m:
        return PatrolReadiness(
            False, "POSITIONING", "TOO_FAR_FROM_ROUTE_START", distance, yaw_error_deg
        )
    if yaw_error_deg > start_max_yaw_deg:
        return PatrolReadiness(
            False, "POSITIONING", "START_YAW_OUTSIDE_GATE", distance, yaw_error_deg
        )
    return PatrolReadiness(True, "READY", "OK", distance, yaw_error_deg)


def evaluate_runtime_gate(
    *,
    expected_version: str,
    localization_status: Any,
    localization_age_s: float,
    localization_timeout_s: float,
    pose_xy_yaw: Optional[Sequence[float]],
    pose_frame: str,
    pose_age_s: float,
    pose_timeout_s: float,
    action_server_ready: bool,
    require_robot_state: bool = False,
    robot_motion_status: Any = None,
    robot_motion_age_s: float = float("inf"),
    robot_motion_timeout_s: float = 0.5,
) -> PatrolReadiness:
    """Gate continued motion without re-applying the route-start constraint."""
    if localization_age_s < 0.0 or localization_age_s > localization_timeout_s:
        return PatrolReadiness(False, "LOCALIZING", "LOCALIZATION_STATUS_STALE")
    localization_valid, reason = _localization_ok(localization_status, expected_version)
    if not localization_valid:
        return PatrolReadiness(False, "LOCALIZING", reason)
    if pose_xy_yaw is None or len(pose_xy_yaw) != 3:
        return PatrolReadiness(False, "LOCALIZING", "LOCALIZATION_POSE_MISSING")
    if pose_frame != "map":
        return PatrolReadiness(False, "FAULT", "LOCALIZATION_POSE_FRAME")
    if pose_age_s < 0.0 or pose_age_s > pose_timeout_s:
        return PatrolReadiness(False, "LOCALIZING", "LOCALIZATION_POSE_STALE")
    try:
        tuple(_finite(value, "localization_pose") for value in pose_xy_yaw)
    except RouteError:
        return PatrolReadiness(False, "FAULT", "LOCALIZATION_POSE_NONFINITE")
    if not action_server_ready:
        return PatrolReadiness(False, "BOOTING", "NAV2_FOLLOW_PATH_UNAVAILABLE")
    if require_robot_state:
        robot_gate = evaluate_robot_motion_state(
            robot_motion_status,
            age_s=robot_motion_age_s,
            timeout_s=robot_motion_timeout_s,
            require_stationary=False,
        )
        if not robot_gate.ready:
            return robot_gate
    return PatrolReadiness(True, "READY", "OK")


def monotonic_age(received_at: float, now: Optional[float] = None) -> float:
    if received_at <= 0.0:
        return float("inf")
    current = time.monotonic() if now is None else now
    return max(0.0, current - received_at)
