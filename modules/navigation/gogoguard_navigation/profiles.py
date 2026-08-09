from __future__ import annotations

import json
import math
import os
import tempfile
import threading
from pathlib import Path
from typing import Any


PROFILE_SCHEMA = "gogoguard.navigation_profile.v5"
LEGACY_PROFILE_SCHEMAS = {
    "gogoguard.navigation_profile.v1",
    "gogoguard.navigation_profile.v2",
    "gogoguard.navigation_profile.v3",
    "gogoguard.navigation_profile.v4",
}
DEFAULT_PROFILE: dict[str, Any] = {
    "schema": PROFILE_SCHEMA,
    "revision": 1,
    "motion": {
        "targetCruiseMps": 0.60,
        "maxForwardMps": 0.90,
        "detourSpeedMps": 0.40,
        "turnSpeedRadps": 0.40,
        "lateralSpeedMps": 0.20,
        "accelerationMps2": 0.90,
        "decelerationMps2": 0.90,
    },
    "avoidance": {
        "stopZoneFrontM": 0.55,
        "stopZoneRearM": 0.38,
        "stopZoneHalfWidthM": 0.27,
        "slowZoneFrontM": 1.00,
        "slowZoneRearM": 0.55,
        "slowZoneHalfWidthM": 0.30,
        "slowdownRatio": 0.85,
        "rejoinLookaheadM": 2.0,
        "obstructionCostThreshold": 65,
        "obstructionMinSamples": 2,
        "obstructionConfirmationS": 0.50,
    },
    "recovery": {
        "progressTimeoutS": 5.0,
        "replanIntervalS": 0.75,
    },
    "localization": {
        "statusTimeoutS": 0.60,
        "dropoutGraceS": 1.00,
        "recoveryStableS": 0.50,
    },
    "controller": {
        "frequencyHz": 15.0,
        "timeSteps": 56,
        "batchSize": 1000,
        "iterationCount": 1,
    },
}


class ProfileError(ValueError):
    pass


def _finite(value: Any, label: str, minimum: float, maximum: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ProfileError(f"{label} 必须是数字") from exc
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ProfileError(f"{label} 必须在 {minimum} 到 {maximum} 之间")
    return result


def validate_profile(value: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProfileError("参数配置必须是对象")
    motion = dict(value.get("motion") or {})
    avoidance = dict(value.get("avoidance") or {})
    recovery = dict(value.get("recovery") or {})
    localization = dict(value.get("localization") or {})
    controller = dict(value.get("controller") or {})
    schema = str(value.get("schema") or "gogoguard.navigation_profile.v1")
    if schema not in {PROFILE_SCHEMA, *LEGACY_PROFILE_SCHEMAS}:
        raise ProfileError("参数配置版本不支持")
    legacy_v1_v2 = schema in {
        "gogoguard.navigation_profile.v1",
        "gogoguard.navigation_profile.v2",
    }

    def matches_number(raw: Any, expected: float) -> bool:
        try:
            return float(raw) == expected
        except (TypeError, ValueError):
            return False

    # V1/V2 allowed values which the commissioned Unitree bridge could never
    # execute. Preserve old profiles, but migrate those impossible requests to
    # the real receiver limits. V3 rejects future out-of-range writes.
    legacy_cruise = motion.get("straightSpeedMps", 0.60)
    target_cruise = motion.get("targetCruiseMps", legacy_cruise)
    max_forward = motion.get("maxForwardMps")
    if max_forward is None and schema != PROFILE_SCHEMA:
        try:
            max_forward = max(0.90, float(target_cruise))
        except (TypeError, ValueError):
            # Let the field-specific validator below report the malformed
            # cruise value instead of leaking a raw migration exception.
            max_forward = 0.90
    detour_speed = motion.get("detourSpeedMps", 0.40)
    turn_speed = motion.get("turnSpeedRadps", 0.40)
    lateral_speed = motion.get("lateralSpeedMps", 0.20)
    if legacy_v1_v2:
        try:
            lateral_speed = min(float(lateral_speed), 0.20)
        except (TypeError, ValueError):
            pass
    acceleration = motion.get("accelerationMps2", 0.90)
    deceleration = motion.get("decelerationMps2", 0.90)
    # V4's first field run proved that its whole aggressive speed tuple was
    # unsafe for path feasibility: a 0.60 m/s VelocityDeadband objective made
    # low-speed corner trajectories prohibitively expensive. Migrate only the
    # exact shipped V4 tuple back to the last route-completing speed envelope;
    # preserve independently edited operator values.
    if schema == "gogoguard.navigation_profile.v4" and all(
        (
            matches_number(target_cruise, 0.60),
            matches_number(max_forward, 0.90),
            matches_number(detour_speed, 0.60),
            matches_number(turn_speed, 0.60),
            matches_number(acceleration, 2.00),
            matches_number(deceleration, 2.50),
        )
    ):
        detour_speed = 0.40
        turn_speed = 0.40
        acceleration = 0.90
        deceleration = 0.90
    batch_size = controller.get("batchSize")
    if legacy_v1_v2 and (
        matches_number(controller.get("frequencyHz"), 15.0)
        and matches_number(controller.get("timeSteps"), 56)
        and matches_number(controller.get("batchSize"), 700)
        and matches_number(controller.get("iterationCount"), 1)
    ):
        batch_size = 1000
    # V1 called the Nav2 progress watchdog an obstacle decision. Preserve the
    # saved value during migration, but give it only its real V2 meaning.
    legacy_progress_timeout = avoidance.get("blockedDecisionS", 5.0)
    normalized = {
        "schema": PROFILE_SCHEMA,
        "revision": int(value.get("revision") or 1),
        "motion": {
            "targetCruiseMps": _finite(target_cruise, "MPPI前进速度上限", 0.40, 0.90),
            "maxForwardMps": _finite(max_forward, "接收链路硬上限", 0.60, 0.90),
            "detourSpeedMps": _finite(
                detour_speed, "局部绕行速度", 0.24, 0.90
            ),
            "turnSpeedRadps": _finite(turn_speed, "转弯角速度", 0.10, 0.60),
            "lateralSpeedMps": _finite(lateral_speed, "侧向速度", 0.05, 0.20),
            "accelerationMps2": _finite(acceleration, "加速度", 0.50, 4.00),
            "decelerationMps2": _finite(deceleration, "减速度", 0.50, 5.00),
        },
        "avoidance": {
            "stopZoneFrontM": _finite(avoidance.get("stopZoneFrontM"), "停车区前缘", 0.40, 1.00),
            "stopZoneRearM": _finite(avoidance.get("stopZoneRearM"), "停车区后缘", 0.25, 0.80),
            "stopZoneHalfWidthM": _finite(avoidance.get("stopZoneHalfWidthM"), "停车区半宽", 0.22, 0.50),
            "slowZoneFrontM": _finite(avoidance.get("slowZoneFrontM"), "减速区前缘", 0.60, 2.00),
            "slowZoneRearM": _finite(avoidance.get("slowZoneRearM"), "减速区后缘", 0.30, 1.00),
            "slowZoneHalfWidthM": _finite(avoidance.get("slowZoneHalfWidthM"), "减速区半宽", 0.25, 0.70),
            "slowdownRatio": _finite(avoidance.get("slowdownRatio"), "减速比例", 0.50, 1.00),
            "rejoinLookaheadM": _finite(avoidance.get("rejoinLookaheadM"), "重入前视距离", 0.50, 6.00),
            "obstructionCostThreshold": int(
                _finite(
                    avoidance.get("obstructionCostThreshold", 65),
                    "路线障碍代价阈值",
                    1,
                    100,
                )
            ),
            "obstructionMinSamples": int(
                _finite(
                    avoidance.get("obstructionMinSamples", 2),
                    "路线障碍连续样本",
                    1,
                    10,
                )
            ),
            "obstructionConfirmationS": _finite(
                avoidance.get("obstructionConfirmationS", 0.50),
                "路线障碍持续确认时间",
                0.20,
                2.00,
            ),
        },
        "recovery": {
            "progressTimeoutS": _finite(
                recovery.get("progressTimeoutS", legacy_progress_timeout),
                "运动进展观察时间",
                2.0,
                12.0,
            ),
            "replanIntervalS": _finite(
                recovery.get("replanIntervalS", 0.75), "持续重新规划间隔", 0.25, 3.00
            ),
        },
        "localization": {
            "statusTimeoutS": _finite(localization.get("statusTimeoutS"), "定位消息超时", 0.20, 2.00),
            "dropoutGraceS": _finite(localization.get("dropoutGraceS"), "定位短暂丢失容忍", 0.20, 5.00),
            "recoveryStableS": _finite(localization.get("recoveryStableS"), "定位恢复稳定时间", 0.20, 3.00),
        },
        "controller": {
            "frequencyHz": _finite(controller.get("frequencyHz"), "MPPI 频率", 10.0, 30.0),
            "timeSteps": int(_finite(controller.get("timeSteps"), "MPPI 时域步数", 20, 80)),
            "batchSize": int(_finite(batch_size, "MPPI 样本数", 200, 2000)),
            "iterationCount": int(_finite(controller.get("iterationCount"), "MPPI 迭代数", 1, 2)),
        },
    }
    if normalized["avoidance"]["slowZoneHalfWidthM"] < normalized["avoidance"]["stopZoneHalfWidthM"]:
        raise ProfileError("减速区必须覆盖停车区")
    if normalized["avoidance"]["slowZoneFrontM"] < normalized["avoidance"]["stopZoneFrontM"]:
        raise ProfileError("减速区前缘必须早于停车区")
    if normalized["motion"]["maxForwardMps"] < normalized["motion"]["targetCruiseMps"]:
        raise ProfileError("前进控制上限必须高于或等于目标实际巡航速度")
    if (
        normalized["motion"]["detourSpeedMps"]
        * normalized["avoidance"]["slowdownRatio"]
        < 0.20
    ):
        raise ProfileError("局部绕行经过减速后必须保持至少 0.20 m/s 有效步态")
    workload = (
        normalized["controller"]["frequencyHz"]
        * normalized["controller"]["timeSteps"]
        * normalized["controller"]["batchSize"]
        * normalized["controller"]["iterationCount"]
    )
    if workload > 1_200_000:
        raise ProfileError("MPPI 计算量超过背板的已校验上限")
    prediction_distance = (
        normalized["controller"]["timeSteps"]
        / normalized["controller"]["frequencyHz"]
        * normalized["motion"]["targetCruiseMps"]
    )
    if prediction_distance < normalized["avoidance"]["slowZoneFrontM"]:
        raise ProfileError("MPPI 预测距离必须覆盖减速区前缘")
    return normalized


class NavigationProfileStore:
    """Versioned operator settings; one file is the active robot profile."""

    def __init__(self, data_root: Path) -> None:
        self.root = Path(data_root) / "navigation" / "profiles"
        self.active_path = self.root / "active.json"
        self.history_root = self.root / "history"
        self._lock = threading.Lock()
        self.history_root.mkdir(parents=True, exist_ok=True)
        if not self.active_path.exists():
            self._write(self.active_path, DEFAULT_PROFILE)

    @staticmethod
    def _write(path: Path, value: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                json.dump(value, handle, ensure_ascii=False, indent=2, sort_keys=True)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def get(self) -> dict[str, Any]:
        try:
            return validate_profile(json.loads(self.active_path.read_text(encoding="utf-8")))
        except (OSError, ValueError, ProfileError):
            return validate_profile(DEFAULT_PROFILE)

    def update(self, value: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            current = self.get()
            candidate = validate_profile(value)
            candidate["revision"] = int(current.get("revision", 1)) + 1
            self._write(self.history_root / f"revision-{current['revision']:04d}.json", current)
            self._write(self.active_path, candidate)
            return {
                "profile": candidate,
                "apply": "restart_navigation_runtime",
                "message": "参数已保存；停止巡检并重启定位与 Nav2 后生效",
            }

    def rollback(self) -> dict[str, Any]:
        with self._lock:
            history = sorted(self.history_root.glob("revision-*.json"))
            if not history:
                raise ProfileError("没有可回滚的历史参数")
            previous = validate_profile(json.loads(history[-1].read_text(encoding="utf-8")))
            current = self.get()
            previous["revision"] = int(current.get("revision", 1)) + 1
            self._write(self.active_path, previous)
            history[-1].unlink()
            return {
                "profile": previous,
                "apply": "restart_navigation_runtime",
                "message": "已回滚到上一版参数",
            }
