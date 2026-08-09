from __future__ import annotations

import json
import math
import os
import tempfile
import threading
from pathlib import Path
from typing import Any


PROFILE_SCHEMA = "gogoguard.navigation_profile.v2"
LEGACY_PROFILE_SCHEMA = "gogoguard.navigation_profile.v1"
DEFAULT_PROFILE: dict[str, Any] = {
    "schema": PROFILE_SCHEMA,
    "revision": 1,
    "motion": {
        "straightSpeedMps": 0.60,
        "detourSpeedMps": 0.40,
        "turnSpeedRadps": 0.40,
        "lateralSpeedMps": 0.20,
        "accelerationMps2": 0.90,
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
        "mppiRetryLimit": 2,
        "detourAttemptLimit": 2,
    },
    "localization": {
        "statusTimeoutS": 0.60,
        "dropoutGraceS": 1.00,
        "recoveryStableS": 0.50,
    },
    "controller": {
        "frequencyHz": 15.0,
        "timeSteps": 56,
        "batchSize": 700,
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
    schema = str(value.get("schema") or LEGACY_PROFILE_SCHEMA)
    if schema not in {PROFILE_SCHEMA, LEGACY_PROFILE_SCHEMA}:
        raise ProfileError("参数配置版本不支持")
    # V1 called the Nav2 progress watchdog an obstacle decision. Preserve the
    # saved value during migration, but give it only its real V2 meaning.
    legacy_progress_timeout = avoidance.get("blockedDecisionS", 5.0)
    normalized = {
        "schema": PROFILE_SCHEMA,
        "revision": int(value.get("revision") or 1),
        "motion": {
            "straightSpeedMps": _finite(motion.get("straightSpeedMps"), "直线速度", 0.20, 1.00),
            "detourSpeedMps": _finite(
                motion.get("detourSpeedMps", 0.40), "局部绕行速度", 0.24, 0.80
            ),
            "turnSpeedRadps": _finite(motion.get("turnSpeedRadps"), "转弯角速度", 0.10, 0.80),
            "lateralSpeedMps": _finite(motion.get("lateralSpeedMps"), "侧向速度", 0.05, 0.40),
            "accelerationMps2": _finite(motion.get("accelerationMps2"), "加速度", 0.20, 2.00),
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
            "mppiRetryLimit": int(
                _finite(recovery.get("mppiRetryLimit", 2), "MPPI 短暂失败重试次数", 0, 3)
            ),
            "detourAttemptLimit": int(
                _finite(recovery.get("detourAttemptLimit", 2), "单次巡检绕行上限", 1, 3)
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
            "batchSize": int(_finite(controller.get("batchSize"), "MPPI 样本数", 200, 2000)),
            "iterationCount": int(_finite(controller.get("iterationCount"), "MPPI 迭代数", 1, 2)),
        },
    }
    if normalized["avoidance"]["slowZoneHalfWidthM"] < normalized["avoidance"]["stopZoneHalfWidthM"]:
        raise ProfileError("减速区必须覆盖停车区")
    if normalized["avoidance"]["slowZoneFrontM"] < normalized["avoidance"]["stopZoneFrontM"]:
        raise ProfileError("减速区前缘必须早于停车区")
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
        * normalized["motion"]["straightSpeedMps"]
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
