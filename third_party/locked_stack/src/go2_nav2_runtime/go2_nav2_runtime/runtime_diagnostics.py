#!/usr/bin/env python3
"""Turn bounded production flight logs into an actionable diagnostic report."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import time
from collections import Counter
from pathlib import Path


SUGGESTIONS = {
    "runtime_motion_not_authorized": "先看同一时刻 runtime.reason；不得绕过运行授权门。",
    "runtime_authorization_timeout": "检查巡检管理器进程、ROS 调度和授权话题新鲜度。",
    "localization_status_timeout": "检查持续定位进程、点云输入和 10 Hz 状态心跳。",
    "localization_not_tracking": "检查定位质量、地图身份和 LOST/DEGRADED 原因。",
    "localization_not_usable": "检查配准质量门，不要通过放宽安全门恢复运动。",
    "localization_map_version_mismatch": "核对 active release，重启整代运行时而非热换地图。",
    "cmd_timeout": "检查 MPPI、速度平滑器和 Collision Monitor 的节点存活与调度延迟。",
    "cloud_timeout": "检查 MID-360、标定点云转换、点云格式和有效点数量。",
    "obstacle": "回看四级速度链和安全状态中的距离/点数，再核对现场视频。",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _nonnegative_integer(value):
    try:
        return max(0, int(value))
    except (TypeError, ValueError):
        return 0


def _is_nonzero(payload, epsilon=1.0e-4):
    if not isinstance(payload, dict):
        return False
    values = [_finite(payload.get(name)) for name in ("vx", "vy", "wz")]
    return all(value is not None for value in values) and any(
        abs(value) > epsilon for value in values
    )


def _read_json_lines(paths, *, prefixed=False):
    records = []
    errors = []
    for path in paths:
        try:
            with path.open("r", encoding="utf-8") as stream:
                for line_number, raw in enumerate(stream, 1):
                    line = raw.strip()
                    if not line:
                        continue
                    if prefixed:
                        if " " not in line:
                            errors.append("%s:%d:missing_prefix" % (path, line_number))
                            continue
                        prefix, line = line.split(" ", 1)
                    else:
                        prefix = ""
                    try:
                        payload = json.loads(line)
                    except ValueError:
                        errors.append("%s:%d:invalid_json" % (path, line_number))
                        continue
                    if isinstance(payload, dict):
                        if prefix:
                            payload = {"recordType": prefix, **payload}
                        records.append(payload)
                    else:
                        errors.append("%s:%d:not_object" % (path, line_number))
        except OSError as exc:
            errors.append("%s:read:%s" % (path, exc))
    return records, errors


def diagnose(log_dir: Path):
    root = Path(log_dir).resolve()
    if root.is_symlink() or not root.is_dir():
        raise ValueError("log directory must be a real directory")
    trace_paths = sorted(root.glob("runtime-*.jsonl"))
    performance_paths = sorted(root.glob("performance.jsonl*"))
    sdk_paths = [root / "sdk-events.jsonl", root / "sdk-events.jsonl.1"]
    sdk_paths = [path for path in sdk_paths if path.is_file()]

    trace, trace_errors = _read_json_lines(trace_paths)
    performance, performance_errors = _read_json_lines(
        performance_paths, prefixed=True
    )
    sdk, sdk_errors = _read_json_lines(sdk_paths)
    streams = Counter()
    safety_reasons = Counter()
    runtime_reasons = Counter()
    localization_reasons = Counter()
    localization_quality_profiles = set()
    localization_numeric = {
        "maximumRegistrationMs": None,
        "maximumLocalMapRefreshMs": None,
        "maximumRegistrationDeadlineMissCount": 0,
        "minimumLocalMapTrackingPoints": None,
        "minimumLocalMapCoarsePoints": None,
        "recoverySamples": 0,
        "unusableSamples": 0,
    }
    command_nonzero = Counter()
    ever_motion_authorized = False
    bindings = set()
    latest_safety = None
    safety_numeric = {
        "maximumObstacleStopPointCount": 0,
        "maximumLeftCorridorPointCount": 0,
        "maximumRightCorridorPointCount": 0,
        "minimumObstacleDistanceM": None,
    }
    first_wall = None
    last_wall = None
    for record in trace:
        if record.get("schema") != "go2.runtime_trace.v1":
            trace_errors.append("trace_schema")
            continue
        stream = str(record.get("stream", ""))
        payload = record.get("payload")
        streams[stream] += 1
        wall = _finite(record.get("wallTime"))
        if wall is not None:
            first_wall = wall if first_wall is None else min(first_wall, wall)
            last_wall = wall if last_wall is None else max(last_wall, wall)
        if stream == "recorder" and isinstance(payload, dict):
            binding = payload.get("binding")
            if isinstance(binding, dict):
                bindings.add(json.dumps(binding, sort_keys=True))
        elif stream == "safety" and isinstance(payload, dict):
            latest_safety = payload
            safety_reasons[str(payload.get("stopReason", "unknown"))] += 1
            safety_numeric["maximumObstacleStopPointCount"] = max(
                safety_numeric["maximumObstacleStopPointCount"],
                _nonnegative_integer(payload.get("obstacleStopPointCount")),
            )
            safety_numeric["maximumLeftCorridorPointCount"] = max(
                safety_numeric["maximumLeftCorridorPointCount"],
                _nonnegative_integer(payload.get("leftCorridorPointCount")),
            )
            safety_numeric["maximumRightCorridorPointCount"] = max(
                safety_numeric["maximumRightCorridorPointCount"],
                _nonnegative_integer(payload.get("rightCorridorPointCount")),
            )
            distance = _finite(payload.get("obstacleDistanceM"))
            if distance is not None:
                current = safety_numeric["minimumObstacleDistanceM"]
                safety_numeric["minimumObstacleDistanceM"] = (
                    distance if current is None else min(current, distance)
                )
        elif stream == "runtime" and isinstance(payload, dict):
            runtime_reasons[str(payload.get("reason", "unknown"))] += 1
            ever_motion_authorized = (
                ever_motion_authorized or payload.get("motionAuthorized") is True
            )
        elif stream == "authorization" and isinstance(payload, dict):
            ever_motion_authorized = (
                ever_motion_authorized or payload.get("authorized") is True
            )
        elif stream == "localization" and isinstance(payload, dict):
            localization_reasons[
                str(payload.get("reason") or payload.get("state") or "unknown")
            ] += 1
            quality_profile = str(payload.get("qualityProfileId", "")).strip()
            if quality_profile:
                localization_quality_profiles.add(quality_profile)
            registration_ms = _finite(payload.get("registrationMs"))
            if registration_ms is not None:
                current = localization_numeric["maximumRegistrationMs"]
                localization_numeric["maximumRegistrationMs"] = (
                    registration_ms
                    if current is None
                    else max(current, registration_ms)
                )
            refresh_ms = _finite(payload.get("localMapRefreshMs"))
            if refresh_ms is not None:
                current = localization_numeric["maximumLocalMapRefreshMs"]
                localization_numeric["maximumLocalMapRefreshMs"] = (
                    refresh_ms if current is None else max(current, refresh_ms)
                )
            localization_numeric["maximumRegistrationDeadlineMissCount"] = max(
                localization_numeric["maximumRegistrationDeadlineMissCount"],
                _nonnegative_integer(
                    payload.get("registrationDeadlineMissCount")
                ),
            )
            for field, output_name in (
                ("localMapTrackingPoints", "minimumLocalMapTrackingPoints"),
                ("localMapCoarsePoints", "minimumLocalMapCoarsePoints"),
            ):
                value = _nonnegative_integer(payload.get(field))
                if value:
                    current = localization_numeric[output_name]
                    localization_numeric[output_name] = (
                        value if current is None else min(current, value)
                    )
            if payload.get("searchMode") == "RECOVERY":
                localization_numeric["recoverySamples"] += 1
            if payload.get("usable") is not True:
                localization_numeric["unusableSamples"] += 1
        elif stream in {
            "nav2_raw_cmd",
            "nav2_smoothed_cmd",
            "collision_monitored_cmd",
            "final_cmd",
        } and _is_nonzero(payload):
            command_nonzero[stream] += 1

    sdk_events = Counter(str(record.get("event", "unknown")) for record in sdk)
    sdk_stop_failures = [
        record
        for record in sdk
        if str(record.get("event", "")).endswith("stop")
        and record.get("returnCode") != 0
    ]
    perf_samples = [
        record
        for record in performance
        if record.get("recordType") == "PERF_SAMPLE"
    ]
    max_cpu = max(
        (_finite(record.get("system_cpu_pct")) or 0.0 for record in perf_samples),
        default=None,
    )
    max_temperature = max(
        (_finite(record.get("max_temperature_c")) or 0.0 for record in perf_samples),
        default=None,
    )
    max_monitor_late = max(
        (_finite(record.get("monitor_wake_late_ms")) or 0.0 for record in perf_samples),
        default=None,
    )

    findings = []
    if not trace_paths or not trace:
        findings.append(
            {
                "severity": "error",
                "code": "RUNTIME_TRACE_MISSING",
                "action": "先恢复 runtime_trace_recorder；没有飞行记录不得完成投产验收。",
            }
        )
    if len(bindings) > 1:
        findings.append(
            {
                "severity": "error",
                "code": "MULTIPLE_RUNTIME_BINDINGS",
                "action": "按运行实例拆分分析，并核对是否发生了地图或版本切换。",
            }
        )
    if len(localization_quality_profiles) > 1:
        findings.append(
            {
                "severity": "error",
                "code": "MULTIPLE_LOCALIZATION_QUALITY_PROFILES",
                "action": "运行中出现多个定位参数档；核对是否跨地图代热切换。",
            }
        )
    if localization_numeric["maximumRegistrationDeadlineMissCount"]:
        findings.append(
            {
                "severity": "warning",
                "code": "LOCALIZATION_REGISTRATION_DEADLINE_MISS",
                "count": localization_numeric[
                    "maximumRegistrationDeadlineMissCount"
                ],
                "action": "联查同一时刻进程 CPU、温度、局部地图刷新与 MPPI 负载。",
            }
        )
    if (
        localization_numeric["minimumLocalMapTrackingPoints"] is not None
        and localization_numeric["minimumLocalMapTrackingPoints"] < 500
    ):
        findings.append(
            {
                "severity": "error",
                "code": "LOCALIZATION_LOCAL_MAP_SPARSE",
                "action": "停止运动验收；核对地图范围、当前位置和稳定定位几何。",
            }
        )
    for reason, count in safety_reasons.most_common():
        if reason == "normal":
            continue
        if not ever_motion_authorized and reason in {
            "runtime_motion_not_authorized",
            "runtime_authorization_timeout",
        }:
            # A fail-closed authorization gate is the expected state before an
            # operator starts patrol.  Keep the counts in the report, but do
            # not turn a healthy zero-motion commissioning window into an
            # incident.
            continue
        findings.append(
            {
                "severity": "warning",
                "code": "SAFETY_OVERRIDE_%s" % reason.upper(),
                "count": count,
                "action": SUGGESTIONS.get(
                    reason, "按时间戳联查 runtime、localization 和四级速度链。"
                ),
            }
        )
    if command_nonzero["nav2_raw_cmd"] and not command_nonzero["nav2_smoothed_cmd"]:
        findings.append(
            {
                "severity": "warning",
                "code": "COMMAND_STOPPED_AT_SMOOTHER",
                "action": "检查 velocity_smoother 超时、速度边界和生命周期状态。",
            }
        )
    if command_nonzero["nav2_smoothed_cmd"] and not command_nonzero["collision_monitored_cmd"]:
        findings.append(
            {
                "severity": "warning",
                "code": "COMMAND_STOPPED_AT_COLLISION_MONITOR",
                "action": "检查 Collision Monitor 区域、点云点数和接近碰撞预测。",
            }
        )
    if command_nonzero["collision_monitored_cmd"] and not command_nonzero["final_cmd"]:
        findings.append(
            {
                "severity": "warning",
                "code": "COMMAND_STOPPED_AT_FINAL_SAFETY",
                "action": "使用同一时刻 safety.stopReason 定位最终安全门输入。",
            }
        )
    if sdk_stop_failures:
        findings.append(
            {
                "severity": "error",
                "code": "SDK_STOPMOVE_FAILED",
                "count": len(sdk_stop_failures),
                "action": "停止运动验收，检查 Unitree SDK/DDS/网卡后再恢复。",
            }
        )
    if max_cpu is not None and max_cpu >= 90.0:
        findings.append(
            {
                "severity": "warning",
                "code": "SYSTEM_CPU_SATURATION",
                "value": max_cpu,
                "action": "联查 MPPI、定位和点云线程；优先降低预览/上传负载。",
            }
        )
    if max_temperature is not None and max_temperature >= 85.0:
        findings.append(
            {
                "severity": "warning",
                "code": "THERMAL_PRESSURE",
                "value": max_temperature,
                "action": "检查散热、nvpmodel 和降频，再评估控制周期抖动。",
            }
        )

    input_files = []
    for path in trace_paths + performance_paths + sdk_paths:
        try:
            input_files.append(
                {
                    "path": str(path),
                    "size": path.stat().st_size,
                    "sha256": _sha256(path),
                }
            )
        except OSError:
            pass
    errors = trace_errors + performance_errors + sdk_errors
    severity = "healthy"
    if any(item["severity"] == "error" for item in findings) or errors:
        severity = "error"
    elif findings:
        severity = "attention"
    return {
        "schema": "go2.runtime_diagnostic_report.v1",
        "generatedAt": time.time(),
        "logDirectory": str(root),
        "result": severity,
        "window": {
            "firstWallTime": first_wall,
            "lastWallTime": last_wall,
            "durationS": (
                last_wall - first_wall
                if first_wall is not None and last_wall is not None
                else None
            ),
        },
        "bindings": [json.loads(value) for value in sorted(bindings)],
        "streamCounts": dict(sorted(streams.items())),
        "safetyReasons": dict(safety_reasons.most_common()),
        "safetyMetrics": safety_numeric,
        "latestSafety": latest_safety,
        "runtimeReasons": dict(runtime_reasons.most_common()),
        "localizationReasons": dict(localization_reasons.most_common()),
        "localizationQualityProfiles": sorted(localization_quality_profiles),
        "localizationMetrics": localization_numeric,
        "nonzeroCommandSamples": dict(command_nonzero),
        "everMotionAuthorized": ever_motion_authorized,
        "sdkEvents": dict(sdk_events),
        "performance": {
            "sampleCount": len(perf_samples),
            "maximumSystemCpuPct": max_cpu,
            "maximumTemperatureC": max_temperature,
            "maximumMonitorWakeLateMs": max_monitor_late,
        },
        "parseErrors": errors,
        "findings": findings,
        "inputs": input_files,
    }


def _write_new(path: Path, payload):
    content = (
        json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        os.close(descriptor)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path(os.environ.get("GO2_RUNTIME_LOG_DIR", "/data/go2/runtime_logs")),
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        report = diagnose(args.log_dir)
        if args.output is not None:
            _write_new(args.output.resolve(), report)
        print(json.dumps(report, ensure_ascii=False, sort_keys=True, allow_nan=False))
        return 0 if report["result"] == "healthy" else 2
    except (OSError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
