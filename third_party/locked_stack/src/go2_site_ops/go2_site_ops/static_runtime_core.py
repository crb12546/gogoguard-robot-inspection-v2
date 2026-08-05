"""ROS-free evaluation for a no-motion robot runtime commissioning receipt."""

from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Sequence


RECEIPT_SCHEMA = "go2.static_runtime_commissioning.v1"
REQUIRED_TOPICS = {
    "/go2/runtime/status": "std_msgs/msg/String",
    "/localization/status": "std_msgs/msg/String",
    "/go2/safety/status": "std_msgs/msg/String",
    "/go2/runtime/trace_status": "std_msgs/msg/String",
    "/go2/runtime/motion_authorized": "std_msgs/msg/Bool",
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/navigation/cloud_body": "sensor_msgs/msg/PointCloud2",
    "/Odometry": "nav_msgs/msg/Odometry",
    "/lf/sportmodestate": "unitree_go/msg/SportModeState",
}
REQUIRED_SERVICES = {
    "/go2/patrol/start": "go2_nav2_interfaces/srv/StartPatrol",
    "/go2/patrol/stop": "std_srvs/srv/Trigger",
    "/localization/reset": "std_srvs/srv/Trigger",
}
# The shipped Go2 firmware observed on the real robot reports 1001 in a
# healthy, stationary SportModeState.  Unitree's own sport API also assigns
# 1001 to DAMP.  Treating every non-zero wire value as a fault would therefore
# make a healthy robot permanently impossible to commission.  No other
# undocumented value is accepted.
NORMAL_UNITREE_SPORT_STATUS_CODES = {0, 1001}


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _positive_integer(value: Any) -> bool:
    try:
        return int(value) > 0
    except (TypeError, ValueError):
        return False


def _samples(evidence: Mapping[str, Any], name: str) -> Sequence[Any]:
    samples = evidence.get("samples", {}).get(name, [])
    return samples if isinstance(samples, list) else []


def evaluate_static_runtime(
    evidence: Mapping[str, Any],
    *,
    expected_map_version: str,
    expected_manifest_hash: str,
) -> Dict[str, Any]:
    """Evaluate a read-only snapshot without claiming localization accuracy."""

    errors = []
    topic_types = evidence.get("topicTypes", {})
    service_types = evidence.get("serviceTypes", {})
    for name, expected in REQUIRED_TOPICS.items():
        observed = topic_types.get(name, []) if isinstance(topic_types, Mapping) else []
        if observed != [expected]:
            errors.append("topic_type:%s" % name)
    for name, expected in REQUIRED_SERVICES.items():
        observed = service_types.get(name, []) if isinstance(service_types, Mapping) else []
        if expected not in observed:
            errors.append("service_type:%s" % name)

    for name in REQUIRED_TOPICS:
        if len(_samples(evidence, name)) < 2:
            errors.append("sample_count:%s" % name)

    runtime = _samples(evidence, "/go2/runtime/status")
    runtime_instance = None
    previous_sequence = None
    for sample in runtime:
        if not isinstance(sample, Mapping):
            errors.append("runtime_status_json")
            continue
        instance = sample.get("runtimeInstanceId")
        try:
            sequence = int(sample.get("statusSequence"))
        except (TypeError, ValueError):
            sequence = -1
        if (
            sample.get("schema") != "go2.runtime_status.v1"
            or not isinstance(instance, str)
            or len(instance) != 32
            or sequence <= 0
            or sample.get("mapVersion") != expected_map_version
            or sample.get("manifestHash") != expected_manifest_hash
            or sample.get("state")
            not in {"READY", "POSITIONING", "LOCALIZING", "COMPLETED"}
            or sample.get("motionAuthorized") is not False
        ):
            errors.append("runtime_status_contract")
        if runtime_instance is not None and instance != runtime_instance:
            errors.append("runtime_generation_changed")
        if previous_sequence is not None and sequence <= previous_sequence:
            errors.append("runtime_sequence_not_increasing")
        runtime_instance = instance
        previous_sequence = sequence

    localization = _samples(evidence, "/localization/status")
    for sample in localization:
        if (
            not isinstance(sample, Mapping)
            or sample.get("schemaVersion") != 1
            or sample.get("mapVersion") != expected_map_version
            or sample.get("state") not in {"SEARCHING", "TRACKING"}
            or not isinstance(sample.get("usable"), bool)
        ):
            errors.append("localization_status_contract")
    localization_tracking = bool(
        localization
        and isinstance(localization[-1], Mapping)
        and localization[-1].get("state") == "TRACKING"
        and localization[-1].get("usable") is True
    )

    safety = _samples(evidence, "/go2/safety/status")
    for sample in safety:
        if (
            not isinstance(sample, Mapping)
            or sample.get("schema") != "go2.safety_status.v1"
            or sample.get("expectedMapVersion") != expected_map_version
            or sample.get("exactMap") is not True
            or sample.get("authorized") is not False
            or sample.get("cloudFresh") is not True
            or sample.get("stopReason")
            not in {"runtime_motion_not_authorized", "runtime_authorization_timeout"}
        ):
            errors.append("safety_status_contract")

    traces = _samples(evidence, "/go2/runtime/trace_status")
    for sample in traces:
        binding = sample.get("binding") if isinstance(sample, Mapping) else None
        if (
            not isinstance(sample, Mapping)
            or sample.get("schema") != "go2.runtime_trace_status.v1"
            or sample.get("writable") is not True
            or not isinstance(sample.get("path"), str)
            or not sample.get("path")
            or not isinstance(sample.get("recordsWritten"), int)
            or sample.get("recordsWritten", 0) <= 0
            or not _finite(sample.get("lastFsyncAt"))
            or not _finite(sample.get("lastFsyncAgeS"))
            or float(sample.get("lastFsyncAgeS")) > 2.0
            or not isinstance(binding, Mapping)
            or binding.get("mapVersion") != expected_map_version
            or binding.get("manifestHash") != expected_manifest_hash
        ):
            errors.append("runtime_trace_not_writable")

    if any(
        sample is not False
        for sample in _samples(evidence, "/go2/runtime/motion_authorized")
    ):
        errors.append("motion_authorization_nonzero")

    for sample in _samples(evidence, "/cmd_vel"):
        values = sample.get("values", []) if isinstance(sample, Mapping) else []
        if (
            not isinstance(values, list)
            or len(values) != 3
            or not all(_finite(value) for value in values)
            or any(abs(float(value)) > 1.0e-6 for value in values)
        ):
            errors.append("cmd_vel_nonzero")

    for sample in _samples(evidence, "/navigation/cloud_body"):
        if not isinstance(sample, Mapping) or sample.get("frameId") != "base_link":
            errors.append("body_cloud_frame")
    for sample in _samples(evidence, "/Odometry"):
        if (
            not isinstance(sample, Mapping)
            or sample.get("frameId") != "odom"
            or sample.get("childFrameId") != "base_link"
        ):
            errors.append("odometry_frame")

    for sample in _samples(evidence, "/lf/sportmodestate"):
        values = sample.get("velocity", []) if isinstance(sample, Mapping) else []
        progress = sample.get("progress") if isinstance(sample, Mapping) else None
        if (
            not isinstance(sample, Mapping)
            or sample.get("errorCode") not in NORMAL_UNITREE_SPORT_STATUS_CODES
            or sample.get("mode") not in {0, 1, 3}
            or not _finite(progress)
            or float(progress) > 0.01
            or not isinstance(values, list)
            or len(values) != 3
            or not all(_finite(value) for value in values)
            or math.hypot(float(values[0]), float(values[1])) > 0.08
            or abs(float(values[2])) > 0.12
        ):
            errors.append("robot_not_safe_stationary")

    runtime_files = evidence.get("runtimeFiles", {})
    performance = (
        runtime_files.get("performance", {})
        if isinstance(runtime_files, Mapping)
        else {}
    )
    performance_sample = (
        performance.get("latestSample")
        if isinstance(performance, Mapping)
        else None
    )
    if (
        not isinstance(performance, Mapping)
        or not _positive_integer(performance.get("size"))
        or not _finite(performance.get("mtimeAgeS"))
        or float(performance.get("mtimeAgeS")) > 5.0
        or not isinstance(performance_sample, Mapping)
        or not _finite(performance_sample.get("monotonic_s"))
    ):
        errors.append("performance_trace_not_fresh")
    sdk = (
        runtime_files.get("sdkEvents", {})
        if isinstance(runtime_files, Mapping)
        else {}
    )
    startup_stop = sdk.get("startupStop") if isinstance(sdk, Mapping) else None
    latest_event = sdk.get("latestEvent") if isinstance(sdk, Mapping) else None
    if (
        not isinstance(sdk, Mapping)
        or not _positive_integer(sdk.get("size"))
        or not _finite(sdk.get("mtimeAgeS"))
        or float(sdk.get("mtimeAgeS")) > 5.0
        or not isinstance(startup_stop, Mapping)
        or startup_stop.get("schema") != "go2.sdk_motion_event.v1"
        or startup_stop.get("returnCode") != 0
        or not isinstance(latest_event, Mapping)
        or latest_event.get("schema") != "go2.sdk_motion_event.v1"
    ):
        errors.append("sdk_event_trace_not_fresh")

    unique_errors = sorted(set(errors))
    return {
        "passed": not unique_errors,
        "errors": unique_errors,
        "runtimeInstanceId": runtime_instance,
        "runtimeState": (
            runtime[-1].get("state")
            if runtime and isinstance(runtime[-1], Mapping)
            else None
        ),
        "localizationTracking": localization_tracking,
        "localizationAccuracyVerified": False,
        "motionCommandsAllowed": False,
    }
