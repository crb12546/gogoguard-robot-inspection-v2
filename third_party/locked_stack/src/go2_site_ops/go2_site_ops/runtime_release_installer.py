#!/usr/bin/env python3
"""Install and activate one verified runtime release on the robot."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import stat
import subprocess
import time
from pathlib import Path

from .map_store import ACTIVATION_GATES, MapStoreError, MapVersionStore
from .static_runtime_core import evaluate_static_runtime


STATIC_RECEIPT_SCHEMA = "go2.static_runtime_commissioning.v1"
STATIC_RECEIPT_ID = re.compile(r"^static-runtime-[0-9a-f]{32}$")


def _canonical_static_receipt(receipt) -> bytes:
    return (
        json.dumps(
            receipt,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def _static_receipt_from_output(output: str):
    for line in reversed(str(output).splitlines()):
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            wrapper = json.loads(line)
        except ValueError:
            continue
        if (
            isinstance(wrapper, dict)
            and isinstance(wrapper.get("receipt"), dict)
            and wrapper["receipt"].get("schema") == STATIC_RECEIPT_SCHEMA
            and isinstance(wrapper.get("artifact"), dict)
        ):
            return wrapper
    raise RuntimeError("static commissioning did not return a receipt")


def _validate_static_commissioning(
    wrapper,
    *,
    site_id: str,
    version_id: str,
    manifest_hash: str,
    runtime_instance_id: str,
    runtime_observed_at: float,
    acceptance_dir: Path,
    operator=None,
):
    receipt = wrapper["receipt"]
    artifact = wrapper["artifact"]
    try:
        payload = _canonical_static_receipt(receipt)
    except (TypeError, ValueError) as exc:
        raise RuntimeError("static commissioning receipt is not canonical JSON") from exc
    digest = hashlib.sha256(payload).hexdigest()
    evaluation = receipt.get("evaluation")
    evidence = receipt.get("evidence")
    if not isinstance(evidence, dict):
        raise RuntimeError("static commissioning receipt has no raw evidence")
    recomputed_evaluation = evaluate_static_runtime(
        evidence,
        expected_map_version=version_id,
        expected_manifest_hash=manifest_hash,
    )
    try:
        started_at = float(receipt.get("startedAt"))
        completed_at = float(receipt.get("completedAt"))
    except (TypeError, ValueError):
        started_at = 0.0
        completed_at = 0.0
    if (
        not STATIC_RECEIPT_ID.fullmatch(str(receipt.get("receiptId", "")))
        or (operator is not None and receipt.get("operator") != operator)
        or receipt.get("siteId") != site_id
        or receipt.get("expectedMapVersion") != version_id
        or receipt.get("expectedManifestHash") != manifest_hash
        or receipt.get("readOnlyObservation") is not True
        or receipt.get("serviceCallsIssued") is not False
        or receipt.get("messagesPublished") is not False
        or receipt.get("outcome") != "passed"
        or not isinstance(evaluation, dict)
        or evaluation != recomputed_evaluation
        or evaluation.get("passed") is not True
        or evaluation.get("errors") != []
        or evaluation.get("runtimeInstanceId") != runtime_instance_id
        or evaluation.get("localizationAccuracyVerified") is not False
        or evaluation.get("motionCommandsAllowed") is not False
        or started_at < runtime_observed_at - 2.0
        or completed_at < started_at
        or artifact.get("sha256") != digest
    ):
        raise RuntimeError("static commissioning receipt failed its release binding")
    receipt_path = Path(str(artifact.get("receiptPath", "")))
    sha_path = Path(str(artifact.get("sha256Path", "")))
    requested_acceptance = Path(acceptance_dir)
    if requested_acceptance.is_symlink():
        raise RuntimeError("static commissioning directory cannot be a symlink")
    acceptance_root = requested_acceptance.resolve()
    expected_name = str(receipt["receiptId"]) + ".json"
    if (
        not receipt_path.is_absolute()
        or receipt_path.is_symlink()
        or not receipt_path.is_file()
        or not sha_path.is_absolute()
        or sha_path.is_symlink()
        or not sha_path.is_file()
        or receipt_path.parent.resolve() != acceptance_root
        or sha_path.parent.resolve() != acceptance_root
        or receipt_path.name != expected_name
        or sha_path.name != expected_name + ".sha256"
    ):
        raise RuntimeError("static commissioning artifacts are unavailable")
    for path in (receipt_path, sha_path):
        if not stat.S_ISREG(path.stat().st_mode) or path.stat().st_mode & 0o222:
            raise RuntimeError("static commissioning artifacts must be read-only files")
    if receipt_path.read_bytes() != payload:
        raise RuntimeError("static commissioning receipt file differs from stdout")
    expected_sidecar = "%s  %s\n" % (digest, receipt_path.name)
    if sha_path.read_text(encoding="ascii") != expected_sidecar:
        raise RuntimeError("static commissioning SHA-256 sidecar is invalid")
    return {
        "required": True,
        "passed": True,
        "receiptId": receipt.get("receiptId"),
        "receiptSha256": digest,
        "receiptPath": str(receipt_path),
        "sha256Path": str(sha_path),
        "runtimeInstanceId": evaluation.get("runtimeInstanceId"),
        "runtimeState": evaluation.get("runtimeState"),
        "localizationTracking": evaluation.get("localizationTracking"),
        "localizationAccuracyVerified": False,
        "motionCommandsAllowed": False,
        "receipt": receipt,
    }


def _run_static_commissioning(
    *,
    map_root: Path,
    acceptance_dir: Path,
    site_id: str,
    version_id: str,
    manifest_hash: str,
    operator: str,
    runtime_instance_id: str,
    runtime_observed_at: float,
    timeout_s: float,
):
    command = [
        "ros2",
        "run",
        "go2_validation_runtime",
        "static_runtime_commissioning",
        "--map-root",
        str(map_root),
        "--site-id",
        site_id,
        "--operator",
        operator,
        "--output-dir",
        str(acceptance_dir),
        "--timeout-s",
        str(timeout_s),
    ]
    completed = subprocess.run(
        command,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout_s + 30.0,
    )
    wrapper = _static_receipt_from_output(completed.stdout)
    result = _validate_static_commissioning(
        wrapper,
        site_id=site_id,
        version_id=version_id,
        manifest_hash=manifest_hash,
        runtime_instance_id=runtime_instance_id,
        runtime_observed_at=runtime_observed_at,
        acceptance_dir=acceptance_dir,
        operator=operator,
    )
    if completed.returncode != 0:
        raise RuntimeError("static commissioning command reported failure")
    return result


def _wait_for_runtime(
    *,
    topic: str,
    version_id: str,
    manifest_hash: str,
    timeout_s: float,
):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    observed = {"payload": None, "at": None}
    wait_started_at = time.time()
    rclpy.init()
    node = Node("runtime_release_install_verifier")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    def callback(message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        try:
            published_at = float(payload.get("publishedAt"))
            status_sequence = int(payload.get("statusSequence"))
        except (AttributeError, TypeError, ValueError):
            return
        if (
            isinstance(payload, dict)
            and payload.get("schema") == "go2.runtime_status.v1"
            and isinstance(payload.get("runtimeInstanceId"), str)
            and len(payload["runtimeInstanceId"]) == 32
            and status_sequence > 0
            and published_at >= wait_started_at - 2.0
            and abs(time.time() - published_at) <= 5.0
            and payload.get("mapVersion") == version_id
            and payload.get("manifestHash") == manifest_hash
            and payload.get("motionAuthorized") is False
            and payload.get("state") in {"READY", "POSITIONING", "COMPLETED"}
        ):
            observed["payload"] = payload
            observed["at"] = time.time()

    node.create_subscription(String, topic, callback, qos)
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and time.monotonic() < deadline and observed["payload"] is None:
            rclpy.spin_once(node, timeout_sec=min(0.5, max(0.0, deadline - time.monotonic())))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return observed["payload"], observed["at"]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--map-root", type=Path, required=True)
    parser.add_argument("--site-id", required=True)
    parser.add_argument("--version-id", required=True)
    parser.add_argument("--manifest-hash", required=True)
    parser.add_argument("--approved-by", required=True)
    parser.add_argument("--robot-id", default=os.environ.get("GO2_ROBOT_ID", ""))
    parser.add_argument(
        "--sensor-id", default=os.environ.get("GO2_LIDAR_SENSOR_ID", "")
    )
    parser.add_argument("--consume-source", action="store_true")
    parser.add_argument("--runtime-status-topic", default="/go2/runtime/status")
    parser.add_argument("--runtime-ready-timeout-s", type=float, default=180.0)
    parser.add_argument(
        "--static-commissioning-timeout-s", type=float, default=30.0
    )
    parser.add_argument(
        "--acceptance-dir", type=Path, default=Path("/data/go2/acceptance")
    )
    args = parser.parse_args(argv)
    if not 10.0 <= args.runtime_ready_timeout_s <= 900.0:
        parser.error("runtime ready timeout must be between 10 and 900 seconds")
    if not 10.0 <= args.static_commissioning_timeout_s <= 120.0:
        parser.error("static commissioning timeout must be between 10 and 120 seconds")
    if not args.acceptance_dir.is_absolute():
        parser.error("acceptance directory must be absolute")

    store = MapVersionStore(args.map_root)
    incoming_root = (store.root / ".incoming").resolve()
    source = args.source.resolve()
    if source.parent != incoming_root or not source.name.startswith(args.version_id + "."):
        parser.error("source must be one version-specific directory under map-root/.incoming")
    previous_activation = store.active(args.site_id)
    activation_changed = False
    try:
        release = store.install_release_version(
            source,
            expected_site_id=args.site_id,
            expected_version_id=args.version_id,
            expected_manifest_hash=args.manifest_hash,
            expected_robot_id=args.robot_id,
            expected_sensor_id=args.sensor_id,
        )
        activation = store.active(args.site_id)
        if activation is not None and activation.get("versionId") == args.version_id:
            if (
                activation.get("manifestHash") != args.manifest_hash
                or activation.get("approvedBy") != args.approved_by
            ):
                raise MapStoreError(
                    "release is already active with a different approval identity"
                )
        else:
            activation = store.activate(
                site_id=args.site_id,
                version_id=args.version_id,
                approved_by=args.approved_by,
                gates={name: True for name in ACTIVATION_GATES},
            )
            activation_changed = True
    except MapStoreError as exc:
        parser.exit(2, "runtime release rejected: %s\n" % exc)

    if args.consume_source:
        # This path was constrained above to a unique direct child of
        # .incoming and is removed only after import and activation succeeded.
        MapVersionStore._make_writable(source)
        shutil.rmtree(str(source))
    runtime_status, observed_at = _wait_for_runtime(
        topic=args.runtime_status_topic,
        version_id=release["versionId"],
        manifest_hash=release["manifestHash"],
        timeout_s=args.runtime_ready_timeout_s,
    )
    static_commissioning = None
    failure_reason = None
    if runtime_status is None:
        failure_reason = "NEW_RUNTIME_STATUS_TIMEOUT"
    else:
        try:
            static_commissioning = _run_static_commissioning(
                map_root=store.root,
                acceptance_dir=args.acceptance_dir,
                site_id=release["siteId"],
                version_id=release["versionId"],
                manifest_hash=release["manifestHash"],
                operator=args.approved_by,
                runtime_instance_id=str(runtime_status["runtimeInstanceId"]),
                runtime_observed_at=float(observed_at),
                timeout_s=args.static_commissioning_timeout_s,
            )
        except Exception as exc:
            static_commissioning = {
                "required": True,
                "passed": False,
                "error": str(exc),
            }
            failure_reason = "STATIC_RUNTIME_COMMISSIONING_FAILED"
    rollback = None
    if failure_reason is not None and activation_changed:
        try:
            rollback_record = store.rollback_activation(
                site_id=args.site_id,
                failed_activation=activation,
                previous_activation=previous_activation,
                rolled_back_by=args.approved_by,
                reason=failure_reason,
            )
            restored_status = None
            restored_observed_at = None
            if previous_activation is not None:
                restored_status, restored_observed_at = _wait_for_runtime(
                    topic=args.runtime_status_topic,
                    version_id=str(previous_activation["versionId"]),
                    manifest_hash=str(previous_activation["manifestHash"]),
                    timeout_s=args.runtime_ready_timeout_s,
                )
            rollback = {
                "succeeded": True,
                "record": rollback_record,
                "restoredRuntimeReady": (
                    restored_status is not None
                    if previous_activation is not None
                    else None
                ),
                "restoredRuntimeState": (
                    restored_status.get("state") if restored_status else None
                ),
                "restoredRuntimeObservedAt": restored_observed_at,
            }
        except Exception as exc:  # preserve the failed release receipt
            rollback = {"succeeded": False, "error": str(exc)}
    receipt = {
        "schema": "go2.robot_release_install.v1",
        "siteId": release["siteId"],
        "versionId": release["versionId"],
        "manifestHash": release["manifestHash"],
        "robotId": args.robot_id,
        "sensorId": args.sensor_id,
        "approvedBy": activation["approvedBy"],
        "activatedAt": activation["activatedAt"],
        "activePointerUpdated": failure_reason is None,
        "runtimeReady": runtime_status is not None,
        "runtimeState": runtime_status.get("state") if runtime_status else None,
        "runtimeReason": runtime_status.get("reason") if runtime_status else "STATUS_TIMEOUT",
        "runtimeInstanceId": (
            runtime_status.get("runtimeInstanceId") if runtime_status else None
        ),
        "runtimeStatusSequence": (
            runtime_status.get("statusSequence") if runtime_status else None
        ),
        "runtimeObservedAt": observed_at,
        "motionAuthorized": (
            runtime_status.get("motionAuthorized") if runtime_status else False
        ),
        "staticCommissioning": static_commissioning,
        "rollback": rollback,
    }
    print(json.dumps(receipt, ensure_ascii=False, sort_keys=True))
    if failure_reason is None:
        return 0
    return 4 if rollback is not None and rollback.get("succeeded") is True else 5


if __name__ == "__main__":
    raise SystemExit(main())
