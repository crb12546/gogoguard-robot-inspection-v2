from __future__ import annotations

import hashlib
import json
from datetime import datetime
from pathlib import Path
from typing import Any


EVIDENCE_TRANSACTION_VERSION = 1
EVIDENCE_CONTRACT_SCHEMA = "gogoguard.evidence_txn.v1"
EVIDENCE_MANIFEST_SHA256 = (
    "58230fa7782fb5c4921a5ba730eb359f06b56b57eff57494732caac341281eb0"
)

REJECTION_CODES = frozenset(
    {
        "UNSUPPORTED_CONTRACT_VERSION",
        "UNSUPPORTED_CONTRACT_VALUE",
        "INVALID_CONTRACT",
        "CONTRACT_HASH_MISMATCH",
        "IDEMPOTENCY_KEY_REUSED",
        "EVIDENCE_BINDING_NOT_ALLOWED",
    }
)
REJECTION_CLASSES = frozenset({"contract", "identity", "binding"})
FAILURE_CODES = frozenset(
    {
        "CAPTURE_NO_FRESH_FRAME",
        "CAPTURE_TRANSPORT_UNAVAILABLE",
        "CAPTURE_UPSTREAM_ERROR",
        "CAPTURE_OUTCOME_UNKNOWN",
        "CAPTURE_INFRA_FAILURE",
        "RECEIPT_CAPTURE_ID_MISMATCH",
        "RECEIPT_STALE",
        "RECEIPT_BINDING_MISMATCH",
        "RECEIPT_SCHEMA_INVALID",
    }
)
EVIDENCE_ORIGINS = frozenset(
    {"evidence_ingress", "legacy_segment_nearest_landmark"}
)
EVIDENCE_SCOPES = frozenset({"checkpoint_verdict", "along_route_anomaly"})
ALLOWED_EVIDENCE_BINDINGS = frozenset(
    {
        ("evidence_ingress", "checkpoint_verdict"),
        ("evidence_ingress", "along_route_anomaly"),
        ("legacy_segment_nearest_landmark", "along_route_anomaly"),
    }
)
CAPTURE_STATES = frozenset({"selecting", "processing", "succeeded", "failed"})
SELECTION_WINDOW_MS = 3000
CAPTURE_RESPONSE_FIELDS = frozenset(
    {
        "accepted",
        "captureRequestId",
        "contract",
        "rejection",
        "state",
        "receipt",
        "failureCode",
        "retryAfterMs",
    }
)
CONTRACT_REF_FIELDS = frozenset({"schema", "version", "manifestSha256"})
REJECTION_FIELDS = frozenset(
    {"class", "code", "retryable", "path", "observed", "supported"}
)
RECEIPT_FIELDS = frozenset(
    {
        "captureRequestId",
        "captureId",
        "frameId",
        "ingressAcceptedAt",
        "frameReceivedAt",
        "sourceCapturedAt",
        "backendReceivedAt",
        "streamInstanceId",
        "frameSequence",
        "participant",
        "track",
        "width",
        "height",
        "mime",
        "bytes",
        "sha256",
        "missionId",
        "patrolId",
        "checkpointId",
        "attempt",
        "mapVersion",
        "routeId",
        "evidenceOrigin",
        "evidenceScope",
    }
)

ARTIFACT_FILES = {
    "contract.schema.json": (
        13775,
        "c6f362ad1bafe4df8b894e665bd6de6056e3cbf3dcc78b369a9ec0387fa512a0",
    ),
    "vectors/vectors.json": (
        9742,
        "251756e38745b1984930942771e9c3a6da66d937340213272cbd598208c9547b",
    ),
    "manifest.json": (
        1278,
        EVIDENCE_MANIFEST_SHA256,
    ),
}


class EvidenceContractError(ValueError):
    def __init__(self, code: str, message: str, *, layer: str = "admission") -> None:
        super().__init__(message)
        self.code = code
        self.layer = layer


class ReceiptValidationError(EvidenceContractError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(code, message, layer="business")


def artifact_root() -> Path:
    return Path(__file__).with_name("evidence_txn") / "v1"


def verify_evidence_artifact(root: Path | None = None) -> bool:
    root = artifact_root() if root is None else Path(root)
    try:
        for relative, (expected_bytes, expected_sha256) in ARTIFACT_FILES.items():
            raw = (root / relative).read_bytes()
            if len(raw) != expected_bytes:
                return False
            if hashlib.sha256(raw).hexdigest() != expected_sha256:
                return False
    except OSError:
        return False
    return True


def evidence_transaction_capability(*, enabled: bool) -> dict[str, Any]:
    supported = bool(enabled and verify_evidence_artifact())
    return {
        "supported": supported,
        "versions": [EVIDENCE_TRANSACTION_VERSION] if supported else [],
        "manifestSha256": EVIDENCE_MANIFEST_SHA256,
    }


def contract_ref() -> dict[str, Any]:
    return {
        "schema": EVIDENCE_CONTRACT_SCHEMA,
        "version": EVIDENCE_TRANSACTION_VERSION,
        "manifestSha256": EVIDENCE_MANIFEST_SHA256,
    }


def evidence_binding_allowed(origin: Any, scope: Any) -> bool:
    if not isinstance(origin, str) or not isinstance(scope, str):
        return False
    if origin not in EVIDENCE_ORIGINS or scope not in EVIDENCE_SCOPES:
        return False
    return (origin, scope) in ALLOWED_EVIDENCE_BINDINGS


def capture_request_id(
    *,
    mission_id: str,
    map_version: str,
    route_id: str,
    checkpoint_id: str,
    attempt: int,
) -> str:
    """Return the bilateral v1 idempotency key for one capture attempt.

    The v1 artifact names the hashed fields but does not define their byte
    serialization.  Dog and platform therefore pin this supplemental rule:
    a compact UTF-8 JSON array in the declared field order, non-ASCII left
    unescaped, empty strings retained, and ``attempt`` encoded as a JSON
    number.  Do not change this without a new cross-end contract version.
    """
    identity = (mission_id, map_version, route_id, checkpoint_id)
    if any(not isinstance(value, str) for value in identity):
        raise TypeError("capture request identity fields must be strings")
    if isinstance(attempt, bool) or not isinstance(attempt, int):
        raise TypeError("capture request attempt must be an integer")
    canonical = json.dumps(
        [mission_id, map_version, route_id, checkpoint_id, attempt],
        ensure_ascii=False,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return "cr_" + hashlib.sha256(canonical).hexdigest()


def capture_request(
    *,
    mission_id: str,
    map_version: str,
    route_id: str,
    checkpoint_id: str,
    attempt: int,
) -> dict[str, Any]:
    return {
        "captureRequestId": capture_request_id(
            mission_id=mission_id,
            map_version=map_version,
            route_id=route_id,
            checkpoint_id=checkpoint_id,
            attempt=attempt,
        ),
        "contract": contract_ref(),
        "evidenceOrigin": "evidence_ingress",
        "evidenceScope": "checkpoint_verdict",
    }


def _aware_datetime(value: Any, name: str) -> datetime:
    if not isinstance(value, str) or not value:
        raise ReceiptValidationError("RECEIPT_SCHEMA_INVALID", f"{name} is invalid")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ReceiptValidationError(
            "RECEIPT_SCHEMA_INVALID", f"{name} is invalid"
        ) from exc
    if parsed.tzinfo is None:
        raise ReceiptValidationError(
            "RECEIPT_SCHEMA_INVALID", f"{name} must include a timezone"
        )
    return parsed


def _nonempty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value)


def validate_receipt(
    receipt: Any,
    *,
    expected_robot_id: str,
    expected_capture_request_id: str,
    expected_mission_id: str,
    expected_checkpoint_id: str,
    expected_attempt: int,
    expected_map_version: str,
    expected_route_id: str,
    previous_capture_id: str = "",
) -> dict[str, Any]:
    if not isinstance(receipt, dict):
        raise ReceiptValidationError("RECEIPT_SCHEMA_INVALID", "receipt is not an object")
    if not set(receipt).issubset(RECEIPT_FIELDS):
        raise ReceiptValidationError(
            "RECEIPT_SCHEMA_INVALID", "receipt has an unknown field"
        )
    required_strings = (
        "captureRequestId",
        "captureId",
        "frameId",
        "ingressAcceptedAt",
        "frameReceivedAt",
        "streamInstanceId",
        "participant",
        "track",
        "mime",
        "sha256",
        "missionId",
        "checkpointId",
        "mapVersion",
        "routeId",
        "evidenceOrigin",
        "evidenceScope",
    )
    if any(not _nonempty_string(receipt.get(name)) for name in required_strings):
        raise ReceiptValidationError(
            "RECEIPT_SCHEMA_INVALID", "receipt has a missing string field"
        )
    for name in ("frameSequence", "width", "height", "bytes", "attempt"):
        value = receipt.get(name)
        minimum = 0 if name == "frameSequence" else 1
        if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
            raise ReceiptValidationError(
                "RECEIPT_SCHEMA_INVALID", f"receipt {name} is invalid"
            )
    digest = str(receipt["sha256"])
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise ReceiptValidationError(
            "RECEIPT_SCHEMA_INVALID", "receipt sha256 is invalid"
        )
    if receipt["captureRequestId"] != expected_capture_request_id:
        raise ReceiptValidationError(
            "RECEIPT_BINDING_MISMATCH", "receipt captureRequestId differs"
        )
    capture_id = str(receipt["captureId"])
    if previous_capture_id and capture_id != previous_capture_id:
        raise ReceiptValidationError(
            "RECEIPT_CAPTURE_ID_MISMATCH", "receipt captureId differs"
        )
    expected_bindings = {
        "missionId": expected_mission_id,
        "checkpointId": expected_checkpoint_id,
        "attempt": int(expected_attempt),
        "mapVersion": expected_map_version,
        "routeId": expected_route_id,
    }
    if any(receipt.get(name) != value for name, value in expected_bindings.items()):
        raise ReceiptValidationError(
            "RECEIPT_BINDING_MISMATCH", "receipt mission binding differs"
        )
    if receipt["participant"] != f"robot:{expected_robot_id}":
        raise ReceiptValidationError(
            "RECEIPT_BINDING_MISMATCH", "receipt participant differs"
        )
    if not evidence_binding_allowed(
        receipt.get("evidenceOrigin"), receipt.get("evidenceScope")
    ) or (
        receipt.get("evidenceOrigin"), receipt.get("evidenceScope")
    ) != ("evidence_ingress", "checkpoint_verdict"):
        raise ReceiptValidationError(
            "RECEIPT_BINDING_MISMATCH", "receipt evidence binding is not allowed"
        )
    ingress = _aware_datetime(receipt["ingressAcceptedAt"], "ingressAcceptedAt")
    frame = _aware_datetime(receipt["frameReceivedAt"], "frameReceivedAt")
    elapsed_ms = (frame - ingress).total_seconds() * 1000.0
    if elapsed_ms < 0 or elapsed_ms > SELECTION_WINDOW_MS:
        raise ReceiptValidationError("RECEIPT_STALE", "receipt frame is outside window")
    for optional_time in ("sourceCapturedAt", "backendReceivedAt"):
        value = receipt.get(optional_time)
        if value is not None:
            _aware_datetime(value, optional_time)
    if "patrolId" in receipt:
        patrol_id = receipt["patrolId"]
        if patrol_id is not None and (
            isinstance(patrol_id, bool) or not isinstance(patrol_id, int)
        ):
            raise ReceiptValidationError(
                "RECEIPT_SCHEMA_INVALID", "receipt patrolId is invalid"
            )
    return dict(receipt)


def extract_capture_response(value: Any) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise EvidenceContractError("INVALID_CONTRACT", "capture response is invalid")
    response = value.get("captureResponse")
    if not isinstance(response, dict):
        raise EvidenceContractError(
            "INVALID_CONTRACT", "captureResponse wrapper is missing"
        )
    return dict(response)


def validate_capture_response_envelope(value: Any) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise EvidenceContractError("INVALID_CONTRACT", "capture response is invalid")
    response = dict(value)
    if not set(response).issubset(CAPTURE_RESPONSE_FIELDS):
        raise EvidenceContractError("INVALID_CONTRACT", "capture response has an unknown field")
    contract = response.get("contract")
    if not isinstance(contract, dict):
        raise EvidenceContractError("INVALID_CONTRACT", "contract reference is missing")
    if set(contract) != CONTRACT_REF_FIELDS:
        raise EvidenceContractError("INVALID_CONTRACT", "contract reference shape is invalid")
    version = contract.get("version")
    if isinstance(version, bool) or not isinstance(version, int):
        raise EvidenceContractError("INVALID_CONTRACT", "contract version is invalid")
    if version != EVIDENCE_TRANSACTION_VERSION:
        raise EvidenceContractError(
            "UNSUPPORTED_CONTRACT_VERSION", "contract version is unsupported"
        )
    if contract.get("schema") != EVIDENCE_CONTRACT_SCHEMA:
        raise EvidenceContractError("INVALID_CONTRACT", "contract schema is invalid")
    manifest_hash = contract.get("manifestSha256")
    if manifest_hash != EVIDENCE_MANIFEST_SHA256:
        raise EvidenceContractError(
            "CONTRACT_HASH_MISMATCH", "contract manifest hash differs"
        )
    accepted = response.get("accepted")
    if not isinstance(accepted, bool) or not _nonempty_string(
        response.get("captureRequestId")
    ):
        raise EvidenceContractError("INVALID_CONTRACT", "capture response is incomplete")
    forbidden = ("state", "failureCode", "receipt")
    if accepted is False:
        rejection = response.get("rejection")
        if not isinstance(rejection, dict) or any(name in response for name in forbidden):
            raise EvidenceContractError("INVALID_CONTRACT", "rejection shape is invalid")
        if not set(rejection).issubset(REJECTION_FIELDS):
            raise EvidenceContractError("INVALID_CONTRACT", "rejection has an unknown field")
        if (
            rejection.get("class") not in REJECTION_CLASSES
            or rejection.get("code") not in REJECTION_CODES
            or not isinstance(rejection.get("retryable"), bool)
        ):
            raise EvidenceContractError("INVALID_CONTRACT", "rejection value is invalid")
        return response
    if "rejection" in response or response.get("state") not in CAPTURE_STATES:
        raise EvidenceContractError("INVALID_CONTRACT", "business response is invalid")
    state = response["state"]
    if state == "failed":
        if response.get("failureCode") not in FAILURE_CODES or "receipt" in response:
            raise EvidenceContractError("INVALID_CONTRACT", "failure response is invalid")
    elif "failureCode" in response:
        raise EvidenceContractError("INVALID_CONTRACT", "failure code is out of state")
    if state == "succeeded":
        if "receipt" not in response:
            raise EvidenceContractError("INVALID_CONTRACT", "receipt is missing")
    elif "receipt" in response:
        raise EvidenceContractError("INVALID_CONTRACT", "receipt is out of state")
    retry_after = response.get("retryAfterMs")
    if retry_after is not None and (
        isinstance(retry_after, bool)
        or not isinstance(retry_after, int)
        or retry_after < 0
    ):
        raise EvidenceContractError("INVALID_CONTRACT", "retryAfterMs is invalid")
    return response
