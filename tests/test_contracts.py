import json
import hashlib
import unittest
from itertools import product
from pathlib import Path

from gogoguard_contracts import (
    CameraStreamStatus,
    DeviceStatus,
    MapJob,
    RecordingSession,
    is_safe_external_id,
    jpeg_metadata,
    json_ready,
    validate_platform_mission_message,
    ALLOWED_EVIDENCE_BINDINGS,
    CAPTURE_STATES,
    EVIDENCE_ORIGINS,
    EVIDENCE_SCOPES,
    FAILURE_CODES,
    REJECTION_CLASSES,
    REJECTION_CODES,
    EvidenceContractError,
    capture_request_id,
    contract_ref,
    evidence_binding_allowed,
    extract_capture_response,
    validate_capture_response_envelope,
    verify_evidence_artifact,
)


class ContractsTest(unittest.TestCase):
    def test_evidence_artifact_is_byte_exact_and_manifest_is_truthful(self) -> None:
        root = (
            Path(__file__).resolve().parents[1]
            / "modules/contracts/gogoguard_contracts/evidence_txn/v1"
        )
        self.assertTrue(verify_evidence_artifact(root))
        manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
        for item in manifest["files"]:
            raw = (root / item["path"]).read_bytes()
            self.assertEqual(len(raw), item["bytes"])
            self.assertEqual(hashlib.sha256(raw).hexdigest(), item["sha256"])

    def test_capture_request_id_matches_bilateral_v1_golden_value(self) -> None:
        request_id = capture_request_id(
            mission_id="mission-20260823-053000-abc123",
            map_version="map-62d8cec9a1dc",
            route_id="route-62d8cec9a1dc-workspace-r5",
            checkpoint_id="cp_01",
            attempt=1,
        )
        self.assertEqual(
            request_id,
            "cr_106562dc452434f2ab1a8dca9eb11fc9aac2b9e1dca2f811203847ac4e3355cf",
        )
        string_attempt = hashlib.sha256(
            b'["mission-20260823-053000-abc123","map-62d8cec9a1dc",'
            b'"route-62d8cec9a1dc-workspace-r5","cp_01","1"]'
        ).hexdigest()
        self.assertNotEqual(request_id, "cr_" + string_attempt)

    def test_capture_request_id_retains_empty_string_slots(self) -> None:
        with_empty_slot = capture_request_id(
            mission_id="mission-1",
            map_version="",
            route_id="route-1",
            checkpoint_id="cp_01",
            attempt=1,
        )
        skipped_slot = hashlib.sha256(
            b'["mission-1","route-1","cp_01",1]'
        ).hexdigest()
        self.assertNotEqual(with_empty_slot, "cr_" + skipped_slot)
        with self.assertRaises(TypeError):
            capture_request_id(
                mission_id="mission-1",
                map_version="map-1",
                route_id="route-1",
                checkpoint_id="cp_01",
                attempt=True,
            )

    def test_evidence_enums_are_exactly_equal_to_schema(self) -> None:
        root = (
            Path(__file__).resolve().parents[1]
            / "modules/contracts/gogoguard_contracts/evidence_txn/v1"
        )
        schema = json.loads((root / "contract.schema.json").read_text(encoding="utf-8"))
        definitions = schema["$defs"]
        self.assertEqual(REJECTION_CODES, frozenset(definitions["rejectionCode"]["enum"]))
        self.assertEqual(REJECTION_CLASSES, frozenset(definitions["rejectionClass"]["enum"]))
        self.assertEqual(FAILURE_CODES, frozenset(definitions["failureCode"]["enum"]))
        self.assertEqual(EVIDENCE_ORIGINS, frozenset(definitions["evidenceOrigin"]["enum"]))
        self.assertEqual(EVIDENCE_SCOPES, frozenset(definitions["evidenceScope"]["enum"]))
        self.assertEqual(CAPTURE_STATES, frozenset(definitions["captureState"]["enum"]))
        self.assertFalse(REJECTION_CODES & FAILURE_CODES)

    def test_all_evidence_binding_pairs_are_explicit_and_fail_closed(self) -> None:
        classified = {
            (origin, scope): evidence_binding_allowed(origin, scope)
            for origin, scope in product(EVIDENCE_ORIGINS, EVIDENCE_SCOPES)
        }
        self.assertEqual(
            {pair for pair, allowed in classified.items() if allowed},
            ALLOWED_EVIDENCE_BINDINGS,
        )
        self.assertFalse(evidence_binding_allowed("future", "checkpoint_verdict"))
        self.assertFalse(evidence_binding_allowed("evidence_ingress", "future"))

    def test_evidence_golden_admission_and_mutual_exclusion_vectors(self) -> None:
        root = (
            Path(__file__).resolve().parents[1]
            / "modules/contracts/gogoguard_contracts/evidence_txn/v1"
        )
        vectors = json.loads(
            (root / "vectors/vectors.json").read_text(encoding="utf-8")
        )
        self.assertEqual(
            {item["id"] for item in vectors["admission_rejections"]},
            {"R-01", "R-02", "R-03", "R-04", "R-05"},
        )
        self.assertEqual(
            {item["id"] for item in vectors["mutual_exclusion"]},
            {"X-01", "X-02", "X-03", "X-04", "X-05", "X-06"},
        )
        self.assertEqual(
            {
                item["expect"]["failureCode"]
                for item in vectors["business_outcomes"]
                if "failureCode" in item["expect"]
            },
            FAILURE_CODES,
        )
        self.assertEqual(
            {item["id"] for item in vectors["risk_vectors"]},
            {f"V-{index:02d}" for index in range(1, 11)},
        )
        request_id = "cr_test"
        base = {
            "accepted": True,
            "captureRequestId": request_id,
            "contract": contract_ref(),
            "state": "selecting",
        }
        self.assertEqual(validate_capture_response_envelope(base), base)
        self.assertEqual(
            extract_capture_response({"captureResponse": base}),
            base,
        )
        for invalid_outer in (base, {"capture": base}, {}):
            with self.assertRaises(EvidenceContractError):
                extract_capture_response(invalid_outer)
        invalid = dict(base)
        invalid["contract"] = {**contract_ref(), "version": 2}
        with self.assertRaises(EvidenceContractError) as caught:
            validate_capture_response_envelope(invalid)
        self.assertEqual(caught.exception.code, "UNSUPPORTED_CONTRACT_VERSION")
        invalid = dict(base)
        invalid["contract"] = {
            **contract_ref(),
            "manifestSha256": "0" * 64,
        }
        with self.assertRaises(EvidenceContractError) as caught:
            validate_capture_response_envelope(invalid)
        self.assertEqual(caught.exception.code, "CONTRACT_HASH_MISMATCH")
        for invalid in (
            {**base, "rejection": {"class": "contract"}},
            {**base, "state": "succeeded"},
            {**base, "state": "failed", "failureCode": "UNSUPPORTED_CONTRACT_VERSION"},
            {**base, "futureField": True},
        ):
            with self.assertRaises(EvidenceContractError):
                validate_capture_response_envelope(invalid)

    def test_contracts_are_json_ready(self) -> None:
        self.assertEqual(json_ready(RecordingSession())["state"], "idle")
        self.assertEqual(json_ready(MapJob())["state"], "queued")
        self.assertEqual(json_ready(DeviceStatus())["schema"], "gogoguard.device_status.v1")
        self.assertEqual(json_ready(CameraStreamStatus())["protocol"], "webrtc")

    def test_non_finite_numbers_become_json_null(self) -> None:
        value = json_ready(
            {
                "positive": float("inf"),
                "negative": float("-inf"),
                "unknown": float("nan"),
                "finite": 1.25,
            }
        )
        self.assertEqual(
            value,
            {
                "positive": None,
                "negative": None,
                "unknown": None,
                "finite": 1.25,
            },
        )
        self.assertNotIn("Infinity", json.dumps(value, allow_nan=False))
        self.assertNotIn("NaN", json.dumps(value, allow_nan=False))

    def test_jpeg_metadata_comes_from_sof_and_content_identity(self) -> None:
        payload = bytes.fromhex(
            "ffd8ffc00011080002000303011100021100031100ffd9"
        )
        metadata = jpeg_metadata(payload)
        self.assertEqual(metadata["width"], 3)
        self.assertEqual(metadata["height"], 2)
        self.assertEqual(metadata["bytes"], len(payload))
        self.assertEqual(len(metadata["sha256"]), 64)
        with self.assertRaisesRegex(ValueError, "SOF"):
            jpeg_metadata(b"\xff\xd8sample\xff\xd9")

    def test_frozen_platform_ids_and_verdict_are_validated(self) -> None:
        mission_id = "mission-20260811-143022-RG-狗02"
        self.assertTrue(is_safe_external_id(mission_id))
        verdict = validate_platform_mission_message(
            {
                "schema": "gogoguard.checkpoint_verdict.v1",
                "verdictId": "vd_a1b2c3",
                "missionId": mission_id,
                "mapVersion": "map-6855ba54ae11",
                "routeId": "route-6855ba54ae11-workspace-r4",
                "checkpointId": "cp_01",
                "attempt": 1,
                "action": "continue",
                "expiresAt": "2099-01-01T00:00:00+00:00",
            }
        )
        self.assertEqual(verdict["missionId"], mission_id)
        with self.assertRaisesRegex(ValueError, "action"):
            validate_platform_mission_message({**verdict, "action": "move"})
