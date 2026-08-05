"""Versioned, production-gated Go2 sensor-mount calibration files."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Mapping

from .coordinate_contract import ContractError, content_sha256
from .se3 import RigidTransform


SCHEMA = "go2.mount_calibration.v2"
FIXED_GEOMETRY_SCHEMA = "go2.mount_calibration.v3"
LEGACY_SCHEMA = "go2.mount_calibration.v1"
VALID_STATES = {"draft", "measured", "validated", "retired"}


def _nonempty(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ContractError("%s must be a non-empty string" % label)
    return value.strip()


def _timestamp(value: Any) -> str:
    parsed = _nonempty(value, "measured_at")
    try:
        datetime.fromisoformat(parsed.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ContractError("measured_at must be ISO-8601: %s" % exc)
    return parsed


def _finite_nonnegative(value: Any, label: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        raise ContractError("%s must be numeric" % label)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ContractError("%s must be finite and non-negative" % label)
    return parsed


def _sha256(value: Any, label: str) -> str:
    parsed = _nonempty(value, label).lower()
    if len(parsed) != 64 or any(character not in "0123456789abcdef" for character in parsed):
        raise ContractError("%s must be a SHA-256 digest" % label)
    return parsed


def _maximum(value: Any, maximum: float, label: str) -> float:
    parsed = _finite_nonnegative(value, label)
    if parsed > maximum:
        raise ContractError("%s exceeds production limit %.6g" % (label, maximum))
    return parsed


def _minimum(value: Any, minimum: float, label: str) -> float:
    parsed = _finite_nonnegative(value, label)
    if parsed < minimum:
        raise ContractError("%s is below production limit %.6g" % (label, minimum))
    return parsed


@dataclass(frozen=True)
class MountCalibration:
    raw: Mapping[str, Any]
    transform: RigidTransform

    @classmethod
    def from_dict(
        cls,
        data: Mapping[str, Any],
        require_validated: bool = False,
    ) -> "MountCalibration":
        if not isinstance(data, dict):
            raise ContractError("mount calibration root must be an object")
        try:
            transform = RigidTransform.from_dict(data.get("transform", {}))
        except (TypeError, ValueError) as exc:
            raise ContractError("invalid mount transform: %s" % exc)
        calibration = cls(dict(data), transform)
        calibration.validate(require_validated=require_validated)
        return calibration

    @classmethod
    def load(cls, path: Path, require_validated: bool = False) -> "MountCalibration":
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ContractError("cannot read mount calibration %s: %s" % (path, exc))
        return cls.from_dict(data, require_validated=require_validated)

    @property
    def digest(self) -> str:
        return content_sha256(self.raw)

    @property
    def calibration_id(self) -> str:
        return str(self.raw["calibration_id"])

    @property
    def state(self) -> str:
        return str(self.raw["state"])

    def validate(self, require_validated: bool = False) -> None:
        schema = self.raw.get("schema")
        if schema not in {SCHEMA, FIXED_GEOMETRY_SCHEMA, LEGACY_SCHEMA}:
            raise ContractError("unsupported mount calibration schema")
        _nonempty(self.raw.get("calibration_id"), "calibration_id")
        state = self.raw.get("state")
        if state not in VALID_STATES:
            raise ContractError("invalid calibration state: %r" % state)
        if require_validated and state != "validated":
            raise ContractError("motion/map publication requires validated calibration")
        if require_validated and schema not in {SCHEMA, FIXED_GEOMETRY_SCHEMA}:
            raise ContractError(
                "legacy mount calibration cannot authorize motion/map publication"
            )
        _nonempty(self.raw.get("robot_id"), "robot_id")
        _nonempty(self.raw.get("sensor_id"), "sensor_id")
        _timestamp(self.raw.get("measured_at"))
        _nonempty(self.raw.get("method"), "method")

        if self.transform.parent_frame != "base_link":
            raise ContractError("mount transform parent must be base_link")
        if self.transform.child_frame != "lidar_link":
            raise ContractError("mount transform child must be lidar_link")

        quality = self.raw.get("quality")
        if not isinstance(quality, dict):
            raise ContractError("quality must be an object")
        if schema == LEGACY_SCHEMA:
            self._validate_legacy_quality(quality)
        elif schema == FIXED_GEOMETRY_SCHEMA:
            self._validate_fixed_geometry_provenance_and_quality(quality)
        else:
            self._validate_v2_provenance_and_quality(quality, state)

        approvals = self.raw.get("approvals")
        if not isinstance(approvals, dict):
            raise ContractError("approvals must be an object")
        if state == "validated":
            _nonempty(approvals.get("validated_by"), "approvals.validated_by")
            _timestamp(approvals.get("validated_at"))

    def _validate_fixed_geometry_provenance_and_quality(
        self,
        quality: Mapping[str, Any],
    ) -> None:
        """Validate an owner-frozen, single-axis mechanical installation.

        This schema records the actual acceptance basis instead of fabricating
        a translation survey or an independent-validation result. It is for a
        rigid installation whose roll, yaw and centreline position are fixed by
        construction and whose pitch is supported by multiple field sessions.
        """

        provenance = self.raw.get("provenance")
        if not isinstance(provenance, dict):
            raise ContractError("provenance must be an object")
        if provenance.get("decision") != "operator_fixed_geometry":
            raise ContractError("fixed geometry needs an operator decision")
        if provenance.get("mount_model") != "single_axis_pitch":
            raise ContractError("fixed geometry mount_model must be single_axis_pitch")

        capture_hashes = provenance.get("capture_sha256")
        if not isinstance(capture_hashes, list) or len(capture_hashes) < 3:
            raise ContractError("fixed geometry needs at least three field captures")
        normalized_hashes = [
            _sha256(value, "provenance.capture_sha256") for value in capture_hashes
        ]
        if len(set(normalized_hashes)) != len(normalized_hashes):
            raise ContractError("fixed geometry capture hashes must be distinct")
        _sha256(provenance.get("installation_photo_sha256"), "installation photo")

        sources = provenance.get("geometry_sources")
        if not isinstance(sources, list) or len(sources) < 2:
            raise ContractError("fixed geometry needs robot and sensor geometry sources")
        source_ids = set()
        for index, source in enumerate(sources):
            if not isinstance(source, dict):
                raise ContractError("geometry_sources[%d] must be an object" % index)
            source_ids.add(_nonempty(source.get("id"), "geometry source id"))
            _nonempty(source.get("uri"), "geometry source uri")
        if not {"unitree_go2_model", "livox_mid360_manual"}.issubset(source_ids):
            raise ContractError("fixed geometry needs official Go2 and MID-360 sources")

        pitch_deg = _finite_nonnegative(
            quality.get("pitch_estimate_deg"), "quality.pitch_estimate_deg"
        )
        if pitch_deg > 90.0:
            raise ContractError("fixed geometry pitch must not exceed 90 degrees")
        _maximum(
            quality.get("pitch_session_span_deg"),
            1.0,
            "quality.pitch_session_span_deg",
        )
        _maximum(
            quality.get("translation_uncertainty_m"),
            0.02,
            "quality.translation_uncertainty_m",
        )
        _finite_nonnegative(
            quality.get("stationary_body_height_m"),
            "quality.stationary_body_height_m",
        )
        _maximum(
            quality.get("stationary_height_spread_m"),
            0.01,
            "quality.stationary_height_spread_m",
        )

        x, y, z, w = self.transform.rotation_xyzw
        if abs(x) > 1.0e-6 or abs(z) > 1.0e-6 or w <= 0.0:
            raise ContractError("fixed geometry rotation must contain pitch only")
        quaternion_pitch_deg = math.degrees(2.0 * math.atan2(y, w))
        if abs(quaternion_pitch_deg - pitch_deg) > 0.01:
            raise ContractError("fixed geometry quaternion and pitch disagree")
        if abs(self.transform.translation_xyz_m[1]) > 0.005:
            raise ContractError("fixed centreline mount must keep |y| within 5 mm")

    def _validate_legacy_quality(self, quality: Mapping[str, Any]) -> None:
        _finite_nonnegative(
            quality.get("gravity_residual_deg"),
            "quality.gravity_residual_deg",
        )
        _finite_nonnegative(
            quality.get("translation_stddev_m"),
            "quality.translation_stddev_m",
        )
        _finite_nonnegative(
            quality.get("rotation_stddev_deg"),
            "quality.rotation_stddev_deg",
        )
        sample_count = quality.get("stationary_sample_count")
        if not isinstance(sample_count, int) or sample_count < 1:
            raise ContractError("quality.stationary_sample_count must be positive")

    def _validate_v2_provenance_and_quality(
        self,
        quality: Mapping[str, Any],
        state: str,
    ) -> None:
        provenance = self.raw.get("provenance")
        if not isinstance(provenance, dict):
            raise ContractError("provenance must be an object")
        if provenance.get("solution_schema") != "go2.mount_calibration_solution.v2":
            raise ContractError("provenance.solution_schema is unsupported")
        _sha256(provenance.get("solution_sha256"), "provenance.solution_sha256")
        capture_hashes = provenance.get("capture_sha256")
        if not isinstance(capture_hashes, list) or len(capture_hashes) < 3:
            raise ContractError("provenance.capture_sha256 needs at least three captures")
        normalized_hashes = [
            _sha256(value, "provenance.capture_sha256") for value in capture_hashes
        ]
        if len(set(normalized_hashes)) != len(normalized_hashes):
            raise ContractError("provenance capture hashes must be distinct")
        _nonempty(provenance.get("clock_policy"), "provenance.clock_policy")
        solver = provenance.get("solver")
        if not isinstance(solver, dict):
            raise ContractError("provenance.solver must be an object")
        if solver.get("library") != "OpenCV":
            raise ContractError("provenance solver must be OpenCV")
        _nonempty(solver.get("version"), "provenance.solver.version")
        if solver.get("primary_method") != "PARK":
            raise ContractError("provenance primary hand-eye method must be PARK")
        methods = solver.get("methods")
        if not isinstance(methods, list) or "PARK" not in methods or len(set(methods)) < 3:
            raise ContractError("provenance needs at least three hand-eye methods")

        translation_survey = provenance.get("translation_survey")
        if not isinstance(translation_survey, dict):
            raise ContractError("provenance.translation_survey must be an object")
        if translation_survey.get("schema") != "go2.mount_translation_survey.v1":
            raise ContractError("provenance translation survey schema is unsupported")
        _sha256(
            translation_survey.get("sha256"),
            "provenance.translation_survey.sha256",
        )
        observation_count = translation_survey.get("observation_count")
        if type(observation_count) is not int or observation_count < 3:
            raise ContractError(
                "provenance.translation_survey.observation_count must be at least 3"
            )
        survey_evidence = translation_survey.get("evidence_sha256")
        if not isinstance(survey_evidence, list) or len(survey_evidence) < observation_count:
            raise ContractError(
                "provenance translation survey needs independent evidence per observation"
            )
        normalized_survey_evidence = [
            _sha256(value, "provenance.translation_survey.evidence_sha256")
            for value in survey_evidence
        ]
        if len(set(normalized_survey_evidence)) < observation_count:
            raise ContractError(
                "provenance translation survey evidence must be independent"
            )

        _maximum(
            quality.get("translation_survey_repeat_p95_m"),
            0.02,
            "quality.translation_survey_repeat_p95_m",
        )
        _maximum(
            quality.get("translation_survey_uncertainty_m"),
            0.02,
            "quality.translation_survey_uncertainty_m",
        )
        _maximum(
            quality.get("translation_survey_instrument_resolution_max_m"),
            0.005,
            "quality.translation_survey_instrument_resolution_max_m",
        )
        _maximum(
            quality.get("repeat_rotation_p95_deg"),
            0.75,
            "quality.repeat_rotation_p95_deg",
        )
        _maximum(
            quality.get("holdout_rotation_p95_deg"),
            1.5,
            "quality.holdout_rotation_p95_deg",
        )
        _minimum(
            quality.get("rotation_axis_ratio"),
            0.02,
            "quality.rotation_axis_ratio",
        )
        _maximum(
            quality.get("clock_residual_p95_ms"),
            30.0,
            "quality.clock_residual_p95_ms",
        )
        _minimum(
            quality.get("time_alignment_correlation"),
            0.55,
            "quality.time_alignment_correlation",
        )

        independent = self.raw.get("independent_validation")
        if state == "validated":
            if not isinstance(independent, dict):
                raise ContractError(
                    "validated calibration requires independent_validation"
                )
            _maximum(
                independent.get("ground_plane_residual_deg"),
                0.5,
                "independent_validation.ground_plane_residual_deg",
            )
            _maximum(
                independent.get("fixed_structure_translation_p95_m"),
                0.03,
                "independent_validation.fixed_structure_translation_p95_m",
            )
            _maximum(
                independent.get("fixed_structure_rotation_p95_deg"),
                1.0,
                "independent_validation.fixed_structure_rotation_p95_deg",
            )
            _sha256(
                independent.get("evidence_sha256"),
                "independent_validation.evidence_sha256",
            )

    def assert_compatible(
        self,
        contract: Any,
        robot_id: str,
        sensor_id: str,
        require_validated: bool = True,
    ) -> None:
        self.validate(require_validated=require_validated)
        if self.raw.get("coordinate_contract_id") != contract.contract_id:
            raise ContractError("calibration coordinate contract id does not match")
        if self.raw.get("coordinate_contract_sha256") != contract.digest:
            raise ContractError("calibration coordinate contract hash does not match")
        if self.raw.get("robot_id") != robot_id:
            raise ContractError("calibration belongs to a different robot")
        if self.raw.get("sensor_id") != sensor_id:
            raise ContractError("calibration belongs to a different sensor")

    def summary(self) -> Dict[str, Any]:
        return {
            "calibrationId": self.calibration_id,
            "state": self.state,
            "sha256": self.digest,
            "robotId": self.raw["robot_id"],
            "sensorId": self.raw["sensor_id"],
            "transform": self.transform.to_dict(),
            "quality": dict(self.raw["quality"]),
        }
