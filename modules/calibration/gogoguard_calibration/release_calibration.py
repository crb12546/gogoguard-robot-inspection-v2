"""Exact calibration evidence required before a map may reach a robot."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping

from .calibration import MountCalibration
from .coordinate_contract import ContractError, CoordinateContract, content_sha256
from .internal_calibration import SensorInternalCalibration


SCHEMA = "go2.release_calibration_bundle.v1"


def _identity(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ContractError("%s must be a non-empty string" % label)
    normalized = value.strip()
    if len(normalized) > 128 or any(
        character in normalized for character in ("/", "\\", "\x00")
    ):
        raise ContractError("%s is not a safe identity" % label)
    return normalized


@dataclass(frozen=True)
class ReleaseCalibrationBundle:
    """Self-contained coordinate, mount, and sensor-internal calibration."""

    robot_id: str
    sensor_id: str
    coordinate_contract: CoordinateContract
    mount_calibration: MountCalibration
    sensor_internal_calibration: SensorInternalCalibration

    @classmethod
    def load(
        cls,
        *,
        coordinate_contract_path: Path,
        mount_calibration_path: Path,
        sensor_internal_calibration_path: Path,
        robot_id: str,
        sensor_id: str,
    ) -> "ReleaseCalibrationBundle":
        return cls._validated(
            robot_id=robot_id,
            sensor_id=sensor_id,
            coordinate_contract=CoordinateContract.load(coordinate_contract_path),
            mount_calibration=MountCalibration.load(
                mount_calibration_path,
                require_validated=True,
            ),
            sensor_internal_calibration=SensorInternalCalibration.load(
                sensor_internal_calibration_path,
                require_validated=True,
            ),
        )

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "ReleaseCalibrationBundle":
        if not isinstance(payload, dict) or payload.get("schema") != SCHEMA:
            raise ContractError("unsupported release calibration bundle schema")
        for key in (
            "coordinateContract",
            "mountCalibration",
            "sensorInternalCalibration",
        ):
            if not isinstance(payload.get(key), dict):
                raise ContractError("release calibration bundle has no %s" % key)
        bundle = cls._validated(
            robot_id=payload.get("robotId"),
            sensor_id=payload.get("sensorId"),
            coordinate_contract=CoordinateContract.from_dict(
                payload["coordinateContract"]
            ),
            mount_calibration=MountCalibration.from_dict(
                payload["mountCalibration"],
                require_validated=True,
            ),
            sensor_internal_calibration=SensorInternalCalibration.from_dict(
                payload["sensorInternalCalibration"],
                require_validated=True,
            ),
        )
        expected = payload.get("bundleSha256")
        if expected is not None and expected != bundle.digest:
            raise ContractError("release calibration bundle hash does not match contents")
        return bundle

    @classmethod
    def _validated(
        cls,
        *,
        robot_id: Any,
        sensor_id: Any,
        coordinate_contract: CoordinateContract,
        mount_calibration: MountCalibration,
        sensor_internal_calibration: SensorInternalCalibration,
    ) -> "ReleaseCalibrationBundle":
        normalized_robot = _identity(robot_id, "robot_id")
        normalized_sensor = _identity(sensor_id, "sensor_id")
        mount_calibration.assert_compatible(
            coordinate_contract,
            robot_id=normalized_robot,
            sensor_id=normalized_sensor,
            require_validated=True,
        )
        sensor_internal_calibration.validate(require_validated=True)
        return cls(
            robot_id=normalized_robot,
            sensor_id=normalized_sensor,
            coordinate_contract=coordinate_contract,
            mount_calibration=mount_calibration,
            sensor_internal_calibration=sensor_internal_calibration,
        )

    def _unhashed_payload(self) -> Dict[str, Any]:
        return {
            "schema": SCHEMA,
            "robotId": self.robot_id,
            "sensorId": self.sensor_id,
            "coordinateContract": dict(self.coordinate_contract.raw),
            "mountCalibration": dict(self.mount_calibration.raw),
            "sensorInternalCalibration": dict(self.sensor_internal_calibration.raw),
        }

    @property
    def digest(self) -> str:
        return content_sha256(self._unhashed_payload())

    def to_dict(self) -> Dict[str, Any]:
        payload = self._unhashed_payload()
        payload["bundleSha256"] = self.digest
        return payload

    def summary(self) -> Dict[str, Any]:
        return {
            "state": "validated",
            "robotId": self.robot_id,
            "sensorId": self.sensor_id,
            "mountCalibrationId": self.mount_calibration.calibration_id,
            "sensorInternalCalibrationId": (
                self.sensor_internal_calibration.calibration_id
            ),
            "coordinateContractId": self.coordinate_contract.contract_id,
            "bundleSha256": self.digest,
        }
