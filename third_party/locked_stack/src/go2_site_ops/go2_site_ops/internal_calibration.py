"""Validated, manufacturer-sourced LiDAR-to-IMU calibration records."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping

from .coordinate_contract import ContractError, content_sha256
from .se3 import RigidTransform


SCHEMA = "go2.sensor_internal_calibration.v1"


@dataclass(frozen=True)
class SensorInternalCalibration:
    raw: Mapping[str, Any]
    transform: RigidTransform

    @classmethod
    def from_dict(
        cls,
        data: Mapping[str, Any],
        *,
        require_validated: bool = True,
    ) -> "SensorInternalCalibration":
        if not isinstance(data, dict) or data.get("schema") != SCHEMA:
            raise ContractError("unsupported sensor internal calibration schema")
        transform_data = data.get("transform")
        if not isinstance(transform_data, dict):
            raise ContractError("sensor internal calibration has no transform")
        if transform_data.get("parentFrame") != "lidar_link":
            raise ContractError("sensor internal parent must be lidar_link")
        if transform_data.get("childFrame") != "lidar_imu":
            raise ContractError("sensor internal child must be lidar_imu")
        tum = transform_data.get("tum")
        if not isinstance(tum, list) or len(tum) != 7:
            raise ContractError("sensor internal transform must be TUM[7]")
        try:
            values = tuple(float(value) for value in tum)
        except (TypeError, ValueError) as exc:
            raise ContractError("sensor internal TUM must be numeric: %s" % exc)
        if not all(math.isfinite(value) for value in values):
            raise ContractError("sensor internal TUM must be finite")
        try:
            transform = RigidTransform(
                parent_frame="lidar_link",
                child_frame="lidar_imu",
                translation_xyz_m=values[:3],
                rotation_xyzw=values[3:],
            )
        except ValueError as exc:
            raise ContractError("invalid sensor internal transform: %s" % exc)
        calibration = cls(raw=dict(data), transform=transform)
        calibration.validate(require_validated=require_validated)
        return calibration

    @classmethod
    def load(
        cls,
        path: Path,
        *,
        require_validated: bool = True,
    ) -> "SensorInternalCalibration":
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ContractError(
                "cannot read sensor internal calibration %s: %s" % (path, exc)
            )
        return cls.from_dict(data, require_validated=require_validated)

    def validate(self, *, require_validated: bool = True) -> None:
        calibration_id = self.raw.get("calibrationId")
        if not isinstance(calibration_id, str) or not calibration_id.strip():
            raise ContractError("sensor internal calibrationId is required")
        source = self.raw.get("source")
        if not isinstance(source, dict) or source.get("kind") != "manufacturer_manual":
            raise ContractError("sensor internal calibration needs manufacturer source")
        if require_validated and self.raw.get("validated") is not True:
            raise ContractError("sensor internal calibration is not validated")

    @property
    def calibration_id(self) -> str:
        return str(self.raw["calibrationId"])

    @property
    def digest(self) -> str:
        # Hash the operational calibration contract, not descriptive fields
        # such as prose basis or model display name. This is the same payload
        # embedded in each GLIM job manifest.
        return content_sha256(self.to_dict())

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema": SCHEMA,
            "calibrationId": self.calibration_id,
            "transform": dict(self.raw["transform"]),
            "source": (
                dict(self.raw["source"])
                if isinstance(self.raw["source"], Mapping)
                else self.raw["source"]
            ),
            "validated": True,
        }
