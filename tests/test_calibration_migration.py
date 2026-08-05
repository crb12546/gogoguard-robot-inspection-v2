from __future__ import annotations

import json
import math
import unittest
from pathlib import Path

from gogoguard_calibration import (
    ContractError,
    CoordinateContract,
    MountCalibration,
    SensorInternalCalibration,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "modules" / "calibration" / "gogoguard_calibration" / "config"


class CalibrationMigrationTest(unittest.TestCase):
    def test_fixed_mount_is_identity_bound_and_loadable(self) -> None:
        contract = CoordinateContract.load(CONFIG / "coordinate_contract.json")
        mount = MountCalibration.load(CONFIG / "go2-u2-mid360.json", require_validated=True)
        mount.assert_compatible(
            contract,
            robot_id="LLYJ0001",
            sensor_id="ARMCP1U0038561",
            require_validated=True,
        )
        self.assertAlmostEqual(mount.raw["quality"]["pitch_estimate_deg"], 32.242667)
        self.assertEqual(mount.transform.translation_xyz_m, (0.235, 0.0, 0.125))

    def test_internal_imu_has_factory_translation_and_identity_rotation(self) -> None:
        internal = SensorInternalCalibration.load(
            CONFIG / "mid360_internal_calibration.json", require_validated=True
        )
        self.assertEqual(internal.transform.translation_xyz_m, (0.011, 0.02329, -0.04412))
        self.assertEqual(internal.transform.rotation_xyzw, (0.0, 0.0, 0.0, 1.0))

    def test_mount_quaternion_encodes_the_frozen_pitch_only(self) -> None:
        mount = MountCalibration.load(CONFIG / "go2-u2-mid360.json", require_validated=True)
        x, y, z, w = mount.transform.rotation_xyzw
        pitch = math.degrees(2.0 * math.atan2(y, w))
        self.assertAlmostEqual(pitch, 32.242667, places=5)
        self.assertEqual((x, z), (0.0, 0.0))

    def test_changed_robot_identity_is_rejected(self) -> None:
        contract = CoordinateContract.load(CONFIG / "coordinate_contract.json")
        mount = MountCalibration.load(CONFIG / "go2-u2-mid360.json", require_validated=True)
        with self.assertRaises(ContractError):
            mount.assert_compatible(contract, robot_id="another-go2", sensor_id="ARMCP1U0038561")

    def test_fastlio_output_override_matches_composed_calibration(self) -> None:
        mount = MountCalibration.load(CONFIG / "go2-u2-mid360.json", require_validated=True)
        internal = SensorInternalCalibration.load(
            CONFIG / "mid360_internal_calibration.json", require_validated=True
        )
        base_to_imu = mount.transform.compose(internal.transform)
        override = (ROOT / "config" / "robot" / "fastlio_base_output.yaml").read_text(encoding="utf-8")
        for value in (*base_to_imu.translation_xyz_m, *base_to_imu.rotation_xyzw):
            self.assertIn(str(value), override)
        self.assertIn(mount.digest, override)
        self.assertIn(internal.digest, override)
        self.assertIn("output.require_base_frame_calibration: true", override)

    def test_migrated_operational_files_equal_locked_source(self) -> None:
        source = ROOT / "third_party" / "locked_stack" / "src" / "go2_site_ops" / "config"
        for name in ("coordinate_contract.json", "go2-u2-mid360.json", "mid360_internal_calibration.json"):
            self.assertEqual(
                json.loads((CONFIG / name).read_text(encoding="utf-8")),
                json.loads((source / name).read_text(encoding="utf-8")),
            )
