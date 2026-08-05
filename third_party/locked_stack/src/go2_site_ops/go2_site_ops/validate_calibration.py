"""CLI for validating and identifying a Go2 mount calibration."""

import argparse
import json
from pathlib import Path

from .calibration import MountCalibration
from .coordinate_contract import ContractError


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration", type=Path)
    parser.add_argument("--require-validated", action="store_true")
    parser.add_argument("--robot-id", default="")
    parser.add_argument("--sensor-id", default="")
    args = parser.parse_args()
    try:
        calibration = MountCalibration.load(
            args.calibration,
            require_validated=args.require_validated,
        )
        if args.robot_id and calibration.raw.get("robot_id") != args.robot_id:
            raise ContractError("calibration belongs to a different robot")
        if args.sensor_id and calibration.raw.get("sensor_id") != args.sensor_id:
            raise ContractError("calibration belongs to a different sensor")
    except ContractError as exc:
        parser.exit(2, "INVALID: %s\n" % exc)
    print(json.dumps(calibration.summary(), ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
