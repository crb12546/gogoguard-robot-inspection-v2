"""CLI for validating and identifying a coordinate contract."""

import argparse
import json
from pathlib import Path

from .coordinate_contract import CoordinateContract, ContractError


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("contract", type=Path)
    args = parser.parse_args()
    try:
        contract = CoordinateContract.load(args.contract)
    except ContractError as exc:
        parser.exit(2, "INVALID: %s\n" % exc)
    print(
        json.dumps(
            {
                "valid": True,
                "contractId": contract.contract_id,
                "sha256": contract.digest,
                "frames": contract.role_frames,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
