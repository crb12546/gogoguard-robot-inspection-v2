from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any

from gogoguard_contracts import utc_now


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def battery_status_from_low_state(message: Any) -> dict[str, Any]:
    """Translate Unitree LowState without inventing unsupported semantics."""
    bms = getattr(message, "bms_state", None)
    soc = _number(getattr(bms, "soc", None))
    if soc is not None and not 0.0 <= soc <= 100.0:
        soc = None
    voltage = _number(getattr(message, "power_v", None))
    if voltage is not None and voltage <= 0.0:
        voltage = None
    current = _number(getattr(message, "power_a", None))
    return {
        "schema": "gogoguard.battery_status.v1",
        "percent": int(soc) if soc is not None else None,
        # Unitree publishes current and a BMS status byte, but its public Go2
        # LowState contract does not define a stable charging predicate. Keep
        # this honest until a docked real-device receipt establishes it.
        "charging": None,
        "voltage": voltage,
        "currentAmps": current,
        "bmsStatus": int(getattr(bms, "status", 0)) if bms is not None else None,
        "observedAt": utc_now(),
        "source": "unitree_sdk",
    }


def write_battery_status(path: Path, status: dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(status, handle, ensure_ascii=False, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, 0o640)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main() -> None:
    parser = argparse.ArgumentParser(description="persist Unitree battery observations")
    parser.add_argument("--topic", default="/lf/lowstate")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/var/lib/gogoguard/device/battery.json"),
    )
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from unitree_go.msg import LowState

    class BatteryObserver(Node):
        def __init__(self) -> None:
            super().__init__("gogoguard_battery_observer")
            qos = QoSProfile(
                depth=5,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.create_subscription(LowState, args.topic, self._on_state, qos)

        def _on_state(self, message: Any) -> None:
            try:
                write_battery_status(args.output, battery_status_from_low_state(message))
            except Exception as exc:
                self.get_logger().error(f"battery status write failed: {type(exc).__name__}")

    rclpy.init()
    node = BatteryObserver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
