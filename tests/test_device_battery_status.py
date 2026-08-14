from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from gogoguard_device_io.battery_status import (
    battery_status_from_low_state,
    write_battery_status,
)


class BatteryStatusTest(unittest.TestCase):
    def test_edge_observer_uses_unitree_bare_dds_domain(self) -> None:
        repository = Path(__file__).resolve().parents[1]
        entrypoint = (
            repository / "deployment"
            / "container"
            / "edge-entrypoint"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "setsid env ROS_DOMAIN_ID=0 gogoguard-battery-observer",
            entrypoint,
        )
        observer = (
            repository
            / "modules"
            / "device_io"
            / "gogoguard_device_io"
            / "battery_status.py"
        ).read_text(encoding="utf-8")
        self.assertIn("reliability=ReliabilityPolicy.RELIABLE", observer)

    def test_converts_unitree_low_state_without_guessing_charging(self) -> None:
        message = SimpleNamespace(
            bms_state=SimpleNamespace(soc=87, status=3),
            power_v=32.14,
            power_a=2.5,
        )
        status = battery_status_from_low_state(message)
        self.assertEqual(status["percent"], 87)
        self.assertEqual(status["voltage"], 32.14)
        self.assertEqual(status["currentAmps"], 2.5)
        self.assertIsNone(status["charging"])
        self.assertEqual(status["source"], "unitree_sdk")

    def test_invalid_values_are_persisted_as_null(self) -> None:
        message = SimpleNamespace(
            bms_state=SimpleNamespace(soc=255, status=0),
            power_v=float("nan"),
            power_a=float("inf"),
        )
        status = battery_status_from_low_state(message)
        self.assertIsNone(status["percent"])
        self.assertIsNone(status["voltage"])
        self.assertIsNone(status["currentAmps"])
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "device" / "battery.json"
            write_battery_status(path, status)
            self.assertEqual(json.loads(path.read_text(encoding="utf-8")), status)


if __name__ == "__main__":
    unittest.main()
