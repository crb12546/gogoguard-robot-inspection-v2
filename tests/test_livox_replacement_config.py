from __future__ import annotations

import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class LivoxReplacementConfigTest(unittest.TestCase):
    def test_commissioned_mid360s_network_contract(self) -> None:
        config_path = ROOT / "config" / "robot" / "livox-mid360s.json"
        config = json.loads(config_path.read_text(encoding="utf-8"))
        self.assertEqual(config["lidar_summary_info"]["lidar_type"], 8)
        self.assertEqual(config["lidar_configs"][0]["ip"], "192.168.1.134")
        self.assertEqual(
            config["Mid360s"]["host_net_info"][0]["host_ip"],
            "192.168.1.5",
        )

    def test_edge_runtime_uses_product_config_and_replacement_identity(self) -> None:
        entrypoint = (
            ROOT / "deployment" / "container" / "edge-entrypoint"
        ).read_text(encoding="utf-8")
        self.assertIn("config/robot/livox-mid360s.json", entrypoint)
        self.assertIn("ARMCP6B0035634", entrypoint)
        self.assertIn("go2-u2-mid360s-ARMCP6B0035634.json", entrypoint)
        self.assertNotIn(
            "third_party/locked_stack/src/livox_ros_driver2/config/MID360s_config.json",
            entrypoint,
        )


if __name__ == "__main__":
    unittest.main()
