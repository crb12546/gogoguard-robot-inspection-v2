from __future__ import annotations

import ast
import os
import tempfile
import threading
import unittest
from pathlib import Path

from gogoguard_navigation import NavigationManager
from gogoguard_navigation.manager import UNITREE_SDK_LIBRARY_PATH


ROOT = Path(__file__).resolve().parents[1]
TRACE_RECORDER = (
    ROOT
    / "third_party/locked_stack/src/go2_nav2_runtime/go2_nav2_runtime"
    / "runtime_trace_recorder.py"
)
SAFE_CMD_NODE = (
    ROOT
    / "third_party/locked_stack/src/go2_fastlio_patrol/go2_fastlio_patrol"
    / "unitree_safe_cmd_node.py"
)
LOCALIZER_CONFIG = (
    ROOT
    / "third_party/locked_stack/src/go2_map_manager/config"
    / "continuous_map_localizer.yaml"
)
NAV2_CONFIG = (
    ROOT
    / "third_party/locked_stack/src/go2_nav2_runtime/config"
    / "go2_nav2_patrol.yaml"
)


class NavigationRuntimeCompatibilityTest(unittest.TestCase):
    def test_trace_recorder_does_not_overwrite_rclpy_node_handle(self) -> None:
        tree = ast.parse(TRACE_RECORDER.read_text(encoding="utf-8"))
        assignments = []
        for node in ast.walk(tree):
            targets = []
            if isinstance(node, (ast.Assign, ast.AnnAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for target in targets:
                if (
                    isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "self"
                    and target.attr == "handle"
                ):
                    assignments.append(target.lineno)
        self.assertEqual(assignments, [])

    def test_safe_command_cloud_subscription_uses_sensor_qos(self) -> None:
        tree = ast.parse(SAFE_CMD_NODE.read_text(encoding="utf-8"))
        cloud_subscriptions = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not (
                isinstance(node.func, ast.Attribute)
                and node.func.attr == "create_subscription"
                and len(node.args) >= 4
                and isinstance(node.args[0], ast.Name)
                and node.args[0].id == "PointCloud2"
            ):
                continue
            cloud_subscriptions.append(node.args[3])
        self.assertEqual(len(cloud_subscriptions), 1)
        qos = cloud_subscriptions[0]
        self.assertIsInstance(qos, ast.Name)
        self.assertEqual(qos.id, "qos_profile_sensor_data")

    def test_future_sensor_clock_fails_the_cloud_watchdog_closed(self) -> None:
        source = SAFE_CMD_NODE.read_text(encoding="utf-8")
        self.assertIn("sensor_clock_future", source)
        self.assertIn("max_future_cloud_stamp_s", source)

    def test_patrol_start_rejects_a_dead_motion_bridge(self) -> None:
        class Process:
            def __init__(self, return_code):
                self.return_code = return_code

            def poll(self):
                return self.return_code

        manager = NavigationManager.__new__(NavigationManager)
        manager._lock = threading.Lock()
        manager._candidate = {
            "map_version": "map-123456789abc",
            "route_id": "route-123456789abc-recorded",
        }
        manager._process = Process(None)
        manager._receiver_process = Process(-6)

        with self.assertRaisesRegex(RuntimeError, "motion bridge is not running"):
            manager.start_patrol()

    def test_motion_bridge_uses_the_paired_unitree_dds_prefix(self) -> None:
        manager = NavigationManager.__new__(NavigationManager)
        with tempfile.TemporaryDirectory() as temporary:
            manager.log_root = Path(temporary)
            original = os.environ.get("LD_LIBRARY_PATH")
            os.environ["LD_LIBRARY_PATH"] = "/opt/ros/humble/lib"
            try:
                environment = manager._receiver_environment()
            finally:
                if original is None:
                    os.environ.pop("LD_LIBRARY_PATH", None)
                else:
                    os.environ["LD_LIBRARY_PATH"] = original

        self.assertEqual(environment["LD_LIBRARY_PATH"], UNITREE_SDK_LIBRARY_PATH)
        self.assertNotIn("/opt/ros/humble/lib", environment["LD_LIBRARY_PATH"])

    def test_motion_bridge_prepares_balance_posture_before_nav2(self) -> None:
        source = (
            ROOT / "modules/navigation/gogoguard_navigation/manager.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            '[str(SDK_RECEIVER), "eth0", "5005", "prepare-posture"]',
            source,
        )

    def test_mppi_forward_samples_clear_the_commissioned_gait_deadband(self) -> None:
        config = NAV2_CONFIG.read_text(encoding="utf-8")
        self.assertIn("vx_min: 0.20", config)
        self.assertIn("required_movement_radius: 0.20", config)

    def test_slow_zone_preserves_a_passable_corridor_and_effective_gait(self) -> None:
        config = NAV2_CONFIG.read_text(encoding="utf-8")
        self.assertIn(
            "points: [1.15, 0.55, 1.15, -0.55, -0.70, -0.55, -0.70, 0.55]",
            config,
        )
        self.assertIn("slowdown_ratio: 0.70", config)

    def test_startup_search_preserves_the_250_point_quality_gate(self) -> None:
        config = LOCALIZER_CONFIG.read_text(encoding="utf-8")
        profile = (
            ROOT
            / "third_party/locked_stack/src/go2_site_ops/go2_site_ops"
            / "localization_quality.py"
        ).read_text(encoding="utf-8")
        self.assertIn("coarse_scan_leaf: 0.65", config)
        self.assertIn("coarse_scan_leaf_m=0.65", profile)
        self.assertIn("quality.min_input_points: 250", config)


if __name__ == "__main__":
    unittest.main()
