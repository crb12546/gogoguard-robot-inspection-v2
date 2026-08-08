from __future__ import annotations

import ast
import io
import os
import re
import shutil
import subprocess
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

from gogoguard_navigation import NavigationManager
from gogoguard_navigation.manager import DEFAULT_PROFILE, UNITREE_SDK_LIBRARY_PATH


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
NAV2_LAUNCH = (
    ROOT
    / "third_party/locked_stack/src/go2_nav2_runtime/launch"
    / "active_map_patrol.launch.py"
)
INCIDENT_OVERLAY = (
    ROOT / "modules/navigation/overlays/go2_incident_diagnostics.patch"
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

    def test_delivery_overlay_keeps_effective_gait_and_adds_detour_controller(self) -> None:
        runtime_source = ROOT / "third_party/locked_stack/src/go2_nav2_runtime"
        bridge_source = ROOT / "third_party/locked_stack/src/go2_cmd_vel_bridge"
        overlay = (
            ROOT
            / "modules/navigation/overlays/go2_nav2_runtime_delivery.patch"
        )
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary)
            shutil.copytree(runtime_source, workspace / "go2_nav2_runtime")
            shutil.copytree(bridge_source, workspace / "go2_cmd_vel_bridge")
            subprocess.run(
                ["git", "-C", str(workspace), "apply", str(overlay)],
                check=True,
                capture_output=True,
                text=True,
            )
            subprocess.run(
                ["git", "-C", str(workspace), "apply", str(INCIDENT_OVERLAY)],
                check=True,
                capture_output=True,
                text=True,
            )
            runtime_core = (
                workspace
                / "go2_nav2_runtime/go2_nav2_runtime/runtime_core.py"
            ).read_text(encoding="utf-8")
            self.assertIn(
                "0.10 <= settings.speed_limit_mps <= 0.60",
                runtime_core,
            )
            nav2_config = (
                workspace / "go2_nav2_runtime/config/go2_nav2_patrol.yaml"
            ).read_text(encoding="utf-8")
            self.assertIn("controller_plugins: [FollowPath, DetourPath]", nav2_config)
            self.assertIn("vx_min: 0.20", nav2_config)
            self.assertIn(
                "plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                nav2_config,
            )
            self.assertIn("desired_linear_vel: 0.24", nav2_config)
            self.assertIn("use_rotate_to_heading: true", nav2_config)
            detour_speed = float(
                re.search(
                    r"DetourPath:.*?desired_linear_vel:\s*([0-9.]+)",
                    nav2_config,
                    re.DOTALL,
                ).group(1)
            )
            slowdown_ratio = float(
                re.search(
                    r"SlowZone:.*?slowdown_ratio:\s*([0-9.]+)",
                    nav2_config,
                    re.DOTALL,
                ).group(1)
            )
            self.assertGreaterEqual(detour_speed * slowdown_ratio, 0.20)
            voxel = re.search(
                r"voxel_layer:.*?origin_z:\s*([-0-9.]+).*?"
                r"z_resolution:\s*([0-9.]+).*?z_voxels:\s*([0-9]+).*?"
                r"max_obstacle_height:\s*([0-9.]+)",
                nav2_config,
                re.DOTALL,
            )
            self.assertIsNotNone(voxel)
            origin_z, z_resolution, z_voxels, max_obstacle_height = (
                float(voxel.group(1)),
                float(voxel.group(2)),
                int(voxel.group(3)),
                float(voxel.group(4)),
            )
            self.assertLessEqual(origin_z, -0.32)
            self.assertGreaterEqual(
                origin_z + z_resolution * z_voxels,
                max_obstacle_height,
            )
            runtime_manager = (
                workspace
                / "go2_nav2_runtime/go2_nav2_runtime/patrol_runtime_manager.py"
            ).read_text(encoding="utf-8")
            self.assertIn('goal.controller_id = "DetourPath"', runtime_manager)
            self.assertIn("plan_detour_diagnostic", runtime_manager)
            launch = (
                workspace
                / "go2_nav2_runtime/launch/active_map_patrol.launch.py"
            ).read_text(encoding="utf-8")
            self.assertIn(
                '"DetourPath.rotate_to_heading_angular_vel": turn_speed',
                launch,
            )
            receiver = (
                workspace
                / "go2_cmd_vel_bridge/src/go2_sdk2_udp_receiver.cpp"
            ).read_text(encoding="utf-8")
            self.assertIn("max_vx=0.600 max_vy=0.200", receiver)
            self.assertIn("pkt.vx, pkt.vy, pkt.vyaw, 0.60, 0.20, 0.5", receiver)

    def test_runtime_exit_reaps_its_motion_bridge(self) -> None:
        class Process:
            def __init__(self, return_code, *, wait_return_code=None):
                self.returncode = return_code
                self.wait_return_code = (
                    return_code if wait_return_code is None else wait_return_code
                )
                self.pid = 999999
                self.waited = False

            def poll(self):
                return self.returncode

            def wait(self, timeout=None):
                self.waited = True
                self.returncode = self.wait_return_code
                return self.returncode

        runtime = Process(1)
        receiver = Process(None, wait_return_code=0)
        runtime_log = io.BytesIO()
        receiver_log = io.BytesIO()
        manager = NavigationManager.__new__(NavigationManager)
        manager._lock = threading.Lock()
        manager._process = runtime
        manager._receiver_process = receiver
        manager._log_handle = runtime_log
        manager._receiver_log_handle = receiver_log
        manager._last_runtime_exit = None
        manager._last_receiver_exit = None

        manager._reap_runtime_generation(
            runtime, receiver, runtime_log, receiver_log
        )

        self.assertIsNone(manager._process)
        self.assertIsNone(manager._receiver_process)
        self.assertEqual(manager._last_runtime_exit, 1)
        self.assertEqual(manager._last_receiver_exit, 0)
        self.assertTrue(receiver.waited)
        self.assertTrue(runtime_log.closed)
        self.assertTrue(receiver_log.closed)

    def test_failed_nav2_readiness_cleans_motion_bridge_before_reporting(self) -> None:
        class Process:
            def __init__(self, return_code, pid):
                self.returncode = return_code
                self.pid = pid
                self.waited = False

            def poll(self):
                return self.returncode

            def wait(self, timeout=None):
                self.waited = True
                if self.returncode is None:
                    self.returncode = -2
                return self.returncode

        receiver = Process(None, 999998)
        runtime = Process(1, 999999)
        candidate = {
            "candidate_id": "map-123456789abc",
            "map_version": "map-123456789abc",
            "localization_map": "/tmp/map.pcd",
            "route": "/tmp/route.json",
            "runtime_profile": "/tmp/runtime_profile.json",
            "localization_map_hash": "a",
            "route_hash": "b",
            "runtime_profile_hash": "c",
        }

        class Routes:
            def get(self, candidate_id):
                return candidate

        class Profiles:
            def get(self):
                return DEFAULT_PROFILE

        with tempfile.TemporaryDirectory() as temporary:
            manager = NavigationManager.__new__(NavigationManager)
            manager.site_id = "site"
            manager.robot_id = "robot"
            manager.sensor_id = "sensor"
            manager.log_root = Path(temporary)
            manager.routes = Routes()
            manager.profiles = Profiles()
            manager._lock = threading.Lock()
            manager._candidate = None
            manager._process = None
            manager._receiver_process = None
            manager._log_handle = None
            manager._receiver_log_handle = None
            manager._last_runtime_exit = None
            manager._last_receiver_exit = None

            with mock.patch(
                "gogoguard_navigation.manager.subprocess.Popen",
                side_effect=[receiver, runtime],
            ), mock.patch("gogoguard_navigation.manager.time.sleep"):
                with self.assertRaisesRegex(
                    RuntimeError, "Nav2 failed readiness check: exit 1"
                ):
                    manager.start_runtime("map-123456789abc")

        self.assertTrue(receiver.waited)
        self.assertIsNone(manager._process)
        self.assertIsNone(manager._receiver_process)
        self.assertEqual(manager._last_runtime_exit, 1)
        self.assertEqual(manager._last_receiver_exit, -2)

    def test_mppi_forward_samples_clear_the_commissioned_gait_deadband(self) -> None:
        config = NAV2_CONFIG.read_text(encoding="utf-8")
        self.assertIn("vx_min: 0.20", config)
        self.assertIn("required_movement_radius: 0.15", config)

    def test_slow_zone_preserves_a_passable_corridor_and_effective_gait(self) -> None:
        config = NAV2_CONFIG.read_text(encoding="utf-8")
        self.assertIn(
            "points: [1.00, 0.30, 1.00, -0.30, -0.55, -0.30, -0.55, 0.30]",
            config,
        )
        self.assertIn("slowdown_ratio: 0.85", config)

    def test_diagnostics_names_a_blocked_patrol_instead_of_runnable(self) -> None:
        manager = NavigationManager.__new__(NavigationManager)
        with tempfile.TemporaryDirectory() as temporary:
            manager.log_root = Path(temporary)
            manager.status = lambda: {
                "runtime_process": {"running": True},
                "motion_bridge": {"running": True},
                "runtime": {
                    "state": "BLOCKED",
                    "reason": "LOCAL_PATH_BLOCKED",
                    "operatorMessage": "局部路径受阻",
                },
                "localization": {"usable": True, "reason": "accepted"},
                "safety": {"stopReason": "authorization_off"},
            }
            manager.profile = lambda: DEFAULT_PROFILE
            result = manager.diagnostics()
        self.assertEqual(result["summary"], "巡检受阻")

    def test_operator_blocked_time_controls_nav2_progress_checker(self) -> None:
        source = NAV2_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            '"progress_checker.movement_time_allowance": blocked_decision_s',
            source,
        )

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
