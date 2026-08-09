from __future__ import annotations

import ast
import io
import os
import re
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
    / "modules/navigation/ros/go2_nav2_runtime/go2_nav2_runtime"
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
    / "modules/navigation/ros/go2_nav2_runtime/config"
    / "go2_nav2_patrol.yaml"
)
NAV2_LAUNCH = (
    ROOT
    / "modules/navigation/ros/go2_nav2_runtime/launch"
    / "active_map_patrol.launch.py"
)
RUNTIME_MANAGER = ROOT / (
    "modules/navigation/ros/go2_nav2_runtime/go2_nav2_runtime/"
    "patrol_runtime_manager.py"
)
OWNED_RECEIVER = ROOT / (
    "modules/device_io/ros/go2_cmd_vel_bridge/src/"
    "go2_sdk2_udp_receiver.cpp"
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

    def test_patrol_start_waits_for_a_new_healthy_costmap(self) -> None:
        class Process:
            def poll(self):
                return None

        manager = NavigationManager.__new__(NavigationManager)
        manager._lock = threading.Lock()
        manager._candidate = {
            "map_version": "map-123456789abc",
            "route_id": "route-123456789abc-recorded",
        }
        manager._process = Process()
        manager._receiver_process = Process()
        statuses = iter(
            [
                {
                    "runtime_process": {"running": True},
                    "runtime": {"costmapHealth": {"sequence": 4, "healthy": True}},
                },
                {
                    "runtime_process": {"running": True},
                    "runtime": {"costmapHealth": {"sequence": 5, "healthy": True}},
                },
            ]
        )
        manager.status = lambda: next(statuses)
        calls = []

        def service(name, type_name, request, timeout=12):
            calls.append((name, type_name))
            return {"success": True, "output": "success: true"}

        manager._service = service
        result = manager.start_patrol()
        self.assertTrue(result["success"])
        self.assertEqual(
            calls[0],
            (
                "/local_costmap/clear_entirely_local_costmap",
                "nav2_msgs/srv/ClearEntireCostmap",
            ),
        )
        self.assertEqual(calls[1][0], "/go2/patrol/start")

    def test_localization_resume_uses_the_same_fresh_costmap_barrier(self) -> None:
        source = RUNTIME_MANAGER.read_text(encoding="utf-8")
        refresh = source[source.index("def _resume_costmap_refresh") :]
        self.assertIn("ClearEntireCostmap.Request()", refresh)
        self.assertIn(
            "self.costmap_sequence > baseline and health.healthy",
            refresh,
        )
        resume_branch = source[source.index("elif self.resume_pending") :]
        self.assertIn("refresh = self._resume_costmap_refresh(now)", resume_branch)
        self.assertLess(
            resume_branch.index("refresh = self._resume_costmap_refresh(now)"),
            resume_branch.index("self._request_resume()"),
        )

    def test_operator_stop_is_idempotent_and_reports_remote_release(self) -> None:
        manager = NavigationManager.__new__(NavigationManager)
        manager._lock = threading.Lock()
        manager._process = None
        manager._receiver_process = None
        manager._log_handle = None
        manager._receiver_log_handle = None
        manager._last_runtime_exit = None
        manager._last_receiver_exit = None
        manager.status = lambda: {
            "runtime_process": {"running": False},
            "motion_bridge": {"running": False},
        }
        result = manager.stop_patrol()
        self.assertTrue(result["success"])
        self.assertTrue(result["remoteControlReleased"])
        self.assertTrue(result["runtimeStopped"])
        self.assertTrue(result["motionBridgeStopped"])

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

    def test_owned_runtime_keeps_gait_and_controller_contracts(self) -> None:
        runtime_core = (RUNTIME_MANAGER.parent / "runtime_core.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("0.10 <= settings.speed_limit_mps <= 0.60", runtime_core)
        nav2_config = NAV2_CONFIG.read_text(encoding="utf-8")
        self.assertIn("controller_plugins: [FollowPath, DetourPath]", nav2_config)
        self.assertIn("vx_min: 0.20", nav2_config)
        self.assertIn(
            "plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
            nav2_config,
        )
        self.assertIn("desired_linear_vel: 0.40", nav2_config)
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
        self.assertIn("global_frame: map", nav2_config)
        self.assertIn("plugins: [obstacle_layer, inflation_layer]", nav2_config)
        self.assertIn("plugin: nav2_costmap_2d::ObstacleLayer", nav2_config)
        self.assertNotIn("nav2_costmap_2d::VoxelLayer", nav2_config)
        self.assertNotIn("origin_z:", nav2_config)
        runtime_manager = RUNTIME_MANAGER.read_text(encoding="utf-8")
        self.assertIn('goal.controller_id = "DetourPath"', runtime_manager)
        self.assertIn("route_obstruction_evidence", runtime_manager)
        self.assertIn("_request_mppi_suffix", runtime_manager)
        self.assertNotIn("detour_in_map[:-1]", runtime_manager)
        launch = NAV2_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            '"DetourPath.rotate_to_heading_angular_vel": turn_speed', launch
        )
        receiver = OWNED_RECEIVER.read_text(encoding="utf-8")
        self.assertIn("max_vx=0.600 max_vy=0.200", receiver)
        self.assertIn("pkt.vx, pkt.vy, pkt.vyaw, 0.60, 0.20, 0.5", receiver)

    def test_container_builds_owned_runtime_without_navigation_overlays(self) -> None:
        dockerfile = (
            ROOT / "deployment/container/Dockerfile"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "COPY modules/navigation/ros/go2_nav2_runtime ", dockerfile
        )
        self.assertIn(
            "COPY modules/device_io/ros/go2_cmd_vel_bridge/src/", dockerfile
        )
        self.assertNotIn("go2_nav2_runtime_delivery.patch", dockerfile)
        self.assertNotIn("go2_incident_diagnostics.patch", dockerfile)
        self.assertNotIn(
            "cp -a third_party/locked_stack/src/go2_nav2_runtime", dockerfile
        )

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
        self.assertIn("plugin: nav2_controller::PoseProgressChecker", config)
        self.assertIn("required_movement_angle: 0.15", config)

    def test_motion_receiver_prepares_posture_before_zero_motion_baseline(self) -> None:
        source = OWNED_RECEIVER.read_text(encoding="utf-8")
        stand_up = source.index("sport_client.StandUp()")
        balance_stand = source.index("sport_client.BalanceStand()")
        startup_stop = source.index("const int32_t startup_stop_ret = sport_client.StopMove()")
        socket_bind = source.index("if (bind(sock")
        ready_banner = source.index('SDK2 receiver started on interface:')

        self.assertLess(stand_up, balance_stand)
        self.assertLess(balance_stand, startup_stop)
        self.assertLess(startup_stop, socket_bind)
        self.assertLess(socket_bind, ready_banner)
        self.assertIn("return 8;", source[startup_stop:socket_bind])

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

    def test_progress_timeout_only_controls_nav2_progress_checker(self) -> None:
        source = NAV2_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            '"progress_checker.movement_time_allowance": progress_timeout_s',
            source,
        )
        self.assertNotIn("blocked_decision_s", source)

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
