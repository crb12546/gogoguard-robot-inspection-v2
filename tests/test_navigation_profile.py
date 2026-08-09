import tempfile
import unittest
from pathlib import Path

from gogoguard_navigation.profiles import (
    DEFAULT_PROFILE,
    NavigationProfileStore,
    ProfileError,
    validate_profile,
)
from gogoguard_navigation.manager import NavigationManager


class NavigationProfileTest(unittest.TestCase):
    def test_delivery_defaults_match_commissioning_decision(self):
        profile = validate_profile(DEFAULT_PROFILE)
        self.assertEqual(profile["motion"]["straightSpeedMps"], 0.60)
        self.assertEqual(profile["motion"]["turnSpeedRadps"], 0.40)
        self.assertEqual(profile["avoidance"]["slowZoneHalfWidthM"], 0.30)
        self.assertEqual(profile["motion"]["detourSpeedMps"], 0.40)
        self.assertEqual(profile["recovery"]["progressTimeoutS"], 5.0)
        self.assertEqual(profile["recovery"]["mppiRetryLimit"], 2)
        self.assertEqual(profile["controller"]["batchSize"], 1000)

    def test_v2_impossible_speeds_migrate_to_receiver_limits(self):
        legacy = validate_profile(DEFAULT_PROFILE)
        legacy["schema"] = "gogoguard.navigation_profile.v2"
        legacy["controller"]["batchSize"] = 700
        legacy["motion"].update(
            {
                "straightSpeedMps": 0.9,
                "detourSpeedMps": 0.7,
                "turnSpeedRadps": 0.6,
                "lateralSpeedMps": 0.4,
            }
        )
        migrated = validate_profile(legacy)
        self.assertEqual(migrated["schema"], "gogoguard.navigation_profile.v3")
        self.assertEqual(
            migrated["motion"],
            {
                "straightSpeedMps": 0.6,
                "detourSpeedMps": 0.6,
                "turnSpeedRadps": 0.5,
                "lateralSpeedMps": 0.2,
                "accelerationMps2": 0.9,
            },
        )
        self.assertEqual(migrated["controller"]["batchSize"], 1000)

    def test_v3_rejects_speed_above_receiver_limit(self):
        profile = validate_profile(DEFAULT_PROFILE)
        profile["motion"]["straightSpeedMps"] = 0.65
        with self.assertRaisesRegex(ProfileError, "目标直线巡航速度"):
            validate_profile(profile)

    def test_migrated_cruise_profile_reaches_launch_boundary(self):
        legacy = validate_profile(DEFAULT_PROFILE)
        legacy["schema"] = "gogoguard.navigation_profile.v2"
        legacy["controller"]["batchSize"] = 700
        legacy["motion"]["straightSpeedMps"] = 0.9
        profile = validate_profile(legacy)
        candidate = {
            "localization_map": "/maps/map.pcd",
            "map_version": "map-aaaaaaaaaaaa",
            "route": "/maps/route.json",
            "runtime_profile": "/maps/runtime_profile.json",
            "localization_map_hash": "map-hash",
            "route_hash": "route-hash",
            "runtime_profile_hash": "profile-hash",
        }
        arguments = NavigationManager._launch_arguments(
            candidate,
            site_id="site",
            robot_id="robot",
            sensor_id="sensor",
            log_root=Path("/logs"),
            profile=profile,
        )
        self.assertIn("straight_speed_mps:=0.6", arguments)
        self.assertIn("mppi_batch_size:=1000", arguments)

    def test_v1_blocked_timeout_migrates_to_progress_watchdog(self):
        legacy = validate_profile(DEFAULT_PROFILE)
        legacy["schema"] = "gogoguard.navigation_profile.v1"
        legacy.pop("recovery")
        legacy["avoidance"]["blockedDecisionS"] = 3.5
        migrated = validate_profile(legacy)
        self.assertEqual(migrated["schema"], "gogoguard.navigation_profile.v3")
        self.assertEqual(migrated["recovery"]["progressTimeoutS"], 3.5)
        self.assertNotIn("blockedDecisionS", migrated["avoidance"])

    def test_invalid_slow_zone_cannot_be_saved(self):
        profile = validate_profile(DEFAULT_PROFILE)
        profile["avoidance"]["slowZoneHalfWidthM"] = 0.25
        profile["avoidance"]["stopZoneHalfWidthM"] = 0.30
        with self.assertRaisesRegex(ProfileError, "减速区"):
            validate_profile(profile)

    def test_mppi_compute_budget_is_bounded(self):
        profile = validate_profile(DEFAULT_PROFILE)
        profile["controller"] = dict(profile["controller"])
        profile["controller"].update(
            {"frequencyHz": 30, "timeSteps": 80, "batchSize": 2000, "iterationCount": 2}
        )
        with self.assertRaisesRegex(ProfileError, "MPPI 计算量"):
            validate_profile(profile)

    def test_update_is_versioned_and_rollback_is_available(self):
        with tempfile.TemporaryDirectory() as temporary:
            store = NavigationProfileStore(Path(temporary))
            first = store.get()
            changed = dict(first)
            changed["motion"] = dict(first["motion"])
            changed["motion"]["straightSpeedMps"] = 0.55
            second = store.update(changed)["profile"]
            self.assertEqual(second["revision"], 2)
            self.assertEqual(second["motion"]["straightSpeedMps"], 0.55)
            restored = store.rollback()["profile"]
            self.assertEqual(restored["revision"], 3)
            self.assertEqual(restored["motion"]["straightSpeedMps"], 0.60)


if __name__ == "__main__":
    unittest.main()
