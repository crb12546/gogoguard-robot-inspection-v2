import copy
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
        self.assertEqual(profile["motion"]["targetCruiseMps"], 0.60)
        self.assertEqual(profile["motion"]["maxForwardMps"], 0.90)
        self.assertEqual(profile["motion"]["accelerationMps2"], 0.90)
        self.assertEqual(profile["motion"]["decelerationMps2"], 0.90)
        self.assertEqual(profile["motion"]["turnSpeedRadps"], 0.40)
        self.assertNotIn("detourSpeedMps", profile["motion"])
        self.assertNotIn("slowZoneHalfWidthM", profile["avoidance"])
        self.assertEqual(profile["recovery"]["progressTimeoutS"], 5.0)
        self.assertEqual(profile["recovery"]["replanIntervalS"], 0.75)
        self.assertEqual(profile["controller"]["batchSize"], 1000)

    def test_v2_impossible_speeds_migrate_to_receiver_limits(self):
        legacy = validate_profile(DEFAULT_PROFILE)
        legacy["schema"] = "gogoguard.navigation_profile.v2"
        legacy["controller"]["batchSize"] = 700
        legacy["motion"] = {
            "straightSpeedMps": 0.9,
            "detourSpeedMps": 0.7,
            "turnSpeedRadps": 0.6,
            "lateralSpeedMps": 0.4,
            "accelerationMps2": 0.9,
        }
        migrated = validate_profile(legacy)
        self.assertEqual(migrated["schema"], "gogoguard.navigation_profile.v6")
        self.assertEqual(migrated["motion"]["targetCruiseMps"], 0.9)
        self.assertEqual(migrated["motion"]["maxForwardMps"], 0.9)
        self.assertNotIn("detourSpeedMps", migrated["motion"])
        self.assertEqual(migrated["motion"]["turnSpeedRadps"], 0.6)
        self.assertEqual(migrated["motion"]["lateralSpeedMps"], 0.2)
        self.assertEqual(migrated["controller"]["batchSize"], 1000)

    def test_v5_requires_control_headroom_for_cruise_target(self):
        profile = validate_profile(DEFAULT_PROFILE)
        profile["motion"]["targetCruiseMps"] = 0.85
        profile["motion"]["maxForwardMps"] = 0.8
        with self.assertRaisesRegex(ProfileError, "前进控制上限"):
            validate_profile(profile)

    def test_v4_field_regression_migrates_to_route_completing_envelope(self):
        legacy = copy.deepcopy(DEFAULT_PROFILE)
        legacy["schema"] = "gogoguard.navigation_profile.v4"
        legacy["motion"].update(
            {
                "detourSpeedMps": 0.60,
                "turnSpeedRadps": 0.60,
                "accelerationMps2": 2.00,
                "decelerationMps2": 2.50,
            }
        )
        migrated = validate_profile(legacy)
        self.assertEqual(migrated["schema"], "gogoguard.navigation_profile.v6")
        self.assertNotIn("detourSpeedMps", migrated["motion"])
        self.assertEqual(migrated["motion"]["turnSpeedRadps"], 0.40)
        self.assertEqual(migrated["motion"]["accelerationMps2"], 0.90)
        self.assertEqual(migrated["motion"]["decelerationMps2"], 0.90)

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
            "allowed_area_mask": "/maps/allowed-area-mask.yaml",
            "allowed_area_mask_image": "/maps/allowed-area-mask.pgm",
            "navigation_map": "/maps/navigation-map.yaml",
            "navigation_map_image": "/maps/navigation-map.pgm",
            "localization_map_hash": "map-hash",
            "route_hash": "route-hash",
            "runtime_profile_hash": "profile-hash",
            "allowed_area_mask_hash": "mask-hash",
            "allowed_area_mask_image_hash": "mask-image-hash",
            "navigation_map_hash": "navigation-map-hash",
            "navigation_map_image_hash": "navigation-map-image-hash",
        }
        arguments = NavigationManager._launch_arguments(
            candidate,
            site_id="site",
            robot_id="robot",
            sensor_id="sensor",
            log_root=Path("/logs"),
            profile=profile,
        )
        self.assertIn("target_cruise_mps:=0.6", arguments)
        self.assertIn("max_forward_mps:=0.9", arguments)
        self.assertIn("acceleration_mps2:=0.9", arguments)
        self.assertIn("deceleration_mps2:=0.9", arguments)
        self.assertIn("mppi_batch_size:=1000", arguments)

    def test_v1_blocked_timeout_migrates_to_progress_watchdog(self):
        legacy = validate_profile(DEFAULT_PROFILE)
        legacy["schema"] = "gogoguard.navigation_profile.v1"
        legacy.pop("recovery")
        legacy["avoidance"]["blockedDecisionS"] = 3.5
        migrated = validate_profile(legacy)
        self.assertEqual(migrated["schema"], "gogoguard.navigation_profile.v6")
        self.assertEqual(migrated["recovery"]["progressTimeoutS"], 3.5)
        self.assertNotIn("blockedDecisionS", migrated["avoidance"])

    def test_v5_safety_rectangles_are_discarded_during_migration(self):
        profile = validate_profile(DEFAULT_PROFILE)
        profile["schema"] = "gogoguard.navigation_profile.v5"
        profile["avoidance"].update(
            {"stopZoneFrontM": 0.55, "slowZoneFrontM": 1.0, "slowdownRatio": 0.85}
        )
        migrated = validate_profile(profile)
        self.assertEqual(migrated["schema"], "gogoguard.navigation_profile.v6")
        self.assertNotIn("stopZoneFrontM", migrated["avoidance"])
        self.assertNotIn("slowZoneFrontM", migrated["avoidance"])
        self.assertNotIn("slowdownRatio", migrated["avoidance"])

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
            changed["motion"]["targetCruiseMps"] = 0.55
            second = store.update(changed)["profile"]
            self.assertEqual(second["revision"], 2)
            self.assertEqual(second["motion"]["targetCruiseMps"], 0.55)
            restored = store.rollback()["profile"]
            self.assertEqual(restored["revision"], 3)
            self.assertEqual(restored["motion"]["targetCruiseMps"], 0.60)


if __name__ == "__main__":
    unittest.main()
