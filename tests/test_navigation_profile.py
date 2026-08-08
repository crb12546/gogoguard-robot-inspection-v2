import tempfile
import unittest
from pathlib import Path

from gogoguard_navigation.profiles import (
    DEFAULT_PROFILE,
    NavigationProfileStore,
    ProfileError,
    validate_profile,
)


class NavigationProfileTest(unittest.TestCase):
    def test_delivery_defaults_match_commissioning_decision(self):
        profile = validate_profile(DEFAULT_PROFILE)
        self.assertEqual(profile["motion"]["straightSpeedMps"], 0.60)
        self.assertEqual(profile["motion"]["turnSpeedRadps"], 0.40)
        self.assertEqual(profile["avoidance"]["slowZoneHalfWidthM"], 0.30)
        self.assertEqual(profile["motion"]["detourSpeedMps"], 0.40)
        self.assertEqual(profile["recovery"]["progressTimeoutS"], 5.0)
        self.assertEqual(profile["recovery"]["mppiRetryLimit"], 2)

    def test_v1_blocked_timeout_migrates_to_progress_watchdog(self):
        legacy = validate_profile(DEFAULT_PROFILE)
        legacy["schema"] = "gogoguard.navigation_profile.v1"
        legacy.pop("recovery")
        legacy["avoidance"]["blockedDecisionS"] = 3.5
        migrated = validate_profile(legacy)
        self.assertEqual(migrated["schema"], "gogoguard.navigation_profile.v2")
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
            changed["motion"]["straightSpeedMps"] = 0.70
            second = store.update(changed)["profile"]
            self.assertEqual(second["revision"], 2)
            self.assertEqual(second["motion"]["straightSpeedMps"], 0.70)
            restored = store.rollback()["profile"]
            self.assertEqual(restored["revision"], 3)
            self.assertEqual(restored["motion"]["straightSpeedMps"], 0.60)


if __name__ == "__main__":
    unittest.main()
