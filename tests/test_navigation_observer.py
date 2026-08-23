from __future__ import annotations

import unittest

from gogoguard_navigation.observer import fail_closed_runtime_after_timeout


class NavigationObserverRuntimeLivenessTest(unittest.TestCase):
    def test_lost_active_runtime_becomes_explicit_terminal_status(self) -> None:
        runtime = {
            "state": "PATROLLING",
            "reason": "FOLLOWING_ROUTE",
            "runtimeInstanceId": "runtime-1",
            "motionAuthorized": True,
        }
        result = fail_closed_runtime_after_timeout(
            runtime,
            last_runtime_monotonic=10.0,
            now_monotonic=13.1,
        )
        self.assertEqual(result["state"], "INTERRUPTED")
        self.assertEqual(result["reason"], "NAV_RUNTIME_LOST")
        self.assertFalse(result["motionAuthorized"])
        self.assertEqual(result["runtimeInstanceId"], "runtime-1")
        self.assertEqual(runtime["state"], "PATROLLING")

    def test_fresh_active_runtime_is_not_changed(self) -> None:
        runtime = {"state": "PATROLLING", "runtimeInstanceId": "runtime-1"}
        self.assertIs(
            fail_closed_runtime_after_timeout(
                runtime,
                last_runtime_monotonic=10.0,
                now_monotonic=12.9,
            ),
            runtime,
        )

    def test_idle_or_never_observed_runtime_is_not_reclassified(self) -> None:
        idle = {"state": "IDLE", "runtimeInstanceId": "runtime-1"}
        self.assertIs(
            fail_closed_runtime_after_timeout(
                idle,
                last_runtime_monotonic=10.0,
                now_monotonic=100.0,
            ),
            idle,
        )
        self.assertIsNone(
            fail_closed_runtime_after_timeout(
                None,
                last_runtime_monotonic=None,
                now_monotonic=100.0,
            )
        )


if __name__ == "__main__":
    unittest.main()
