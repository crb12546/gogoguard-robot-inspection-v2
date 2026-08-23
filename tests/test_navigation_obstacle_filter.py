from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FILTER_CORE = (
    ROOT
    / "modules/navigation/ros/go2_nav2_runtime/go2_nav2_runtime"
    / "obstacle_filter_core.py"
)


def load_filter_core():
    spec = importlib.util.spec_from_file_location("obstacle_filter_core", FILTER_CORE)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load obstacle filter core")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class NavigationObstacleFilterTest(unittest.TestCase):
    def test_only_rigid_self_volume_is_removed(self) -> None:
        module = load_filter_core()
        box = module.SelfExclusionBox()
        values = [
            (0.0, 0.0, 0.4),
            (0.41, 0.21, 0.4),
            (0.49, 0.0, 0.4),
            (0.0, 0.29, 0.4),
            (-0.31, 0.38, 0.48),
        ]
        filtered = list(module.filtered_xyz_points(values, self_box=box))
        self.assertNotIn((0.0, 0.0, 0.4), filtered)
        self.assertNotIn((0.41, 0.21, 0.4), filtered)
        self.assertIn((0.49, 0.0, 0.4), filtered)
        self.assertIn((0.0, 0.29, 0.4), filtered)
        # The field-observed side cluster is deliberately not hidden as self;
        # the narrower rectangular planner decides whether it is passable.
        self.assertIn((-0.31, 0.38, 0.48), filtered)

    def test_invalid_and_non_finite_points_never_reach_navigation(self) -> None:
        module = load_filter_core()
        values = [(1.0, 0.0), (math.nan, 0.0, 0.2), (1.0, 0.0, 0.2)]
        self.assertEqual(
            list(
                module.filtered_xyz_points(
                    values, self_box=module.SelfExclusionBox()
                )
            ),
            [(1.0, 0.0, 0.2)],
        )

    def test_invalid_self_box_fails_closed(self) -> None:
        module = load_filter_core()
        with self.assertRaisesRegex(ValueError, "invalid"):
            list(
                module.filtered_xyz_points(
                    [(1.0, 0.0, 0.2)],
                    self_box=module.SelfExclusionBox(x_min=1.0, x_max=0.0),
                )
            )


if __name__ == "__main__":
    unittest.main()
