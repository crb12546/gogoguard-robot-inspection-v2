import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "modules/navigation/ros/go2_nav2_runtime"))
sys.path.insert(0, str(ROOT / "third_party/locked_stack/src/go2_site_ops"))

from go2_nav2_runtime.local_detour import GridView, plan_detour, route_rejoin_index
from gogoguard_navigation.detour_diagnostics import plan_detour_diagnostic


class LocalDetourTest(unittest.TestCase):
    def test_routes_around_wall_and_rejoins_ahead(self):
        width = height = 30
        costs = [0] * (width * height)
        # A wall blocks the direct line but leaves passages above and below.
        for y in range(8, 23):
            costs[y * width + 15] = 100
        grid = GridView(width, height, 0.10, 0.0, 0.0, tuple(costs))
        path = plan_detour(grid, (0.55, 1.55), (2.55, 1.55))
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 2)
        self.assertTrue(any(abs(point[1] - 1.55) > 0.65 for point in path))
        route = [(index * 0.25, 0.0) for index in range(20)]
        self.assertEqual(route_rejoin_index(route, 4, 2.0), 12)

    def test_returns_none_when_goal_is_sealed(self):
        width = height = 12
        costs = [0] * (width * height)
        for y in range(height):
            costs[y * width + 6] = 100
        grid = GridView(width, height, 0.10, 0.0, 0.0, tuple(costs))
        self.assertIsNone(plan_detour(grid, (0.25, 0.55), (0.95, 0.55)))
        path, diagnostic = plan_detour_diagnostic(grid, (0.25, 0.55), (0.95, 0.55))
        self.assertIsNone(path)
        self.assertEqual(diagnostic["outcome"], "disconnected")
        self.assertGreater(diagnostic["expansionCount"], 0)

    def test_diagnostic_exposes_search_and_result_without_changing_path(self):
        width = height = 20
        costs = [0] * (width * height)
        grid = GridView(width, height, 0.10, 0.0, 0.0, tuple(costs))
        expected = plan_detour(grid, (0.25, 0.25), (1.55, 1.55))
        actual, diagnostic = plan_detour_diagnostic(grid, (0.25, 0.25), (1.55, 1.55))
        self.assertEqual(actual, expected)
        self.assertEqual(diagnostic["outcome"], "path_found")
        self.assertGreater(len(diagnostic["expandedCells"]), 0)
        self.assertEqual(diagnostic["pathPoints"], [[round(x, 3), round(y, 3)] for x, y in actual])


if __name__ == "__main__":
    unittest.main()
