import unittest

from gogoguard_route import (
    NavigationWorkspaceError,
    navigation_surface_cells,
    plan_navigation_preview,
    validate_workspace,
)


class NavigationWorkspaceTest(unittest.TestCase):
    def value(self):
        return {
            "mapJobId": "map-123456789abc",
            "route": [[0.0, 0.0], [1.0, 0.0]],
            "allowedArea": [[-1.0, -1.0], [2.0, -1.0], [2.0, 1.0], [-1.0, 1.0]],
            "robotRadiusM": 0.48,
        }

    def test_ready_workspace_has_one_visible_robot_radius(self):
        value = validate_workspace(self.value(), require_ready=True)
        self.assertTrue(value["ready"])
        self.assertEqual(value["robotRadiusM"], 0.48)
        self.assertEqual(len(value["workspaceHash"]), 64)

    def test_route_cannot_leave_green_allowed_area(self):
        value = self.value()
        value["route"] = [[0.0, 0.0], [3.0, 0.0]]
        with self.assertRaisesRegex(NavigationWorkspaceError, "leaves"):
            validate_workspace(value, require_ready=True)

    def test_route_must_leave_radius_clearance_from_green_boundary(self):
        value = self.value()
        value["route"] = [[0.0, 0.7], [1.0, 0.7]]
        with self.assertRaisesRegex(NavigationWorkspaceError, "robot radius"):
            validate_workspace(value, require_ready=True)

    def test_self_crossing_green_boundary_is_rejected(self):
        value = self.value()
        value["allowedArea"] = [[-1.0, -1.0], [2.0, 1.0], [-1.0, 1.0], [2.0, -1.0]]
        with self.assertRaisesRegex(NavigationWorkspaceError, "crosses itself"):
            validate_workspace(value)

    def test_new_surface_requires_explicit_review_but_v1_workspace_migrates(self):
        fresh = self.value()
        fresh["navigationSurface"] = {
            "reviewed": False,
            "manualBlockedCells": [],
            "manualClearCells": [],
        }
        with self.assertRaisesRegex(NavigationWorkspaceError, "static navigation map"):
            validate_workspace(fresh, require_ready=True)

        migrated = validate_workspace(self.value(), require_ready=True)
        self.assertTrue(migrated["navigationSurface"]["reviewed"])

    def test_manual_surface_edits_override_height_slice(self):
        value = self.value()
        value["navigationSurface"] = {
            "resolutionM": 0.10,
            "obstacleMinZ": 0.05,
            "obstacleMaxZ": 1.80,
            "manualBlockedCells": [[5, 5]],
            "manualClearCells": [[1, 0]],
            "reviewed": True,
        }
        preview = navigation_surface_cells(
            value,
            [[0.15, 0.05, 1.0], [0.25, 0.05, 2.5]],
        )
        self.assertNotIn("ground", preview)
        self.assertNotIn([1, 0], preview["occupiedCells"])
        self.assertIn([5, 5], preview["occupiedCells"])

    def test_surface_aggregation_drops_isolated_return_but_keeps_wall_cluster(self):
        value = self.value()
        value["navigationSurface"] = {
            "resolutionM": 0.10,
            "obstacleMinZ": 0.05,
            "obstacleMaxZ": 1.80,
            "manualBlockedCells": [],
            "manualClearCells": [],
            "reviewed": True,
        }
        preview = navigation_surface_cells(
            value,
            [
                [0.15, 0.55, 1.0],  # isolated return
                [0.75, 0.55, 1.0],
                [0.85, 0.55, 1.0],  # adjacent cells: coherent obstacle
            ],
        )
        self.assertNotIn([1, 5], preview["occupiedCells"])
        self.assertIn([7, 5], preview["occupiedCells"])
        self.assertIn([8, 5], preview["occupiedCells"])
        self.assertEqual(preview["stats"]["rejectedIsolatedCellCount"], 1)

    def test_ground_preview_separates_floor_from_roof_and_stays_review_only(self):
        value = self.value()
        value["navigationSurface"] = {
            "resolutionM": 0.10,
            "obstacleMinZ": 0.05,
            "obstacleMaxZ": 1.80,
            "manualBlockedCells": [],
            "manualClearCells": [],
            "reviewed": True,
        }
        cloud = []
        for cell_x in range(-4, 10, 2):
            for cell_y in range(-4, 5, 2):
                cloud.append([cell_x / 10, cell_y / 10, 0.0])
                cloud.append([cell_x / 10, cell_y / 10, 3.0])
        cloud.extend([[0.4, 0.4, 1.0], [0.45, 0.4, 1.0]])

        ground = navigation_surface_cells(value, cloud, include_ground=True)["ground"]

        self.assertEqual(ground["schema"], "gogoguard.ground_surface_preview.v1")
        self.assertFalse(ground["navigationAuthority"])
        self.assertAlmostEqual(ground["referenceElevationM"], 0.0, places=3)
        self.assertGreater(ground["stats"]["measuredCellCount"], 10)
        self.assertTrue(
            all(abs(row[2]) < 0.01 for row in ground["measuredCells"]),
            "roof and obstacle returns must not become the floor elevation",
        )
        self.assertEqual(ground["regionSource"], "operator_allowed_area")

    def test_ground_preview_before_green_area_only_dilates_measured_support(self):
        value = self.value()
        value["allowedArea"] = None
        cloud = [
            [0.0, 0.0, 0.0],
            [0.05, 0.0, 0.01],
            [0.2, 0.0, 0.0],
            [0.25, 0.0, 0.01],
        ]

        ground = navigation_surface_cells(value, cloud, include_ground=True)["ground"]

        self.assertEqual(ground["regionSource"], "measured_support_neighborhood")
        self.assertGreater(ground["stats"]["measuredCellCount"], 0)
        self.assertLess(
            sum(ground["stats"][key] for key in (
                "measuredCellCount", "inferredCellCount", "unknownCellCount"
            )),
            200,
        )

    def test_preview_routes_around_static_obstacle(self):
        value = self.value()
        value["allowedArea"] = [[-1, -1], [4, -1], [4, 3], [-1, 3]]
        value["route"] = [[0, 0], [3, 0]]
        value["navigationSurface"] = {
            "resolutionM": 0.10,
            "obstacleMinZ": 0.05,
            "obstacleMaxZ": 1.80,
            "manualBlockedCells": [],
            "manualClearCells": [],
            "reviewed": True,
        }
        wall = [[1.5, y / 10, 1.0] for y in range(-10, 11)]
        result = plan_navigation_preview(value, wall, [0, 0], [3, 0])
        self.assertTrue(result["reachable"])
        self.assertGreater(result["lengthM"], 3.0)


if __name__ == "__main__":
    unittest.main()
