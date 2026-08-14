import unittest

from gogoguard_route import NavigationWorkspaceError, validate_workspace


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


if __name__ == "__main__":
    unittest.main()
