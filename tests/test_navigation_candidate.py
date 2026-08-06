import json
import math
import struct
import tempfile
import unittest
from pathlib import Path

from gogoguard_route import RouteManager


class NavigationCandidateTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.job_id = "map-123456789abc"
        self.artifacts = self.root / "map-jobs" / self.job_id / "artifacts"
        self.artifacts.mkdir(parents=True)
        points = [
            (-1.0, -1.0, 0.0, 0.2),
            (1.0, -1.0, 0.0, 0.4),
            (1.0, 1.0, 0.0, 0.6),
            (-1.0, 1.0, 0.0, 0.8),
        ]
        header = (
            "ply\nformat binary_little_endian 1.0\n"
            f"element vertex {len(points)}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property float intensity\nend_header\n"
        ).encode("ascii")
        with (self.artifacts / "map.ply").open("wb") as handle:
            handle.write(header)
            for point in points:
                handle.write(struct.pack("<ffff", *point))
        trajectory = []
        for index in range(21):
            trajectory.append([index * 0.10, 0.0, 0.0])
        # The final vertical posture change has no planar motion and must not
        # create a route tail.
        trajectory.extend([[2.0, 0.0, -index * 0.02] for index in range(10)])
        (self.artifacts / "map.json").write_text(
            json.dumps({"source": "cloud-glim", "trajectory": trajectory}),
            encoding="utf-8",
        )

    def tearDown(self):
        self.temporary.cleanup()

    def test_prepare_converts_map_and_removes_stationary_posture_tail(self):
        candidate = RouteManager(self.root, site_id="test-site").prepare_map_job(self.job_id)
        self.assertEqual(candidate["point_count"], 4)
        self.assertEqual(candidate["candidate_generation"], 3)
        self.assertTrue(Path(candidate["localization_map"]).read_bytes().startswith(b"# .PCD v0.7"))
        route = json.loads(Path(candidate["route"]).read_text(encoding="utf-8"))
        self.assertEqual(route["schema"], "go2.route.v1")
        self.assertGreaterEqual(len(route["waypoints"]), 7)
        self.assertLessEqual(len(route["waypoints"]), 10)
        self.assertTrue(math.isclose(route["waypoints"][-1]["x"], 2.0, abs_tol=0.11))
        self.assertEqual(candidate["route_waypoint_count"], len(route["waypoints"]))
        self.assertEqual(len(candidate["route_hash"]), 64)
        self.assertEqual(len(candidate["localization_map_hash"]), 64)
        profile = json.loads(Path(candidate["runtime_profile"]).read_text(encoding="utf-8"))
        self.assertEqual(profile["patrol"]["speedLimitMps"], 0.60)

    def test_repeated_prepare_keeps_runtime_artifact_identity(self):
        manager = RouteManager(self.root, site_id="test-site")
        first = manager.prepare_map_job(self.job_id)
        artifacts = [
            Path(first["localization_map"]),
            Path(first["route"]),
            Path(first["runtime_profile"]),
        ]
        before = [(path.stat().st_ino, path.stat().st_mtime_ns) for path in artifacts]

        second = manager.prepare_map_job(self.job_id)

        after = [(path.stat().st_ino, path.stat().st_mtime_ns) for path in artifacts]
        self.assertEqual(second, first)
        self.assertEqual(after, before)
        self.assertEqual(len(second["source_map_json_sha256"]), 64)


if __name__ == "__main__":
    unittest.main()
