import json
import math
import struct
import tempfile
import unittest
import zipfile
from pathlib import Path

from gogoguard_route import NavigationWorkspaceError, NavigationWorkspaceStore, RouteManager


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
        self.artifacts.chmod(0o550)
        NavigationWorkspaceStore(self.root).update(
            self.job_id,
            {
                "routeSource": "recorded",
                "route": [[0.0, 0.0], [0.5, 0.0], [1.0, 0.0], [1.5, 0.0], [2.0, 0.0]],
                "allowedArea": [[-1.0, -1.0], [3.0, -1.0], [3.0, 1.0], [-1.0, 1.0]],
                "robotRadiusM": 0.48,
            },
        )

    def tearDown(self):
        self.artifacts.chmod(0o750)
        self.temporary.cleanup()

    def test_workspace_is_writable_without_mutating_read_only_map_artifacts(self):
        workspace_path = self.artifacts.parent / "navigation-workspace.json"
        self.assertTrue(workspace_path.is_file())
        self.assertFalse((self.artifacts / "navigation-workspace.json").exists())
        self.assertEqual(self.artifacts.stat().st_mode & 0o777, 0o550)

    def test_legacy_workspace_is_read_then_migrated_on_update(self):
        current_path = self.artifacts.parent / "navigation-workspace.json"
        legacy_path = self.artifacts / "navigation-workspace.json"
        self.artifacts.chmod(0o750)
        current_path.replace(legacy_path)
        self.artifacts.chmod(0o550)

        store = NavigationWorkspaceStore(self.root)
        legacy = store.get(self.job_id)
        updated = store.update(
            self.job_id,
            {
                "routeSource": legacy["routeSource"],
                "route": legacy["route"],
                "allowedArea": legacy["allowedArea"],
                "robotRadiusM": legacy["robotRadiusM"],
            },
        )

        self.assertEqual(updated["revision"], legacy["revision"] + 1)
        self.assertTrue(current_path.is_file())
        self.assertTrue(legacy_path.is_file())

    def test_prepare_converts_map_and_removes_stationary_posture_tail(self):
        candidate = RouteManager(self.root, site_id="test-site").prepare_map_job(self.job_id)
        self.assertEqual(candidate["point_count"], 4)
        self.assertEqual(candidate["candidate_generation"], 6)
        self.assertTrue(Path(candidate["localization_map"]).read_bytes().startswith(b"# .PCD v0.7"))
        route = json.loads(Path(candidate["route"]).read_text(encoding="utf-8"))
        self.assertEqual(route["schema"], "go2.route.v1")
        self.assertEqual(len(route["waypoints"]), 5)
        self.assertTrue(math.isclose(route["waypoints"][-1]["x"], 2.0, abs_tol=0.11))
        self.assertEqual(candidate["route_waypoint_count"], len(route["waypoints"]))
        self.assertEqual(len(candidate["route_hash"]), 64)
        self.assertEqual(len(candidate["localization_map_hash"]), 64)
        self.assertEqual(len(candidate["workspace_hash"]), 64)
        self.assertTrue(Path(candidate["allowed_area_mask"]).is_file())
        self.assertTrue(Path(candidate["allowed_area_mask_image"]).is_file())
        self.assertEqual(
            Path(candidate["allowed_area_mask_image"]).read_bytes()[:2], b"P5"
        )
        profile = json.loads(Path(candidate["runtime_profile"]).read_text(encoding="utf-8"))
        self.assertEqual(profile["patrol"]["speedLimitMps"], 0.60)
        self.assertEqual(
            profile["localization"]["qualityProfileId"],
            "go2-vgicp-orin-v2",
        )
        self.assertEqual(
            profile["navigation"]["plannerProfileId"],
            "go2-nav2-smac-2d-v1",
        )

    def test_repeated_prepare_keeps_runtime_artifact_identity(self):
        manager = RouteManager(self.root, site_id="test-site")
        first = manager.prepare_map_job(self.job_id)
        artifacts = [
            Path(first["localization_map"]),
            Path(first["route"]),
            Path(first["runtime_profile"]),
            Path(first["allowed_area_mask"]),
            Path(first["allowed_area_mask_image"]),
            Path(first["allowed_area_mask_metadata"]),
        ]
        before = [(path.stat().st_ino, path.stat().st_mtime_ns) for path in artifacts]

        second = manager.prepare_map_job(self.job_id)

        after = [(path.stat().st_ino, path.stat().st_mtime_ns) for path in artifacts]
        self.assertEqual(second, first)
        self.assertEqual(after, before)
        self.assertEqual(len(second["source_map_json_sha256"]), 64)

    def test_prepare_requires_a_saved_allowed_area(self):
        (self.artifacts.parent / "navigation-workspace.json").unlink()
        with self.assertRaisesRegex(NavigationWorkspaceError, "green allowed area"):
            RouteManager(self.root, site_id="test-site").prepare_map_job(self.job_id)

    def test_platform_bundle_has_relative_versioned_assets_and_hashes(self):
        manager = RouteManager(self.root, site_id="test-site")
        candidate = manager.prepare_map_job(self.job_id)
        descriptor = manager.export_platform_bundle(self.job_id)
        manifest = json.loads(Path(descriptor["manifest"]).read_text(encoding="utf-8"))
        self.assertEqual(manifest["mapVersion"], self.job_id)
        self.assertEqual(manifest["routeId"], candidate["route_id"])
        self.assertEqual(manifest["frame"], "map")
        self.assertTrue(manifest["gravityAligned"])
        self.assertEqual(
            {item["path"] for item in manifest["files"]},
            {
                "map.pcd",
                "route.json",
                "execution-route.json",
                "navigation-workspace.json",
                "allowed-area-mask.json",
            },
        )
        self.assertTrue(all(not Path(item["path"]).is_absolute() for item in manifest["files"]))
        self.assertEqual(len(descriptor["archiveSha256"]), 64)
        execution_route = json.loads(
            (Path(descriptor["manifest"]).parent / "execution-route.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(execution_route["waypoints"][0]["routeProgressIndex"], 0)
        self.assertEqual(
            execution_route["waypoints"][-1]["routeProgressIndex"],
            len(execution_route["waypoints"]) - 1,
        )
        with zipfile.ZipFile(descriptor["archive"]) as archive:
            self.assertEqual(
                set(archive.namelist()),
                {"manifest.json"} | {item["path"] for item in manifest["files"]},
            )


if __name__ == "__main__":
    unittest.main()
