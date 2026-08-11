import json
import math
import struct
import tempfile
import unittest
import zipfile
from datetime import datetime, timedelta, timezone
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
        optimized_poses = []
        self.trajectory_start = datetime(2026, 8, 11, 1, 2, 3, tzinfo=timezone.utc)
        for index in range(21):
            trajectory.append([index * 0.10, 0.0, 0.0])
            yaw = math.radians(-72.447) if index == 10 else 0.0
            optimized_poses.append(
                {
                    "timestamp": self.trajectory_start.timestamp() + index * 0.1,
                    "x": index * 0.10,
                    "y": 0.0,
                    "z": 0.0,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": math.sin(yaw / 2.0),
                    "qw": math.cos(yaw / 2.0),
                }
            )
        # The final vertical posture change has no planar motion and must not
        # create a route tail.
        for index in range(10):
            trajectory.append([2.0, 0.0, -index * 0.02])
            optimized_poses.append(
                {
                    "timestamp": self.trajectory_start.timestamp() + (21 + index) * 0.1,
                    "x": 2.0,
                    "y": 0.0,
                    "z": -index * 0.02,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": 0.0,
                    "qw": 1.0,
                }
            )
        (self.artifacts / "map.json").write_text(
            json.dumps({"source": "cloud-glim", "trajectory": trajectory}),
            encoding="utf-8",
        )
        (self.artifacts / "trajectory-poses.json").write_text(
            json.dumps(
                {
                    "schema": "gogoguard.optimized_trajectory.v1",
                    "frame": "map",
                    "poses": optimized_poses,
                }
            ),
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
        self.assertEqual(candidate["candidate_generation"], 8)
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
        self.assertEqual(len(second["source_trajectory_poses_sha256"]), 64)
        self.assertIsNone(second["source_checkpoint_sha256"])

        rebound = RouteManager(self.root, site_id="other-site").prepare_map_job(
            self.job_id
        )
        self.assertEqual(rebound["site_id"], "other-site")

    def test_prepare_rebuilds_when_source_checkpoint_asset_changes(self):
        manager = RouteManager(self.root, site_id="test-site")
        first = manager.prepare_map_job(self.job_id)
        self.assertIsNone(first["checkpoint_asset"])

        self.artifacts.chmod(0o750)
        source = self.artifacts / "checkpoints.json"
        source.write_text('{"schema":"legacy.checkpoints.v1","revision":1}\n')
        self.artifacts.chmod(0o550)
        second = manager.prepare_map_job(self.job_id)

        self.assertIsNotNone(second["checkpoint_asset"])
        self.assertEqual(
            Path(second["checkpoint_asset"]).read_bytes(), source.read_bytes()
        )
        self.assertEqual(len(second["source_checkpoint_sha256"]), 64)

        Path(second["checkpoint_asset"]).write_text("tampered\n", encoding="utf-8")
        repaired = manager.prepare_map_job(self.job_id)
        self.assertEqual(
            Path(repaired["checkpoint_asset"]).read_bytes(), source.read_bytes()
        )

        self.artifacts.chmod(0o750)
        source.unlink()
        self.artifacts.chmod(0o550)
        third = manager.prepare_map_job(self.job_id)

        self.assertIsNone(third["checkpoint_asset"])
        self.assertIsNone(third["source_checkpoint_sha256"])
        self.assertFalse(
            (self.root / "navigation" / "candidates" / self.job_id / "checkpoints.json").exists()
        )

    def test_prepare_requires_a_saved_allowed_area(self):
        (self.artifacts.parent / "navigation-workspace.json").unlink()
        with self.assertRaisesRegex(NavigationWorkspaceError, "green allowed area"):
            RouteManager(self.root, site_id="test-site").prepare_map_job(self.job_id)

    def test_legacy_map_without_checkpoints_does_not_require_pose_companion(self):
        self.artifacts.chmod(0o750)
        (self.artifacts / "trajectory-poses.json").unlink()
        self.artifacts.chmod(0o550)

        manager = RouteManager(self.root, site_id="test-site")
        candidate = manager.prepare_map_job(self.job_id)
        checkpoints = manager.checkpoint_descriptor(self.job_id)

        self.assertIsNone(candidate["source_trajectory_poses_sha256"])
        self.assertEqual(checkpoints["checkpoints"], [])

    def test_checkpoint_map_requires_pose_companion(self):
        session_id = "20260811T010203Z-1234abcd"
        (self.artifacts.parent / "job.json").write_text(
            json.dumps({"session_id": session_id}), encoding="utf-8"
        )
        inspection = (
            self.root
            / "recordings"
            / session_id
            / "samples"
            / "inspection"
        )
        inspection.mkdir(parents=True)
        (inspection / "checkpoints.json").write_text(
            json.dumps(
                {
                    "checkpoints": [
                        {"checkpointId": "cp_01", "recordingSampleIndex": 0}
                    ]
                }
            ),
            encoding="utf-8",
        )
        self.artifacts.chmod(0o750)
        (self.artifacts / "trajectory-poses.json").unlink()
        self.artifacts.chmod(0o550)

        manager = RouteManager(self.root, site_id="test-site")
        manager.prepare_map_job(self.job_id)
        with self.assertRaisesRegex(ValueError, "optimized pose timeline"):
            manager.checkpoint_descriptor(self.job_id)

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
                "checkpoints.json",
            },
        )
        self.assertTrue(all(not Path(item["path"]).is_absolute() for item in manifest["files"]))
        self.assertEqual(len(descriptor["archiveSha256"]), 64)
        repeated = manager.export_platform_bundle(self.job_id)
        self.assertEqual(repeated["archiveSha256"], descriptor["archiveSha256"])
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

    def test_recorded_checkpoint_is_bound_to_execution_route_and_sample_is_bundled(self):
        session_id = "20260811T010203Z-1234abcd"
        (self.artifacts.parent / "job.json").write_text(
            json.dumps({"session_id": session_id}), encoding="utf-8"
        )
        recording = self.root / "recordings" / session_id / "samples"
        inspection = recording / "inspection"
        inspection.mkdir(parents=True)
        with (recording / "snapshots.jsonl").open("w", encoding="utf-8") as handle:
            for index in range(21):
                handle.write(
                    json.dumps(
                        {
                            "captured_at": (
                                self.trajectory_start + timedelta(seconds=index * 0.1)
                            ).isoformat(timespec="milliseconds"),
                            "pose": {"x": index * 0.1, "y": 0, "yaw": 0},
                        }
                    )
                    + "\n"
                )
        (inspection / "cp_01-main.jpg").write_bytes(b"\xff\xd8sample\xff\xd9")
        (inspection / "checkpoints.json").write_text(
            json.dumps(
                {
                    "schema": "gogoguard.recording_checkpoints.v1",
                    "sessionId": session_id,
                    "checkpoints": [
                        {
                            "checkpointId": "cp_01",
                            "recordingSampleIndex": 10,
                            "rawPose": {"x": 1.0, "y": 0.0, "yaw": 0.0},
                            "camera": {"pan": -15, "tilt": 22.5, "roll": 0, "frame": "carrier_relative"},
                            "spin": True,
                            "note": "door",
                            "sampleFrames": ["samples/inspection/cp_01-main.jpg"],
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        manager = RouteManager(self.root, site_id="test-site")
        manager.prepare_map_job(self.job_id)
        descriptor = manager.export_platform_bundle(self.job_id)
        checkpoints = json.loads(
            (Path(descriptor["manifest"]).parent / "checkpoints.json").read_text()
        )
        self.assertTrue(checkpoints["audit"]["ready"])
        self.assertEqual(checkpoints["checkpoints"][0]["checkpointId"], "cp_01")
        self.assertEqual(checkpoints["checkpoints"][0]["camera"]["tilt"], 22.5)
        self.assertAlmostEqual(
            math.degrees(checkpoints["checkpoints"][0]["bodyYaw"]),
            -72.447,
            places=3,
        )
        self.assertEqual(
            checkpoints["checkpoints"][0]["binding"]["orientationSource"],
            "glim_quaternion",
        )
        self.assertEqual(
            checkpoints["checkpoints"][0]["binding"]["optimizedTrajectoryIndex"],
            10,
        )
        self.assertLessEqual(
            checkpoints["checkpoints"][0]["binding"]["timestampDeltaS"], 0.001
        )
        with zipfile.ZipFile(descriptor["archive"]) as archive:
            self.assertIn("checkpoints.json", archive.namelist())
            self.assertIn("samples/cp_01-cp_01-main.jpg", archive.namelist())

    def test_checkpoint_binding_fails_closed_without_capture_timestamp(self):
        session_id = "20260811T010203Z-1234abcd"
        (self.artifacts.parent / "job.json").write_text(
            json.dumps({"session_id": session_id}), encoding="utf-8"
        )
        recording = self.root / "recordings" / session_id / "samples"
        inspection = recording / "inspection"
        inspection.mkdir(parents=True)
        (recording / "snapshots.jsonl").write_text(
            json.dumps({"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}}) + "\n",
            encoding="utf-8",
        )
        (inspection / "checkpoints.json").write_text(
            json.dumps(
                {
                    "checkpoints": [
                        {"checkpointId": "cp_01", "recordingSampleIndex": 0}
                    ]
                }
            ),
            encoding="utf-8",
        )
        manager = RouteManager(self.root, site_id="test-site")
        manager.prepare_map_job(self.job_id)
        with self.assertRaisesRegex(ValueError, "no capture timestamp"):
            manager.checkpoint_descriptor(self.job_id)


if __name__ == "__main__":
    unittest.main()
