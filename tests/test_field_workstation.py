from __future__ import annotations

import json
import hashlib
import shutil
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from gogoguard_field_workstation import FieldWorkstationApplication
from gogoguard_field_workstation.robot_client import RobotClient


SESSION_ID = "20260806T010203Z-1234abcd"
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class FakeRobot:
    def __init__(self, source: Path, session: dict) -> None:
        self.source = source
        self.session_value = session

    def sessions(self):
        return [dict(self.session_value)]

    def download_recording(self, session_id, destination, report):
        self.assert_session(session_id)
        shutil.copytree(self.source, destination, dirs_exist_ok=True)
        report(progress=30, message="copied", bytes_transferred=1, bytes_total=1)
        return destination

    @staticmethod
    def assert_session(session_id):
        if session_id != SESSION_ID:
            raise AssertionError(session_id)


class FieldWorkstationTest(unittest.TestCase):
    def test_workstation_entrypoints_include_calibration_dependency(self) -> None:
        native = (
            REPOSITORY_ROOT / "deployment/workstation/run-native"
        ).read_text(encoding="utf-8")
        dockerfile = (
            REPOSITORY_ROOT / "deployment/workstation/Dockerfile"
        ).read_text(encoding="utf-8")
        manifest = json.loads(
            (
                REPOSITORY_ROOT / "architecture/modules/field_workstation.json"
            ).read_text(encoding="utf-8")
        )

        self.assertIn("modules/calibration", native)
        self.assertIn("modules/calibration", dockerfile)
        self.assertIn("calibration", manifest["dependsOn"])

    def test_robot_observability_surfaces_are_proxied(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            app = FieldWorkstationApplication(
                data_root=Path(temporary),
                robot={"base_url": "http://127.0.0.1:9"},
                cloud={},
                map_worker="demo",
            )
            requested = []

            def get(path):
                requested.append(path)
                return {"path": path}

            app.robot.get = get
            self.assertEqual(app.capabilities()["path"], "api/v1/capabilities")
            self.assertEqual(app.interaction_status()["path"], "api/v1/interaction")
            self.assertEqual(app.platform_status()["path"], "api/v1/platform")
            self.assertEqual(
                requested,
                [
                    "api/v1/capabilities",
                    "api/v1/interaction",
                    "api/v1/platform",
                ],
            )

    def test_local_inspection_builds_operator_owned_mission_from_audit(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            app = FieldWorkstationApplication(
                data_root=Path(temporary),
                robot={"base_url": "http://127.0.0.1:9"},
                cloud={},
                map_worker="demo",
            )
            app.maps.get = lambda _job_id: {}
            app.routes.prepare_map_job = lambda _job_id: {}
            app.routes.checkpoint_descriptor = lambda _job_id: {
                "mapVersion": "map-123456789abc",
                "routeId": "route-r7",
                "audit": {"ready": True},
                "checkpoints": [
                    {
                        "checkpointId": "cp_01",
                        "routeProgressIndex": 33,
                        "spin": True,
                        "dwellSec": 3,
                    },
                    {
                        "checkpointId": "cp_02",
                        "routeProgressIndex": 57,
                        "spin": True,
                        "dwellSec": 3,
                    },
                ],
            }
            app.navigation_workspaces.get = lambda _job_id: {"revision": 7}
            app.navigation_status = lambda: {}
            posted = []
            app.robot.post = lambda path, body, **kwargs: posted.append(
                (path, body, kwargs)
            ) or {"state": "accepted"}
            result = app.start_local_inspection("map-123456789abc")
            self.assertEqual(result["state"], "accepted")
            path, body, _ = posted[0]
            self.assertEqual(path, "api/v1/navigation/patrol/start-selected")
            mission = body["mission_plan"]
            self.assertEqual(mission["decisionMode"], "local_operator")
            self.assertEqual(
                [item["routeProgressIndex"] for item in mission["checkpoints"]],
                [33, 57],
            )
            self.assertEqual(
                [item["spin"] for item in mission["checkpoints"]],
                [False, False],
            )

    def test_local_inspection_retry_reuses_unstarted_runtime_mission(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            app = FieldWorkstationApplication(
                data_root=Path(temporary),
                robot={"base_url": "http://127.0.0.1:9"},
                cloud={},
                map_worker="demo",
            )
            app.checkpoint_audit = lambda _job_id: {
                "mapVersion": "map-123456789abc",
                "routeId": "route-r7",
                "audit": {"ready": True},
                "checkpoints": [
                    {"checkpointId": "cp_01", "routeProgressIndex": 33}
                ],
            }
            app.navigation_workspaces.get = lambda _job_id: {"revision": 7}
            app.navigation_status = lambda: {
                "runtime_process": {"running": True},
                "runtime": {
                    "runtimeInstanceId": "generation-new",
                    "state": "READY",
                    "motionAuthorized": False,
                    "mapVersion": "map-123456789abc",
                    "routeId": "route-r7",
                    "checkpoint": {
                        "missionId": "local:map-123456789abc:7:existing",
                        "decisionMode": "local_operator",
                        "phase": "TRAVELING",
                        "completedCheckpointCount": 0,
                        "activeCheckpointId": None,
                    },
                },
            }
            posted = []
            app.robot.post = lambda path, body, **kwargs: posted.append(
                (path, body, kwargs)
            ) or {"state": "accepted"}

            app.start_local_inspection("map-123456789abc")

            self.assertEqual(
                posted[0][1]["mission_plan"]["missionId"],
                "local:map-123456789abc:7:existing",
            )

    def test_checkpoint_reference_is_resolved_only_inside_recording(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            app = FieldWorkstationApplication(
                data_root=root,
                robot={"base_url": "http://127.0.0.1:9"},
                cloud={},
                map_worker="demo",
            )
            sample = root / "recordings" / SESSION_ID / "samples" / "inspection" / "cp_01-main.jpg"
            sample.parent.mkdir(parents=True)
            sample.write_bytes(b"\xff\xd8reference\xff\xd9")
            (sample.parent / "checkpoints.json").write_text(
                json.dumps(
                    {
                        "checkpoints": [
                            {
                                "checkpointId": "cp_01",
                                "sampleFrames": ["samples/inspection/cp_01-main.jpg"],
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            app.checkpoint_audit = lambda _job_id: {
                "recordingSessionId": SESSION_ID,
                "checkpoints": [{"checkpointId": "cp_01"}],
            }
            self.assertEqual(
                app.checkpoint_reference_file("map-123456789abc", "cp_01"),
                sample.resolve(),
            )

    def test_map_jobs_are_newest_first_and_labels_are_local_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            app = FieldWorkstationApplication(
                data_root=Path(temporary),
                robot={"base_url": "http://127.0.0.1:9"},
                cloud={},
                map_worker="demo",
            )
            jobs = [
                {"job_id": "map-aaaaaaaaaaaa", "created_at": "2026-08-08T01:00:00+00:00"},
                {"job_id": "map-bbbbbbbbbbbb", "created_at": "2026-08-08T02:00:00+00:00"},
            ]
            app.maps.list = lambda: jobs
            app.maps.get = lambda job_id: next(
                item for item in jobs if item["job_id"] == job_id
            )
            self.assertEqual(
                [item["job_id"] for item in app.map_jobs()],
                ["map-bbbbbbbbbbbb", "map-aaaaaaaaaaaa"],
            )
            updated = app.update_map_label("map-bbbbbbbbbbbb", "一楼大厅")
            self.assertEqual(updated["label"], "一楼大厅")
            reloaded = FieldWorkstationApplication(
                data_root=Path(temporary),
                robot={"base_url": "http://127.0.0.1:9"},
                cloud={},
                map_worker="demo",
            )
            reloaded.maps.get = app.maps.get
            self.assertEqual(
                reloaded.map_job("map-bbbbbbbbbbbb")["label"], "一楼大厅"
            )

    def test_incident_download_resumes_and_verifies_hash(self) -> None:
        content = b"abcdefgh"
        digest = hashlib.sha256(content).hexdigest()

        class Response:
            status = 206

            def __init__(self):
                self.values = [b"efgh", b""]

            def __enter__(self): return self
            def __exit__(self, *_args): return False
            def read(self, _size): return self.values.pop(0)

        descriptor = {
            "incident": {
                "schema": "gogoguard.incident_bundle.v1",
                "incident_id": "incident-20260808T010203Z-1234abcd",
                "state": "sealed",
                "files": [{"path": "replay.json", "bytes": len(content), "sha256": digest}],
            },
            "files": [{"path": "replay.json", "bytes": len(content), "sha256": digest}],
        }
        with tempfile.TemporaryDirectory() as temporary:
            destination = Path(temporary) / "incident"
            destination.mkdir()
            (destination / ".replay.json.part").write_bytes(b"abcd")
            client = RobotClient("http://127.0.0.1:9")
            client.get = lambda _path: descriptor
            with patch(
                "gogoguard_field_workstation.robot_client.urlopen",
                return_value=Response(),
            ) as opened:
                client.download_incident(descriptor["incident"]["incident_id"], destination)
            self.assertEqual((destination / "replay.json").read_bytes(), content)
            request = opened.call_args.args[0]
            self.assertEqual(request.headers["Range"], "bytes=4-")

    def test_robot_post_accepts_an_operation_specific_timeout(self) -> None:
        class Response:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            @staticmethod
            def read():
                return b'{}'

        client = RobotClient("http://127.0.0.1:9", timeout_s=0.01)
        with patch(
            "gogoguard_field_workstation.robot_client.urlopen",
            return_value=Response(),
        ) as opened:
            self.assertEqual(
                client.post("api/v1/slow-operation", {}, timeout_s=30.0), {}
            )
        self.assertEqual(opened.call_args.kwargs["timeout"], 30.0)

    def test_historical_robot_recording_becomes_workstation_map_version(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "robot-recording"
            samples = source / "samples"
            samples.mkdir(parents=True)
            (samples / "snapshots.jsonl").write_text(
                json.dumps(
                    {
                        "pose": {"x": 0, "y": 0, "z": 0},
                        "points": [[0, 0, 0], [1, 0, 0], [1, 1, 0]],
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            session = {
                "schema": "gogoguard.recording_session.v1",
                "session_id": SESSION_ID,
                "site_id": "site-a",
                "robot_id": "robot-a",
                "state": "sealed",
                "started_at": "2026-08-06T01:02:03+00:00",
                "stopped_at": "2026-08-06T01:03:03+00:00",
                "root": "/robot/path",
                "sample_count": 3,
                "error": None,
                "bundle_manifest": "/robot/path/recording_bundle.json",
                "map_job_id": "map-oldrobot001",
            }
            (source / "session.json").write_text(json.dumps(session), encoding="utf-8")
            app = FieldWorkstationApplication(
                data_root=root / "workstation",
                robot={"base_url": "http://127.0.0.1:9"},
                cloud={},
                map_worker="demo",
            )
            app.robot = FakeRobot(source, session)
            outcome = app.submit_recording(SESSION_ID)
            job_id = outcome["map_job"]["job_id"]
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                job = app.map_job(job_id)
                if job["state"] == "complete":
                    break
                time.sleep(0.02)
            else:
                self.fail("historical recording map did not complete")
            linked = next(item for item in app.sessions() if item["session_id"] == SESSION_ID)
            self.assertEqual(linked["map_job_id"], job_id)
            self.assertTrue((root / "workstation" / "recordings" / SESSION_ID / "session.json").is_file())


if __name__ == "__main__":
    unittest.main()
