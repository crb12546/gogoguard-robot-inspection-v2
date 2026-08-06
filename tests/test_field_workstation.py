from __future__ import annotations

import json
import shutil
import tempfile
import time
import unittest
from pathlib import Path

from gogoguard_field_workstation import FieldWorkstationApplication


SESSION_ID = "20260806T010203Z-1234abcd"


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
