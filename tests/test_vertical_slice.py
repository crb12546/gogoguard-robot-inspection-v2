from __future__ import annotations

import hashlib
import json
import tempfile
import time
import unittest
from pathlib import Path

from gogoguard_site_console import InspectionApplication


TOPICS = {"lidar": "/mapping/livox/lidar", "imu": "/mapping/livox/imu", "odometry": "/Odometry"}


class VerticalSliceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.app = InspectionApplication(data_root=self.root, mode="demo", map_worker="demo",
                                         robot_id="demo-go2", site_id="demo-site", topics=TOPICS)
        self.app.start()
        time.sleep(0.15)

    def tearDown(self) -> None:
        self.app.close()
        self.temp.cleanup()

    def test_live_record_seal_and_map(self) -> None:
        live = self.app.live()
        self.assertTrue(live["device"]["online"])
        self.assertGreater(len(live["points"]), 100)

        session = self.app.start_recording()
        time.sleep(0.45)
        outcome = self.app.stop_recording(session["session_id"])
        sealed = outcome["session"]
        self.assertEqual(sealed["state"], "sealed")
        self.assertGreaterEqual(sealed["sample_count"], 2)

        manifest_path = Path(sealed["bundle_manifest"])
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        sample_entry = next(item for item in manifest["files"] if item["path"] == "samples/snapshots.jsonl")
        sample_path = manifest_path.parent / sample_entry["path"]
        self.assertEqual(hashlib.sha256(sample_path.read_bytes()).hexdigest(), sample_entry["sha256"])

        job_id = outcome["map_job"]["job_id"]
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            job = self.app.map_job(job_id)
            if job["state"] in {"complete", "failed"}:
                break
            time.sleep(0.1)
        self.assertEqual(job["state"], "complete", job.get("error"))
        artifact_root = Path(job["artifact_root"])
        self.assertTrue((artifact_root / "map.ply").is_file())
        self.assertTrue((artifact_root / "map.json").is_file())
        self.assertTrue((artifact_root / "overview.svg").is_file())
        self.assertEqual(job["metrics"]["worker"], "demo")

    def test_rejects_two_active_recordings(self) -> None:
        self.app.start_recording()
        with self.assertRaises(Exception):
            self.app.start_recording()
