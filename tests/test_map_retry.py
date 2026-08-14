from __future__ import annotations

import json
import tempfile
import time
import unittest
from pathlib import Path

from gogoguard_contracts import MapJobState, RecordingSession, RecordingState
from gogoguard_evidence import EventJournal
from gogoguard_map_factory import MapJobManager


class MapRetryTest(unittest.TestCase):
    def test_failed_job_reuses_identity_and_existing_recording(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            recording = root / "recording"
            recording.mkdir()
            session = RecordingSession(
                session_id="20260806T010203Z-1234abcd",
                state=RecordingState.SEALED,
                root=str(recording),
            )
            manager = MapJobManager(
                root, "demo", EventJournal(root / "events.jsonl")
            )
            initial = manager.submit(session)
            failed = self._wait(manager, initial.job_id, MapJobState.FAILED)
            self.assertIn("snapshots.jsonl", failed.error)
            self.assertLess(failed.progress, 100)

            samples = recording / "samples"
            samples.mkdir()
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
            retried = manager.retry(initial.job_id, session)
            self.assertEqual(retried.job_id, initial.job_id)
            complete = self._wait(manager, initial.job_id, MapJobState.COMPLETE)
            self.assertEqual(complete.stage, "complete")

    @staticmethod
    def _wait(manager, job_id: str, expected: MapJobState):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            job = manager.get(job_id)
            if job.state == expected:
                return job
            time.sleep(0.02)
        raise AssertionError(f"map job did not reach {expected}")


if __name__ == "__main__":
    unittest.main()
