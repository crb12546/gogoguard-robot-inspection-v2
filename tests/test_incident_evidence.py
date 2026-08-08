from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from gogoguard_contracts import DiagnosticMode, IncidentBundle, IncidentState
from gogoguard_evidence import (
    DiagnosticProfileStore,
    IncidentStore,
    import_legacy_runtime_trace,
    new_incident_id,
    preset_profile,
)
from gogoguard_evidence.incident_recorder import RollingSamples, SerializedSample


class IncidentEvidenceTest(unittest.TestCase):
    def test_rolling_samples_are_bounded_by_time_and_bytes(self) -> None:
        values = RollingSamples()
        values.append(SerializedSample("/cloud", "type", 1_000_000_000, b"1234"), keep_s=2, byte_limit=8)
        values.append(SerializedSample("/cloud", "type", 2_000_000_000, b"5678"), keep_s=2, byte_limit=8)
        values.append(SerializedSample("/cloud", "type", 4_100_000_000, b"abcd"), keep_s=2, byte_limit=8)
        self.assertEqual([item.timestamp_ns for item in values.snapshot()], [4_100_000_000])

    def test_development_profile_falls_back_after_the_requested_patrol(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            store = DiagnosticProfileStore(Path(temporary))
            self.assertEqual(store.get().mode, DiagnosticMode.PRODUCTION)
            value = preset_profile(DiagnosticMode.DEVELOPMENT, remaining_patrols=1)
            active = store.update({**value.__dict__, "mode": value.mode.value})
            self.assertEqual(active.mode, DiagnosticMode.DEVELOPMENT)
            self.assertEqual(active.point_cloud_hz, 10.0)
            fallback = store.consume_patrol()
            self.assertEqual(fallback.mode, DiagnosticMode.PRODUCTION)
            self.assertEqual(fallback.point_cloud_hz, 0.0)
            self.assertFalse(fallback.record_camera)

    def test_incident_files_are_hashed_and_path_traversal_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            store = IncidentStore(root)
            incident_id = new_incident_id()
            incident_root = store.root / incident_id
            incident_root.mkdir()
            (incident_root / "replay.json").write_text('{"schema":"gogoguard.incident_replay.v1"}\n', encoding="utf-8")
            bundle = IncidentBundle(incident_id=incident_id, diagnostic_mode=DiagnosticMode.DEVELOPMENT)
            sealed = store.seal(
                bundle,
                evidence_present=["runtime_decisions"],
                evidence_missing=["camera"],
                state=IncidentState.PARTIAL,
            )
            self.assertEqual(sealed["state"], "partial")
            self.assertEqual(sealed["files"][0]["path"], "replay.json")
            self.assertEqual(len(sealed["files"][0]["sha256"]), 64)
            self.assertEqual(store.file(incident_id, "replay.json").name, "replay.json")
            with self.assertRaises(KeyError):
                store.file(incident_id, "../profile.json")

    def test_legacy_trace_is_explicitly_partial(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "runtime.jsonl"
            records = [
                {"schema": "go2.runtime_trace.v1", "wallTime": 10.0, "stream": "odometry", "payload": {"x": 1, "y": 2, "z": 0, "yaw": 0.3}},
                {"schema": "go2.runtime_trace.v1", "wallTime": 11.0, "stream": "body_cloud", "payload": {"width": 4815, "dataBytes": 154080}},
                {"schema": "go2.runtime_trace.v1", "wallTime": 12.0, "stream": "runtime", "payload": {"state": "BLOCKED", "reason": "LOCAL_PATH_BLOCKED"}},
            ]
            trace.write_text("".join(json.dumps(item) + "\n" for item in records), encoding="utf-8")
            incident = import_legacy_runtime_trace(root / "workstation", [trace])
            self.assertEqual(incident["state"], "partial")
            self.assertIn("point_cloud_xyz", incident["evidence_missing"])
            replay = json.loads((Path(incident["root"]) / "replay.json").read_text(encoding="utf-8"))
            self.assertEqual(replay["last_pose"]["x"], 1)
            self.assertIn("XYZ points were not recorded", replay["limitations"][0])


if __name__ == "__main__":
    unittest.main()
