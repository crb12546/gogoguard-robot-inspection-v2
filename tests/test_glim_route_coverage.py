from __future__ import annotations

import datetime as dt
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path

from gogoguard_map_factory.manager import (
    MapWorkerError,
    _validate_recording_route_coverage,
)


EXPORTER_PATH = (
    Path(__file__).resolve().parents[1]
    / "deployment"
    / "cloud"
    / "export_glim_artifact.py"
)
SPEC = importlib.util.spec_from_file_location("gogoguard_glim_exporter", EXPORTER_PATH)
assert SPEC and SPEC.loader
EXPORTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXPORTER)


class GlimRouteCoverageTest(unittest.TestCase):
    def _fixture(self, root: Path):
        started = 1_000.0
        started_at = dt.datetime.fromtimestamp(
            started, tz=dt.timezone.utc
        ).isoformat()
        snapshots = root / "snapshots.jsonl"
        raw_samples = []
        for index in range(101):
            timestamp = started + 0.05 + index * 0.25
            distance = max(0.0, timestamp - (started + 3.0)) * 0.20
            raw_samples.append(
                {
                    "timestamp": timestamp,
                    "x": distance,
                    "y": 0.04 * math.sin(distance),
                    "z": -0.40,
                }
            )
        snapshots.write_text(
            "".join(
                json.dumps(
                    {
                        "captured_at": dt.datetime.fromtimestamp(
                            sample["timestamp"], tz=dt.timezone.utc
                        ).isoformat(),
                        "pose": {
                            "x": sample["x"],
                            "y": sample["y"],
                            "z": sample["z"],
                        },
                    }
                )
                + "\n"
                for sample in raw_samples
            ),
            encoding="utf-8",
        )
        angle = 0.40
        cosine, sine = math.cos(angle), math.sin(angle)
        optimized = []
        for index in range(201):
            timestamp = started + 5.0 + index * 0.10
            distance = max(0.0, timestamp - (started + 3.0)) * 0.20
            raw_x, raw_y = distance, 0.04 * math.sin(distance)
            optimized.append(
                {
                    "timestamp": timestamp,
                    "x": cosine * raw_x - sine * raw_y + 2.0,
                    "y": sine * raw_x + cosine * raw_y - 1.0,
                    "z": 0.10,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": 0.0,
                    "qw": 1.0,
                }
            )
        return {"started_at": started_at}, snapshots, optimized

    def test_reconstructs_missing_recording_prefix_in_map_frame(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session, snapshots, optimized = self._fixture(Path(temporary))
            trajectory, coverage = EXPORTER.reconstruct_recording_trajectory(
                optimized, session, snapshots
            )

        self.assertTrue(coverage["complete"])
        self.assertAlmostEqual(coverage["optimizedStartGapSec"], 5.0, places=6)
        self.assertLessEqual(coverage["routeStartGapSec"], 0.5)
        self.assertGreater(coverage["preRollPoseCount"], 10)
        self.assertEqual(coverage["alignment"]["method"], "timestamp_matched_se2")
        self.assertLess(coverage["alignment"]["rmsErrorM"], 0.01)
        self.assertAlmostEqual(trajectory[0][0], 2.0, places=2)
        self.assertAlmostEqual(trajectory[0][1], -1.0, places=2)
        self.assertAlmostEqual(trajectory[0][2], 0.10, places=2)
        self.assertGreater(len(trajectory), len(optimized))

    def test_workstation_rejects_missing_coverage_receipt(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            session, snapshots, optimized = self._fixture(root)
            trajectory, coverage = EXPORTER.reconstruct_recording_trajectory(
                optimized, session, snapshots
            )
            map_path = root / "map.json"
            optimized_path = root / "trajectory-poses.json"
            session_path = root / "session.json"
            map_path.write_text(
                json.dumps({"trajectory": trajectory, "routeCoverage": coverage}),
                encoding="utf-8",
            )
            optimized_path.write_text(
                json.dumps({"poses": optimized, "routeCoverage": coverage}),
                encoding="utf-8",
            )
            session_path.write_text(json.dumps(session), encoding="utf-8")
            accepted = _validate_recording_route_coverage(
                map_path, optimized_path, session_path
            )
            self.assertEqual(accepted["preRollPoseCount"], coverage["preRollPoseCount"])

            map_path.write_text(
                json.dumps({"trajectory": trajectory}), encoding="utf-8"
            )
            with self.assertRaisesRegex(MapWorkerError, "does not cover"):
                _validate_recording_route_coverage(
                    map_path, optimized_path, session_path
                )


if __name__ == "__main__":
    unittest.main()
