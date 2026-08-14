from __future__ import annotations

import json
import hashlib
import os
import tempfile
import time
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

from gogoguard_field_workstation.platform_assets import PlatformAssetUploader
from gogoguard_platform_edge.checkpoint import CheckpointCoordinator, mission_url


class FakeGimbal:
    def __init__(self) -> None:
        self.moves = []

    def move_for_inspection(self, **angles):
        self.moves.append(angles)
        return {
            "angles": {
                "pan": angles["pan_body_deg"] + 0.2,
                "tilt": angles["tilt_euler_deg"] - 0.1,
                "roll": angles["roll_euler_deg"],
            }
        }


class MisalignedGimbal(FakeGimbal):
    def move_for_inspection(self, **angles):
        self.moves.append(angles)
        return {
            "angles": {
                "pan": angles["pan_body_deg"] + 12.0,
                "tilt": angles["tilt_euler_deg"],
                "roll": angles["roll_euler_deg"],
            }
        }


class CheckpointPlatformTest(unittest.TestCase):
    def test_frozen_mission_urls_are_derived_from_heartbeat(self) -> None:
        heartbeat = "http://39.96.37.187/api/v1/robot/heartbeat"
        self.assertEqual(
            mission_url(heartbeat, "event"),
            "http://39.96.37.187/api/v1/robot/mission/event",
        )

    def test_true_stop_to_capture_to_verdict_writes_navigation_controls(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            control = root / "control.json"
            inbox = root / "inbox.jsonl"
            calls = []

            def poster(url, payload, timeout):
                calls.append((url, dict(payload)))
                if payload.get("phase") == "announcing":
                    return {"ok": True, "announcement": {"proceed": True}}
                if payload.get("phase") == "waiting_verdict":
                    return {
                        "ok": True,
                        "verdict": {
                            "schema": "gogoguard.checkpoint_verdict.v1",
                            "verdictId": "vd_1",
                            "missionId": "mission-1",
                            "mapVersion": "map-v1",
                            "routeId": "route-v1",
                            "checkpointId": "cp_01",
                            "attempt": 1,
                            "action": "continue",
                            "expiresAt": "2099-01-01T00:00:00+00:00",
                        },
                    }
                return {"ok": True}

            gimbal = FakeGimbal()
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=control,
                inbox_path=inbox,
                state_path=root / "state.json",
                post_json=poster,
                gimbal=gimbal,
            )
            runtime = {
                "mapVersion": "map-v1",
                "routeId": "route-v1",
                "routeProgressIndex": 12,
                "checkpoint": {
                    "missionId": "mission-1",
                    "activeCheckpointId": "cp_01",
                    "activeRouteProgressIndex": 12,
                    "attempt": 1,
                    "phase": "WAITING_PLATFORM",
                    "dwellSec": 1,
                    "camera": {"pan": -15, "tilt": 22.5, "roll": 0},
                    "verdictTimeoutSec": 15,
                },
            }
            navigation.write_text(json.dumps({"runtime": runtime}), encoding="utf-8")
            coordinator.tick(now=100.0)
            coordinator.tick(now=101.1)
            self.assertEqual(json.loads(control.read_text())["action"], "capture")
            self.assertEqual(gimbal.moves[0]["pan_body_deg"], -15.0)
            self.assertEqual(
                json.loads((root / "state.json").read_text())["camera"]["pan"],
                -14.8,
            )
            runtime["checkpoint"]["phase"] = "WAITING_VERDICT"
            navigation.write_text(json.dumps({"runtime": runtime}), encoding="utf-8")
            coordinator.tick(now=102.0)
            self.assertEqual(json.loads(control.read_text())["action"], "continue")
            phases = [payload["phase"] for _, payload in calls if "phase" in payload]
            self.assertEqual(
                phases[:7],
                [
                    "checkpoint_reached", "stopped", "pose_ready", "announcing",
                    "capture_ready", "captured", "waiting_verdict",
                ],
            )

    def test_platform_does_not_report_pose_ready_when_gimbal_misses_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            calls = []
            navigation.write_text(
                json.dumps(
                    {
                        "runtime": {
                            "mapVersion": "map-v1",
                            "routeId": "route-v1",
                            "checkpoint": {
                                "missionId": "mission-1",
                                "activeCheckpointId": "cp_01",
                                "activeRouteProgressIndex": 12,
                                "attempt": 1,
                                "phase": "WAITING_PLATFORM",
                                "camera": {"pan": 35, "tilt": 5, "roll": 0},
                            },
                        }
                    }
                ),
                encoding="utf-8",
            )
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=root / "control.json",
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=lambda url, payload, timeout: calls.append(payload) or {"ok": True},
                gimbal=MisalignedGimbal(),
            )
            with self.assertRaisesRegex(RuntimeError, "did not reach checkpoint view"):
                coordinator.tick(now=100.0)
            self.assertNotIn("pose_ready", [item.get("phase") for item in calls])

    def test_platform_coordinator_does_not_take_local_operator_checkpoint(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            navigation = root / "navigation.json"
            control = root / "control.json"
            calls = []
            navigation.write_text(
                json.dumps(
                    {
                        "runtime": {
                            "checkpoint": {
                                "missionId": "local:map-v1:1:abcd",
                                "decisionMode": "local_operator",
                                "activeCheckpointId": "cp_01",
                                "attempt": 1,
                                "phase": "WAITING_PLATFORM",
                            }
                        }
                    }
                ),
                encoding="utf-8",
            )
            coordinator = CheckpointCoordinator(
                heartbeat_url="http://39.96.37.187/api/v1/robot/heartbeat",
                navigation_status_path=navigation,
                control_path=control,
                inbox_path=root / "inbox.jsonl",
                state_path=root / "state.json",
                post_json=lambda *args: calls.append(args) or {},
            )
            coordinator.tick(now=100.0)
            self.assertEqual(calls, [])
            self.assertFalse(control.exists())

    def test_platform_asset_upload_resumes_and_stops_before_activation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            archive = root / "bundle.zip"
            with zipfile.ZipFile(archive, "w") as output:
                output.writestr("manifest.json", "{}")
            manifest = root / "manifest.json"
            manifest.write_text(json.dumps({"files": []}), encoding="utf-8")
            uploader = PlatformAssetUploader(
                root,
                {
                    "base_url": "http://39.96.37.187/api/v1",
                    "allow_insecure_http": True,
                },
            )
            requests = []

            def request(method, path, body=None, raw=None):
                requests.append((method, path))
                if path.endswith("/init"):
                    return {
                        "uploadId": "ab_1", "chunkSize": 8, "totalChunks": 2,
                        "siteId": "site-1", "siteName": "天津大学北洋园",
                        "missing": [0, 1],
                    }
                if "/chunk" in path:
                    index = int(path.rsplit("=", 1)[-1])
                    return {"received": list(range(index + 1)), "missing": []}
                if path.endswith("/complete"):
                    return {"ok": True, "summary": {"checkpoints": 1}}
                raise AssertionError((method, path))

            uploader._request = request
            descriptor = {
                "archive": str(archive),
                "manifest": str(manifest),
                "mapVersion": "map-v1",
                "routeId": "route-v1",
                "archiveBytes": archive.stat().st_size,
                "archiveSha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
            }
            with patch.dict(os.environ, {"GOGOGUARD_DEVICE_TOKEN": "secret"}):
                uploader.start("map-123456789abc", descriptor)
                deadline = time.time() + 2
                while time.time() < deadline and uploader.status("map-123456789abc")["state"] not in {"verified", "failed"}:
                    time.sleep(0.01)
            self.assertEqual(uploader.status("map-123456789abc")["state"], "verified")
            self.assertFalse(any(path.endswith("/activate") for _, path in requests))


if __name__ == "__main__":
    unittest.main()
