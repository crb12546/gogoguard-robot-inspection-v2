from __future__ import annotations

import hashlib
import json
import tempfile
import threading
import unittest
from pathlib import Path

from gogoguard_field_workstation.robot_client import RobotClient
from gogoguard_site_console.server import SiteConsoleServer
from gogoguard_transfer import EdgeArtifactExchange, TransferContractError


SESSION_ID = "20260806T010203Z-1234abcd"
JOB_ID = "map-123456789abc"


def digest(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


class EdgeTransferApplication:
    def __init__(self, root: Path) -> None:
        self.data_root = root
        self.exchange = EdgeArtifactExchange(root)

    def recording_export(self, session_id: str) -> dict:
        return self.exchange.recording_descriptor(session_id)

    def recording_export_file(self, session_id: str, name: str) -> Path:
        return self.exchange.recording_file(session_id, name)

    def receive_map_artifact(self, job_id, name, reader, length, sha256):
        return self.exchange.receive_map_artifact(job_id, name, reader, length, sha256)

    def map_import_descriptor(self, job_id: str) -> dict:
        return self.exchange.map_import_descriptor(job_id)

    def commit_map_import(self, job_id: str, payload: dict) -> dict:
        return self.exchange.commit_map_import(job_id, payload)


class EdgeTransferTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        recording = self.root / "recordings" / SESSION_ID
        (recording / "raw" / "rosbag").mkdir(parents=True)
        content = (b"sealed-robot-recording\n" * 8192)
        source = recording / "raw" / "rosbag" / "data.mcap"
        source.write_bytes(content)
        manifest = {
            "schema": "gogoguard.recording_bundle.v1",
            "session_id": SESSION_ID,
            "files": [{
                "path": "raw/rosbag/data.mcap",
                "bytes": len(content),
                "sha256": digest(content),
            }],
        }
        session = {
            "session_id": SESSION_ID,
            "site_id": "site-a",
            "robot_id": "robot-a",
            "state": "sealed",
            "started_at": "2026-08-06T01:02:03+00:00",
            "stopped_at": "2026-08-06T01:03:03+00:00",
            "sample_count": 1,
            "root": str(recording),
            "bundle_manifest": str(recording / "recording_bundle.json"),
            "map_job_id": None,
            "error": None,
        }
        (recording / "recording_bundle.json").write_text(json.dumps(manifest))
        (recording / "session.json").write_text(json.dumps(session))
        self.content = content
        self.exchange = EdgeArtifactExchange(self.root)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_only_manifest_files_from_sealed_recording_are_exported(self) -> None:
        descriptor = self.exchange.recording_descriptor(SESSION_ID)
        self.assertEqual(descriptor["bytes_total"], len(self.content))
        self.assertEqual(
            self.exchange.recording_file(SESSION_ID, "raw/rosbag/data.mcap").read_bytes(),
            self.content,
        )
        with self.assertRaises(TransferContractError):
            self.exchange.recording_file(SESSION_ID, "../../etc/passwd")

    def test_workstation_resumes_download_and_deploys_verified_glim_map(self) -> None:
        static = self.root / "static"
        static.mkdir()
        (static / "index.html").write_text("ok")
        server = SiteConsoleServer(("127.0.0.1", 0), EdgeTransferApplication(self.root), static)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            client = RobotClient(f"http://127.0.0.1:{server.server_port}")
            destination = self.root / "mac-recording"
            part = destination / "raw" / "rosbag" / ".data.mcap.part"
            part.parent.mkdir(parents=True)
            part.write_bytes(self.content[:7777])
            updates = []
            client.download_recording(SESSION_ID, destination, lambda **value: updates.append(value))
            self.assertEqual((destination / "raw" / "rosbag" / "data.mcap").read_bytes(), self.content)
            self.assertGreaterEqual(updates[-1]["progress"], 30)

            artifacts = self.root / "mac-map"
            artifacts.mkdir()
            values = {
                "map.json": json.dumps({"source": "cloud-glim", "points": []}).encode(),
                "map.ply": b"ply\nformat ascii 1.0\nelement vertex 0\nend_header\n",
                "overview.svg": b"<svg xmlns='http://www.w3.org/2000/svg'/>",
                "glim-build.json": b'{"worker":"glim"}',
            }
            for name, value in values.items():
                (artifacts / name).write_bytes(value)
            committed = client.deploy_map(
                {
                    "job_id": JOB_ID,
                    "session_id": SESSION_ID,
                    "created_at": "2026-08-06T01:02:03+00:00",
                    "updated_at": "2026-08-06T01:03:03+00:00",
                    "metrics": {"worker": "cloud-glim"},
                },
                artifacts,
            )
            self.assertEqual(committed["stage"], "deployed_to_robot")
            self.assertTrue((self.root / "map-jobs" / JOB_ID / "artifacts" / "map.ply").is_file())

            # Repeated selection is a no-op transfer and remains successful.
            repeated = client.deploy_map(
                {
                    "job_id": JOB_ID,
                    "session_id": SESSION_ID,
                    "created_at": "2026-08-06T01:02:03+00:00",
                    "updated_at": "2026-08-06T01:03:03+00:00",
                    "metrics": {"worker": "cloud-glim"},
                },
                artifacts,
            )
            self.assertEqual(repeated["stage"], "deployed_to_robot")
            self.assertFalse((self.root / "workstation-imports" / JOB_ID).exists())
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_repeated_staged_artifact_consumes_the_request_body(self) -> None:
        import io

        content = b"map artifact" * 4096
        first = io.BytesIO(content)
        self.exchange.receive_map_artifact(
            JOB_ID, "map.ply", first, len(content), digest(content)
        )
        repeated = io.BytesIO(content)
        self.exchange.receive_map_artifact(
            JOB_ID, "map.ply", repeated, len(content), digest(content)
        )
        self.assertEqual(repeated.tell(), len(content))

    def test_navigation_map_import_rejects_non_glim_artifact(self) -> None:
        values = {
            "map.json": b'{"source":"demo-worker-not-glim"}',
            "map.ply": b"ply",
            "overview.svg": b"<svg/>",
            "glim-build.json": b"{}",
        }
        files = []
        for name, content in values.items():
            self.exchange.receive_map_artifact(
                JOB_ID, name, __import__("io").BytesIO(content), len(content), digest(content)
            )
            files.append({"name": name, "bytes": len(content), "sha256": digest(content)})
        with self.assertRaisesRegex(TransferContractError, "cloud GLIM"):
            self.exchange.commit_map_import(
                JOB_ID,
                {
                    "schema": "gogoguard.workstation_map_deployment.v1",
                    "job_id": JOB_ID,
                    "files": files,
                },
            )


if __name__ == "__main__":
    unittest.main()
