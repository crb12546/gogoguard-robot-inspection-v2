import json
import tempfile
import unittest
from pathlib import Path
from subprocess import CompletedProcess
from unittest.mock import patch

from gogoguard_contracts import MapJob, MapJobState
from gogoguard_map_factory import GlimEditorError, GlimEditorManager, MapJobManager


class Journal:
    def __init__(self):
        self.events = []

    def append(self, name, **values):
        self.events.append((name, values))


class GlimEditorTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.journal = Journal()
        self.maps = MapJobManager(self.root, "ssh", self.journal, {})
        self.parent_id = "map-123456789abc"
        parent = MapJob(
            job_id=self.parent_id,
            session_id="recording-1",
            state=MapJobState.COMPLETE,
            progress=100,
            stage="complete",
        )
        self.maps._jobs[self.parent_id] = parent
        self.maps._save(parent)

    def tearDown(self):
        self.temporary.cleanup()

    def _artifact(self, job_id="map-abcdef123456"):
        root = self.root / "map-jobs" / job_id / "artifacts"
        root.mkdir(parents=True)
        (root / "map.json").write_text(
            json.dumps(
                {
                    "source": "cloud-glim",
                    "parent_map_job_id": self.parent_id,
                    "editor": "official-glim-map-editor",
                    "points": [[0.0, 0.0, 0.0, 0.7]],
                }
            ),
            encoding="utf-8",
        )
        (root / "map.ply").write_bytes(b"ply\n")
        (root / "overview.svg").write_text("<svg/>", encoding="utf-8")
        (root / "glim-build.json").write_text("{}", encoding="utf-8")
        (root / "trajectory-poses.json").write_text(
            json.dumps(
                {
                    "schema": "gogoguard.optimized_trajectory.v1",
                    "frame": "map",
                    "poses": [
                        {
                            "timestamp": 1.0,
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0,
                            "qx": 0.0,
                            "qy": 0.0,
                            "qz": 0.0,
                            "qw": 1.0,
                        },
                        {
                            "timestamp": 2.0,
                            "x": 1.0,
                            "y": 0.0,
                            "z": 0.0,
                            "qx": 0.0,
                            "qy": 0.0,
                            "qz": 0.0,
                            "qw": 1.0,
                        },
                    ],
                }
            ),
            encoding="utf-8",
        )
        return root

    def test_cleaned_map_is_registered_as_a_new_immutable_child(self):
        job_id = "map-abcdef123456"
        value = self.maps.import_edited(self.parent_id, job_id, self._artifact(job_id))
        self.assertEqual(value["job_id"], job_id)
        self.assertEqual(value["state"], "complete")
        self.assertEqual(value["metrics"]["parent_map_job_id"], self.parent_id)
        self.assertEqual(self.maps.get(self.parent_id).state, MapJobState.COMPLETE)
        self.assertIn("map.edited", [name for name, _ in self.journal.events])

    def test_edited_map_rejects_untrusted_provenance(self):
        job_id = "map-abcdef123456"
        root = self._artifact(job_id)
        value = json.loads((root / "map.json").read_text(encoding="utf-8"))
        value["editor"] = "unknown"
        (root / "map.json").write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(Exception, "provenance"):
            self.maps.import_edited(self.parent_id, job_id, root)

    def test_edited_map_rejects_invalid_optimized_trajectory(self):
        job_id = "map-abcdef123456"
        root = self._artifact(job_id)
        (root / "trajectory-poses.json").write_text(
            json.dumps(
                {
                    "schema": "gogoguard.optimized_trajectory.v1",
                    "frame": "map",
                    "poses": [
                        {
                            "timestamp": 1.0,
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0,
                            "qx": 0.0,
                            "qy": 0.0,
                            "qz": 0.0,
                            "qw": 1.0,
                        },
                        {
                            "timestamp": 1.0,
                            "x": 1.0,
                            "y": 0.0,
                            "z": 0.0,
                            "qx": 0.0,
                            "qy": 0.0,
                            "qz": 0.0,
                            "qw": 1.0,
                        },
                    ],
                }
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(Exception, "pose is invalid"):
            self.maps.import_edited(self.parent_id, job_id, root)

    def test_edited_map_rejects_non_object_trajectory_contract(self):
        job_id = "map-abcdef123456"
        root = self._artifact(job_id)
        (root / "trajectory-poses.json").write_text("[]", encoding="utf-8")
        with self.assertRaisesRegex(Exception, "contract is invalid"):
            self.maps.import_edited(self.parent_id, job_id, root)

    def test_editor_json_parser_ignores_tool_banner_lines(self):
        value = GlimEditorManager._last_json(
            "systemd banner\n{\"sessionId\":\"edit-123456789abc\",\"state\":\"active\"}\n"
        )
        self.assertEqual(value["state"], "active")
        with self.assertRaises(GlimEditorError):
            GlimEditorManager._last_json("no json here")

    def test_start_reconnects_existing_cloud_session_after_mac_restart(self):
        editor = GlimEditorManager(
            self.root,
            self.maps,
            "ssh",
            {
                "host": "cloud.example",
                "user": "root",
                "remote_root": "/opt/go2/jobs",
            },
        )
        descriptor = {
            "schema": "gogoguard.glim_editor_session.v1",
            "sourceJobId": self.parent_id,
            "sessionId": "edit-abcdef123456",
            "state": "active",
            "webPort": 16123,
            "localPort": 49123,
            "url": "http://127.0.0.1:49123/vnc.html",
        }
        editor._descriptor_path(self.parent_id).write_text(
            json.dumps(descriptor), encoding="utf-8"
        )
        status = CompletedProcess(
            args=[],
            returncode=0,
            stdout='{"sessionId":"edit-abcdef123456","state":"active"}\n',
        )
        with patch.object(editor, "_run", return_value=status) as run, patch.object(
            editor, "_start_tunnel", return_value=(object(), 49200)
        ) as tunnel:
            value = editor.start(self.parent_id)
        self.assertEqual(value["sessionId"], "edit-abcdef123456")
        self.assertEqual(value["localPort"], 49200)
        self.assertIn("127.0.0.1:49200", value["url"])
        self.assertEqual(run.call_args.args[0][-2:], ["status", "edit-abcdef123456"])
        tunnel.assert_called_once()


if __name__ == "__main__":
    unittest.main()
