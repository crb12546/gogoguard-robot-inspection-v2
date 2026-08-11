from __future__ import annotations

import json
import socket
import tempfile
import threading
import unittest
from pathlib import Path

from gogoguard_platform_edge.pose_stream import (
    InteractionPoseClient,
    build_mission_pose_context,
    build_pose_stream_payload,
)


class PlatformPoseStreamTest(unittest.TestCase):
    def test_payload_is_map_and_route_bound_without_fake_values(self) -> None:
        payload = build_pose_stream_payload(
            robot_id="LLYJ0001",
            sequence=8,
            map_version="map-6855ba54ae11",
            route_id="route-6855ba54ae11-workspace-r4",
            frame_id="map",
            x=3.1,
            y=-2.0,
            z=0.2,
            yaw_rad=1.4,
            source_at="2026-08-11T08:00:00.123+00:00",
            localization={"usable": True, "confidence": 0.82, "reason": "TRACKING"},
        )
        self.assertEqual(payload["schema"], "gogoguard.robot_pose.v1")
        self.assertEqual(payload["mapVersion"], "map-6855ba54ae11")
        self.assertEqual(payload["routeId"], "route-6855ba54ae11-workspace-r4")
        self.assertEqual(payload["pose"]["position"]["x"], 3.1)
        self.assertTrue(payload["localization"]["usable"])
        with self.assertRaisesRegex(ValueError, "not finite"):
            build_pose_stream_payload(
                robot_id="LLYJ0001",
                sequence=9,
                map_version="map-6855ba54ae11",
                route_id="route-6855ba54ae11-workspace-r4",
                frame_id="map",
                x=float("nan"),
                y=0.0,
                z=0.0,
                yaw_rad=0.0,
                source_at="2026-08-11T08:00:00.123+00:00",
                localization={"usable": True},
            )

    def test_unix_client_uses_the_frozen_internal_action(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "control.sock"
            received: list[dict] = []
            ready = threading.Event()

            def server() -> None:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
                    listener.bind(str(socket_path))
                    listener.listen(1)
                    ready.set()
                    connection, _ = listener.accept()
                    with connection:
                        raw = connection.makefile("rb").readline()
                        received.append(json.loads(raw.decode("utf-8")))
                        connection.sendall(
                            b'{"ok":true,"result":{"accepted":true}}\n'
                        )

            thread = threading.Thread(target=server, daemon=True)
            thread.start()
            self.assertTrue(ready.wait(2))
            payload = {"schema": "gogoguard.robot_pose.v1"}
            self.assertTrue(InteractionPoseClient(socket_path).publish(payload))
            thread.join(timeout=2)
            self.assertEqual(
                received,
                [{"action": "publish_pose", "payload": payload}],
            )

    def test_mission_context_exposes_checkpoint_phase_and_camera(self) -> None:
        mission = build_mission_pose_context(
            {
                "checkpoint": {
                    "missionId": "mission-1",
                    "activeCheckpointId": "cp_01",
                    "phase": "SPINNING",
                    "camera": {"pan": -15.0, "tilt": 22.5},
                }
            },
            {"camera": {"pan": -14.8, "tilt": 22.4, "roll": 0.0}},
            spin_progress_rad=3.14,
        )
        self.assertEqual(mission["phase"], "spinning")
        self.assertEqual(mission["checkpointId"], "cp_01")
        self.assertEqual(mission["camera"]["tilt"], 22.4)
        self.assertEqual(mission["spinProgressRad"], 3.14)


if __name__ == "__main__":
    unittest.main()
