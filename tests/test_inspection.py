from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from gogoguard_contracts import InspectionFrame
from gogoguard_inspection import EvidenceFrameBuffer


def frame(frame_id: str) -> InspectionFrame:
    return InspectionFrame(
        frame_id=frame_id,
        mission_id="mission-001",
        checkpoint_id="checkpoint-001",
        view_id="front",
        map_version="map-6855ba54ae11",
        route_id="route-r7",
        captured_at="2026-08-10T00:00:00.000+00:00",
    )


class EvidenceFrameBufferTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_buffers_checksums_evicts_fifo_and_acknowledges(self) -> None:
        buffer = EvidenceFrameBuffer(self.root, max_frames=2, max_bytes=8)
        first = frame("frame-001")
        stored = buffer.store(first, b"aaaa")
        self.assertEqual(stored.frame.byte_count, 4)
        self.assertEqual(len(stored.frame.sha256), 64)
        self.assertEqual(buffer.store(first, b"aaaa").frame.sha256, stored.frame.sha256)

        buffer.store(frame("frame-002"), b"bbbb")
        buffer.store(frame("frame-003"), b"cccc")
        self.assertEqual(
            [item.frame.frame_id for item in buffer.pending()],
            ["frame-002", "frame-003"],
        )
        self.assertEqual(buffer.status()["droppedFrames"], 1)
        self.assertTrue(buffer.acknowledge("frame-002"))
        self.assertFalse(buffer.acknowledge("frame-002"))
        self.assertEqual(buffer.status()["pendingFrames"], 1)

    def test_rejects_hash_conflict_and_detects_payload_tampering(self) -> None:
        buffer = EvidenceFrameBuffer(self.root, max_frames=2, max_bytes=32)
        stored = buffer.store(frame("frame-001"), b"jpeg")
        with self.assertRaisesRegex(ValueError, "different content"):
            buffer.store(frame("frame-001"), b"else")
        stored.payload_path.write_bytes(b"bad!")
        with self.assertRaisesRegex(ValueError, "hash changed"):
            buffer.pending()


if __name__ == "__main__":
    unittest.main()
