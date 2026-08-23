from __future__ import annotations

import hashlib
from typing import Any


JPEG_SOF_MARKERS = {
    0xC0, 0xC1, 0xC2, 0xC3,
    0xC5, 0xC6, 0xC7,
    0xC9, 0xCA, 0xCB,
    0xCD, 0xCE, 0xCF,
}


def jpeg_metadata(payload: Any) -> dict[str, Any]:
    """Return immutable JPEG identity from SOF, without decoding image pixels."""

    if not isinstance(payload, (bytes, bytearray)):
        raise ValueError("JPEG payload must be bytes")
    content = bytes(payload)
    if len(content) < 4 or not content.startswith(b"\xff\xd8") or not content.endswith(b"\xff\xd9"):
        raise ValueError("JPEG markers are invalid")
    offset = 2
    width = height = None
    while offset < len(content) - 1:
        if content[offset] != 0xFF:
            offset += 1
            continue
        while offset < len(content) and content[offset] == 0xFF:
            offset += 1
        if offset >= len(content):
            break
        marker = content[offset]
        offset += 1
        if marker in {0x00, 0x01, 0xD8, 0xD9} or 0xD0 <= marker <= 0xD7:
            continue
        if offset + 2 > len(content):
            break
        segment_length = int.from_bytes(content[offset:offset + 2], "big")
        if segment_length < 2 or offset + segment_length > len(content):
            raise ValueError("JPEG segment length is invalid")
        if marker in JPEG_SOF_MARKERS:
            if segment_length < 7:
                raise ValueError("JPEG SOF segment is invalid")
            height = int.from_bytes(content[offset + 3:offset + 5], "big")
            width = int.from_bytes(content[offset + 5:offset + 7], "big")
            break
        if marker == 0xDA:
            break
        offset += segment_length
    if width is None or height is None or width <= 0 or height <= 0:
        raise ValueError("JPEG SOF dimensions are unavailable")
    return {
        "mime": "image/jpeg",
        "bytes": len(content),
        "sha256": hashlib.sha256(content).hexdigest(),
        "width": width,
        "height": height,
    }
