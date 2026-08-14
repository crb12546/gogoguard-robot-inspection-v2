from __future__ import annotations

import json
import socket
from pathlib import Path
from typing import Any


def request(path: Path, payload: dict[str, Any], *, timeout_s: float = 20.0) -> dict[str, Any]:
    raw = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
    if len(raw) > 65536:
        raise ValueError("interaction request exceeds 64 KiB")
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout_s)
    try:
        client.connect(str(path))
        client.sendall(raw)
        response = client.makefile("rb").readline(65537)
    finally:
        client.close()
    value = json.loads(response.decode("utf-8"))
    if not value.get("ok"):
        raise RuntimeError(value.get("message", "interaction command failed"))
    return value["result"]
