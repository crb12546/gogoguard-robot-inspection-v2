from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any

from gogoguard_contracts import json_ready, utc_now


class EventJournal:
    """Append-only local evidence shared by runtime modules."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def append(self, event: str, **details: Any) -> dict[str, Any]:
        record = {
            "schema": "gogoguard.event.v1",
            "event": event,
            "observed_at": utc_now(),
            "details": json_ready(details),
        }
        line = json.dumps(record, ensure_ascii=False, separators=(",", ":"))
        with self._lock, self.path.open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")
            handle.flush()
        return record
