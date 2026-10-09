#!/usr/bin/env python3
"""Build the file:// snapshot from the canonical structured audit models."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    "system": ROOT / "system-model" / "system.json",
    "teaching": ROOT / "system-model" / "teaching.json",
    "diagnostics": ROOT / "system-model" / "diagnostics.json",
    "health": ROOT / "system-model" / "repository-health.json",
}
TARGET = ROOT / "system-guide" / "assets" / "model-snapshot.js"


def main() -> None:
    model = {
        name: json.loads(path.read_text(encoding="utf-8"))
        for name, path in SOURCES.items()
    }
    encoded = json.dumps(model, ensure_ascii=False, separators=(",", ":"))
    TARGET.write_text(
        "/* Generated from system-model/*.json; do not edit by hand. */\n"
        f"window.GOGOGUARD_GUIDE_MODEL={encoded};\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
