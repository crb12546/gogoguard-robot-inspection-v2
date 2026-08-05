from __future__ import annotations

import sys
from pathlib import Path


root = Path(sys.argv[1])
required = {
    "index.html": ["trajectory", "cloud", "recordButton", "app.js"],
    "app.js": ["/api/v1/live", "/api/v1/sessions/start", "map-jobs", "drawCloud"],
    "styles.css": [".workspace", ".control-panel", "@media"],
}
for name, needles in required.items():
    path = root / name
    if not path.is_file():
        raise SystemExit(f"missing UI asset: {path}")
    content = path.read_text(encoding="utf-8")
    for needle in needles:
        if needle not in content:
            raise SystemExit(f"{path}: missing {needle}")
print("site console UI contract: ok")
