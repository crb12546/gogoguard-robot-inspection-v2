from __future__ import annotations

import sys
from pathlib import Path


root = Path(sys.argv[1])
required = {
    "index.html": [
        "trajectory", "cloud", "cameraFrame", "recordButton", "app.js",
        "巡检操作向导", "参数中心", "诊断中心", "progressTimeout",
        "detourSpeed", "mppiRetryLimit", "obstructionCost",
        "发布所选地图与路线到机器狗", "自动清理并恢复",
        "研发事故记录", "incidentTimeline", "diagnosticMode",
    ],
    "app.js": [
        "/api/v1/live", "/api/v1/camera", "/api/v1/sessions/start",
        "map-jobs", "map_job_id", "drawCloud",
        "/api/v1/navigation/runtime/recover",
        "/api/v1/navigation/diagnostics", "/api/v1/navigation/profile",
        "mapSelectionExplicit", "candidateJobId",
        "巡检请求已提交", "规划绕障中", "LOCAL_DETOUR_ACCEPTED",
        "/api/v1/incidents", "/api/v1/diagnostics/profile", "drawIncidentPlan",
        "save-map-name", "fmtLocalTime", "运行旧地图",
        "profile.remaining_patrols <= 1 ? 'next1' : 'next3'",
        "profile.recovery.progressTimeoutS",
        "profile.avoidance.obstructionCostThreshold",
        "DETOURING", "REJOINING", "RETRYING", "activeController",
    ],
    "styles.css": [
        ".workspace", ".camera-panel", ".control-panel", ".action-help",
        "[hidden]", "@media",
        ".incident-lab", ".incident-views",
    ],
}
for name, needles in required.items():
    path = root / name
    if not path.is_file():
        raise SystemExit(f"missing UI asset: {path}")
    content = path.read_text(encoding="utf-8")
    for needle in needles:
        if needle not in content:
            raise SystemExit(f"{path}: missing {needle}")
if "使用当前地图与录制路线" in (root / "index.html").read_text(encoding="utf-8"):
    raise SystemExit("site console still exposes the retired ambiguous action")
print("site console UI contract: ok")
