from __future__ import annotations

import sys
from pathlib import Path


root = Path(sys.argv[1])
required = {
    "index.html": [
        "trajectory", "cloud", "cameraFrame", "recordButton", "app.js",
        "巡检操作向导", "参数中心", "诊断中心", "progressTimeout",
        "maxForwardSpeed", "acceleration", "deceleration",
        "replanInterval", "obstructionCost",
        "obstructionConfirmation", "停止巡检并释放遥控权",
        "发布所选地图与路线到机器狗", "自动清理并恢复",
        "研发事故记录", "incidentTimeline", "diagnosticMode",
        "巡检地图工作台", "workspaceMap", "startGlimEditor",
        "editRoute", "editAllowedArea", "saveWorkspace",
        "gimbalPanTarget", "gimbalTiltTarget", "gimbalApplyTarget",
        "workspaceZoomIn", "workspaceFit", "workspaceFocusRoute",
        "workspacePan", "workspaceFullscreen", "workspaceCloud",
        "workspace3dSlice", "workspaceView2d", "workspaceView3d",
        "workspaceViewSplit", "showWorkspaceGround",
        "workspaceZoomLabel", "workspaceModeBanner", "finishWorkspaceEdit",
        "擦除红色（常用）", "全屏编辑（含工具）", "实测地面",
    ],
    "app.js": [
        "/api/v1/live", "/api/v1/camera", "/api/v1/sessions/start",
        "map-jobs", "map_job_id", "drawCloud",
        "/api/v1/navigation/runtime/recover",
        "/api/v1/navigation/diagnostics", "/api/v1/navigation/profile",
        "mapSelectionExplicit", "candidateJobId",
        "巡检请求已提交", "规划绕障中", "NAV2_GLOBAL_PATH_USING_MPPI",
        "/api/v1/incidents", "/api/v1/diagnostics/profile", "drawIncidentPlan",
        "save-map-name", "fmtLocalTime", "运行旧地图",
        "profile.remaining_patrols <= 1 ? 'next1' : 'next3'",
        "profile.recovery.progressTimeoutS",
        "profile.recovery.replanIntervalS",
        "profile.motion.targetCruiseMps",
        "profile.motion.maxForwardMps",
        "profile.avoidance.obstructionCostThreshold",
        "profile.avoidance.obstructionConfirmationS",
        "DETOURING", "REJOINING", "RETRYING", "RECOVERING",
        "SEARCHING_PATH", "activeController",
        "/navigation-workspace", "/glim-editor/start", "/glim-editor/publish",
        "robotRadiusM", "allowedArea", "candidateGeneration >= 9",
        "/navigation-surface", "/navigation-plan-preview",
        "visibleNavigationSurfaceCells", "planPreviewResult",
        "candidate.workspace_hash === state.navigationWorkspace.workspaceHash",
        "submitGimbalTarget", "state.gimbalTarget",
        "workspaceViewBounds", "zoomWorkspace", "drawWorkspaceCloud",
        "workspaceGroundModel", "drawWorkspaceGround2d",
        "setWorkspaceDisplayMode",
        "requireWorkspaceEditZoom", "startSurfaceEdit",
        "finishWorkspaceEditing", "mapWorkbenchSection", "附近没有点云",
    ],
    "styles.css": [
        ".workspace", ".camera-panel", ".control-panel", ".action-help",
        "[hidden]", "@media",
        ".incident-lab", ".incident-views",
        ".map-workbench", ".workspace-legend", ".allowed-area-swatch",
        ".obstacle-swatch", ".preview-line", ".navigation-surface-editor",
        ".workspace-3d-reference", ".workspace-coordinate",
        ".workspace-view-tabs", ".ground-measured-swatch",
        "#mapWorkbenchSection:fullscreen", ".workspace-mode-banner",
        ".clear-swatch", ".surface-edit-guide", ".pan-mode",
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
retired = ("detourSpeed", "stopZoneLength", "slowZoneLength", "SlowZone", "DetourPath")
for name in ("index.html", "app.js"):
    content = (root / name).read_text(encoding="utf-8")
    for needle in retired:
        if needle in content:
            raise SystemExit(f"{root / name}: still exposes retired {needle}")
print("site console UI contract: ok")
