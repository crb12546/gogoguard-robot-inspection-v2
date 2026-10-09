#!/usr/bin/env python3
"""Static contracts for the evidence-backed, zero-prerequisite system guide."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "system-model"
GUIDE = ROOT / "system-guide"
MODEL_PATHS = {
    "system": MODEL_DIR / "system.json",
    "teaching": MODEL_DIR / "teaching.json",
    "diagnostics": MODEL_DIR / "diagnostics.json",
    "health": MODEL_DIR / "repository-health.json",
}


def evidence_paths(models: dict[str, dict]) -> set[str]:
    system = models["system"]
    teaching = models["teaching"]
    diagnostics = models["diagnostics"]
    health = models["health"]
    paths: set[str] = set()
    for item in system["boundaries"]:
        paths.update(item.get("evidence", []))
    paths.update(system["startupTree"].get("evidence", []))
    for collection, key in (("flows", "evidence"), ("modules", "code")):
        for item in system[collection]:
            paths.update(item.get(key, []))
    for item in system.get("externalDependencies", []):
        paths.update(item.get("evidence", []))
    for lesson in teaching["localizationCourse"]:
        paths.update(lesson.get("evidence", []))
    for lesson in teaching["navigationCourse"]:
        paths.update(lesson.get("evidence", []))
    for algorithm in teaching["algorithmLessons"]:
        paths.update(algorithm.get("codeEvidence", []))
    for parameter in diagnostics["parameters"]:
        paths.update(parameter.get("evidence", []))
    for decision in health["legacyDecisions"]:
        paths.update(decision.get("codeRefs", []))
    return paths


def main() -> None:
    models = {
        name: json.loads(path.read_text(encoding="utf-8"))
        for name, path in MODEL_PATHS.items()
    }
    expected_schemas = {
        "system": "gogoguard.system_model.v1",
        "teaching": "gogoguard.teaching_model.v2",
        "diagnostics": "gogoguard.diagnostics_model.v2",
        "health": "gogoguard.repository_health.v2",
    }
    for name, schema in expected_schemas.items():
        if models[name].get("schema") != schema:
            raise SystemExit(f"unsupported {name} schema")

    index = (GUIDE / "index.html").read_text(encoding="utf-8")
    script = (GUIDE / "assets" / "app.js").read_text(encoding="utf-8")
    styles = (GUIDE / "assets" / "style.css").read_text(encoding="utf-8")
    snapshot = (GUIDE / "assets" / "model-snapshot.js").read_text(encoding="utf-8")

    required_sections = (
        "start", "mastery", "course-localization", "course-navigation",
        "map-atlas", "algorithm-classroom", "field-debugger", "parameter-lab",
        "repo-health", "evidence",
    )
    for section in required_sections:
        if f'id="{section}"' not in index:
            raise SystemExit(f"guide missing section: {section}")
    for lab in ("concept-canvas", "scenario-canvas", "tracking-canvas"):
        if f'id="{lab}"' not in index:
            raise SystemExit(f"guide missing interactive lab: {lab}")
    for behavior in (
        "drawConcept", "renderLocalization", "renderAlgorithms", "renderDebugger",
        "renderParameters", "renderHealth", "drawTracking", "renderExternal",
    ):
        if behavior not in script:
            raise SystemExit(f"guide script missing behavior: {behavior}")
    if "@media" not in styles or "prefers-reduced-motion" not in styles:
        raise SystemExit("guide styles missing responsive/reduced-motion contract")

    prefix = "window.GOGOGUARD_GUIDE_MODEL="
    try:
        payload = snapshot.split(prefix, 1)[1].rsplit(";", 1)[0]
    except IndexError as exc:
        raise SystemExit("guide snapshot prefix is invalid") from exc
    if json.loads(payload) != models:
        raise SystemExit("guide model snapshot is stale")

    # Historical raw receipts stay in ignored local storage. A public clone
    # validates repository evidence without requiring those private artifacts.
    paths = evidence_paths(models)
    private_roots = {"runtime-data", "workstation-data", "release-cache"}
    external_receipts = {
        path for path in paths if Path(path).parts[0] in private_roots
    }
    missing = sorted(
        path for path in paths - external_receipts if not (ROOT / path).exists()
    )
    if missing:
        raise SystemExit("model references missing evidence:\n" + "\n".join(missing))

    system = models["system"]
    allowed_statuses = set(system["statuses"])
    for collection in (
        "boundaries", "flows", "modules", "frames", "mapTypes", "algorithms",
        "parameters", "findings", "legacy", "externalDependencies",
    ):
        for item in system[collection]:
            if item.get("status") not in allowed_statuses:
                raise SystemExit(f"invalid status in {collection}: {item}")

    teaching = models["teaching"]
    for course_name in ("localizationCourse", "navigationCourse"):
        for lesson in teaching[course_name]:
            if len(lesson.get("newTerms", [])) > 2:
                raise SystemExit(f"term budget exceeded: {course_name}/{lesson['id']}")
    if len(teaching["localizationCourse"]) < 9:
        raise SystemExit("localization course is too shallow")
    required_algorithm_fields = {
        "plainProblem", "ifAbsent", "realityInputs", "codeInputs", "processSteps",
        "output", "realityMeaning", "whyWorks", "assumptions", "failures",
        "whySelected", "alternatives", "runtimeStatus", "codeEvidence", "parameters",
    }
    for algorithm in teaching["algorithmLessons"]:
        missing_fields = sorted(required_algorithm_fields - set(algorithm))
        if missing_fields:
            raise SystemExit(f"algorithm lesson incomplete {algorithm['id']}: {missing_fields}")

    diagnostics = models["diagnostics"]
    if len(diagnostics["fieldDebugger"]) < 10:
        raise SystemExit("field reverse debugger lacks required symptom coverage")
    for issue in diagnostics["fieldDebugger"]:
        if len(issue.get("checks", [])) < 3:
            raise SystemExit(f"debugger path too shallow: {issue['id']}")
    parameter_fields = {
        "controls", "current", "default", "increase", "decrease", "improves",
        "sideEffects", "rationale", "provenance", "evidence",
    }
    for parameter in diagnostics["parameters"]:
        missing_fields = sorted(parameter_fields - set(parameter))
        if missing_fields:
            raise SystemExit(f"parameter guide incomplete {parameter['id']}: {missing_fields}")

    health = models["health"]
    legacy_fields = {
        "originalProblem", "currentReplacement", "runtimeStarted", "codeRefs",
        "configRefs", "gitHistory", "deletionRisk", "validationBeforeDelete",
    }
    for decision in health["legacyDecisions"]:
        missing_fields = sorted(legacy_fields - set(decision))
        if missing_fields:
            raise SystemExit(f"legacy audit incomplete {decision['id']}: {missing_fields}")

    print(
        "system guide v2 contract: ok "
        f"({len(teaching['localizationCourse'])} localization lessons, "
        f"{len(teaching['algorithmLessons'])} algorithm classrooms, "
        f"{len(diagnostics['fieldDebugger'])} field symptoms, "
        f"{len(diagnostics['parameters'])} parameter guides, "
        f"{len(health['legacyDecisions'])} legacy decisions)"
    )
    if external_receipts:
        print(
            f"historical external receipts: {len(external_receipts)} references "
            "(not distributed or verified by this check)"
        )


if __name__ == "__main__":
    main()
