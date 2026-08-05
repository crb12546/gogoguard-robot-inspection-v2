"""Human review tasks limited to facts a field operator can actually know."""

from __future__ import annotations

import math
import re
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


Point3 = Tuple[float, float, float]
IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
SHA256 = re.compile(r"^[0-9a-f]{64}$")


TASK_OPTIONS = {
    "object_persistence": {
        "keep_fixed": "场地长期结构",
        "remove_temporary": "本次采集才出现",
        "defer": "我无法确认",
    },
    "traversability": {
        "allow": "允许机器狗进入",
        "forbid": "禁止机器狗进入",
        "defer": "我无法确认",
    },
}


TASK_OPTION_DESCRIPTIONS = {
    "object_persistence": {
        "keep_fixed": "复查现场后确认，它应当进入长期定位地图。",
        "remove_temporary": "复查现场后确认，它不应进入长期定位地图。",
        "defer": "先不猜，稍后到现场重新确认。",
    },
    "traversability": {
        "allow": "物业允许，并且属于本次巡检范围。",
        "forbid": "例如草坪、台阶、危险边缘或物业禁入区。",
        "defer": "先不猜，向物业或现场负责人确认。",
    },
}


# Keep the two kinds of human reasoning in separate batches.  Switching from
# object persistence in a 3D comparison to property access rules on a 2D map
# for every other task is an operator error trap, even if priorities differ.
REVIEW_PHASE_ORDER = {
    "object_persistence": 0,
    "traversability": 1,
}


TRAVERSABILITY_SOURCES = {
    "operator_boundary_check",
    "operator_drawn",
    "property_rule",
    "route_conflict",
}


@dataclass
class ReviewTask:
    kind: str
    title: str
    question: str
    map_center: Point3
    bounds_min: Point3
    bounds_max: Point3
    evidence: Mapping[str, Any]
    priority: int = 50
    task_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    state: str = "pending"
    decision: Optional[str] = None
    note: str = ""
    decided_by: Optional[str] = None
    decided_at: Optional[float] = None
    decision_history: List[Dict[str, Any]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.kind not in TASK_OPTIONS:
            raise ValueError("unsupported review task kind: %s" % self.kind)
        if not self.title.strip() or not self.question.strip():
            raise ValueError("review task title and question are required")
        if self.priority < 0 or self.priority > 100:
            raise ValueError("review task priority must be within [0, 100]")
        if any(
            self.bounds_min[index] > self.bounds_max[index]
            for index in range(3)
        ):
            raise ValueError("review task bounds are inverted")

    @property
    def options(self) -> Mapping[str, str]:
        return TASK_OPTIONS[self.kind]

    def decision_readiness(self) -> Dict[str, Any]:
        """Return whether this is a question a field operator may answer.

        A missing LiDAR return is not proof that an object was absent, and an
        algorithm is not allowed to invent site access rules.  Keep those two
        safety rules in the shared contract so neither the browser nor another
        API client can bypass them.
        """
        if self.kind == "object_persistence":
            comparison = self.evidence.get("acquisitionComparison")
            if not isinstance(comparison, list):
                return {
                    "ready": False,
                    "reason": "多次采集对照证据不完整，应先补录而不是让现场人员猜测",
                }
            states = set()
            acquisition_ids = set()
            state_by_acquisition: Dict[str, str] = {}
            for record in comparison:
                if not isinstance(record, Mapping):
                    continue
                acquisition_id = str(record.get("acquisitionId", "")).strip()
                state = str(record.get("state", ""))
                try:
                    observable = int(record.get("observableVoxelCount"))
                    hits = int(record.get("hitVoxelCount"))
                    component = int(record.get("componentVoxelCount"))
                except (TypeError, ValueError):
                    continue
                if (
                    not acquisition_id
                    or acquisition_id in acquisition_ids
                    or state not in {"present", "clear", "partial"}
                    or component < 1
                    or observable < 0
                    or hits < 0
                    or observable > component
                    or hits > observable
                    or (state == "present" and hits == 0)
                    or (state == "clear" and (observable != component or hits != 0))
                ):
                    continue
                acquisition_ids.add(acquisition_id)
                states.add(state)
                state_by_acquisition[acquisition_id] = state
            if not {"present", "clear"} <= states:
                return {
                    "ready": False,
                    "reason": "必须同时有一次确实看见和一次完整看清但物体不在的证据",
                }
            visual = self.evidence.get("visualComparison")
            if (
                not isinstance(visual, Mapping)
                or visual.get("schema") != "go2.review_visual_comparison.v1"
                or visual.get("frame") != "map"
                or visual.get("cameraContract") != "same_task_bounds_v1"
            ):
                return {
                    "ready": False,
                    "reason": "逐次点云对照尚未生成，不能只看合并地图或文字计数做判断",
                }
            for expected_state in ("present", "clear"):
                source = visual.get(expected_state)
                if not isinstance(source, Mapping):
                    return {
                        "ready": False,
                        "reason": "逐次点云对照缺少出现或缺席视图",
                    }
                acquisition_id = str(source.get("acquisitionId", ""))
                metadata_path = str(source.get("metadataPath", ""))
                try:
                    point_count = int(source.get("pointCount"))
                except (TypeError, ValueError):
                    point_count = 0
                if (
                    not IDENTIFIER.fullmatch(acquisition_id)
                    or state_by_acquisition.get(acquisition_id) != expected_state
                    or source.get("artifact") != "potree"
                    or metadata_path
                    != "comparisons/%s/metadata.json" % acquisition_id
                    or not SHA256.fullmatch(str(source.get("metadataSha256", "")))
                    or not SHA256.fullmatch(str(source.get("sourcePlySha256", "")))
                    or source.get("provenancePath")
                    != "comparisons/%s/provenance.json" % acquisition_id
                    or not SHA256.fullmatch(str(source.get("provenanceSha256", "")))
                    or point_count < 1
                ):
                    return {
                        "ready": False,
                        "reason": "逐次点云视图与采集证据不一致，已禁止现场判断",
                    }
            return {
                "ready": True,
                "reason": "多次采集已经形成同一位置、同一视角的出现与缺席证据",
            }

        source = str(self.evidence.get("source", "")).strip()
        if source not in TRAVERSABILITY_SOURCES:
            return {
                "ready": False,
                "reason": "通行规则没有可靠来源，算法不能凭点云猜测物业禁入范围",
            }
        return {
            "ready": True,
            "reason": "该区域来自物业规则、现场圈选或路线冲突核对",
        }

    def decide(self, decision: str, operator: str, note: str = "") -> None:
        if self.state != "pending":
            raise ValueError("review task must be reopened before another decision")
        if decision not in self.options:
            raise ValueError("unsupported decision for %s" % self.kind)
        readiness = self.decision_readiness()
        if not readiness["ready"]:
            raise ValueError("review evidence is not ready: %s" % readiness["reason"])
        if not operator.strip():
            raise ValueError("operator is required")
        decided_at = time.time()
        self.decision = decision
        self.note = str(note)
        self.decided_by = operator.strip()
        self.decided_at = decided_at
        self.state = "deferred" if decision == "defer" else "decided"
        self.decision_history.append(
            {
                "action": "decision",
                "decision": decision,
                "operator": self.decided_by,
                "note": self.note,
                "at": decided_at,
            }
        )

    def reopen(self, operator: str, reason: str = "") -> None:
        if self.state not in {"decided", "deferred"}:
            raise ValueError("only a decided review task can be reopened")
        normalized_operator = operator.strip()
        if not normalized_operator:
            raise ValueError("operator is required")
        self.decision_history.append(
            {
                "action": "reopen",
                "previousDecision": self.decision,
                "operator": normalized_operator,
                "reason": str(reason),
                "at": time.time(),
            }
        )
        self.state = "pending"
        self.decision = None
        self.note = ""
        self.decided_by = None
        self.decided_at = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "taskId": self.task_id,
            "kind": self.kind,
            "title": self.title,
            "question": self.question,
            "mapCenter": list(self.map_center),
            "bounds": {
                "min": list(self.bounds_min),
                "max": list(self.bounds_max),
            },
            "evidence": dict(self.evidence),
            "priority": self.priority,
            "state": self.state,
            "options": [
                {
                    "value": value,
                    "label": label,
                    "description": TASK_OPTION_DESCRIPTIONS[self.kind][value],
                }
                for value, label in self.options.items()
            ],
            "decisionReadiness": self.decision_readiness(),
            "decision": self.decision,
            "note": self.note,
            "decidedBy": self.decided_by,
            "decidedAt": self.decided_at,
            "decisionHistory": [dict(item) for item in self.decision_history],
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "ReviewTask":
        bounds = payload.get("bounds")
        if not isinstance(bounds, Mapping):
            raise ValueError("review task has no bounds")

        def point3(value: Any, label: str) -> Point3:
            if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
                raise ValueError("%s must be a three-element array" % label)
            parsed = tuple(float(item) for item in value)
            if len(parsed) != 3:
                raise ValueError("%s must be a three-element array" % label)
            if not all(math.isfinite(item) for item in parsed):
                raise ValueError("%s must contain finite coordinates" % label)
            return parsed  # type: ignore[return-value]

        evidence = payload.get("evidence")
        if not isinstance(evidence, Mapping):
            raise ValueError("review task evidence must be an object")
        task = cls(
            kind=str(payload.get("kind", "")),
            title=str(payload.get("title", "")),
            question=str(payload.get("question", "")),
            map_center=point3(payload.get("mapCenter"), "mapCenter"),
            bounds_min=point3(bounds.get("min"), "bounds.min"),
            bounds_max=point3(bounds.get("max"), "bounds.max"),
            evidence=dict(evidence),
            priority=int(payload.get("priority", 50)),
            task_id=str(payload.get("taskId", "")),
        )
        if not task.task_id:
            raise ValueError("review task id is required")
        state = str(payload.get("state", "pending"))
        if state not in {"pending", "decided", "deferred"}:
            raise ValueError("unsupported review task state")
        decision = payload.get("decision")
        if state == "pending" and decision is not None:
            raise ValueError("pending review task cannot have a decision")
        if state != "pending" and decision not in task.options:
            raise ValueError("decided review task has an unsupported decision")
        if state != "pending" and not task.decision_readiness()["ready"]:
            raise ValueError("decided review task lacks operator decision evidence")
        history = payload.get("decisionHistory", [])
        if not isinstance(history, list) or any(not isinstance(item, Mapping) for item in history):
            raise ValueError("review decision history must be an array of objects")
        task.state = state
        task.decision = decision
        task.note = str(payload.get("note", ""))
        task.decided_by = payload.get("decidedBy")
        task.decided_at = payload.get("decidedAt")
        task.decision_history = [dict(item) for item in history]
        return task


class ReviewQueue:
    def __init__(self):
        self.tasks: Dict[str, ReviewTask] = {}

    def add(self, task: ReviewTask) -> None:
        if task.task_id in self.tasks:
            raise ValueError("duplicate review task id")
        self.tasks[task.task_id] = task

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "ReviewQueue":
        if payload.get("schema") != "go2.review_queue.v1":
            raise ValueError("unsupported review queue schema")
        records = payload.get("tasks")
        if not isinstance(records, list):
            raise ValueError("review queue tasks must be an array")
        queue = cls()
        for record in records:
            if not isinstance(record, Mapping):
                raise ValueError("review task record must be an object")
            queue.add(ReviewTask.from_dict(record))
        return queue

    def next_task(self) -> Optional[ReviewTask]:
        pending = [task for task in self.tasks.values() if task.state == "pending"]
        if not pending:
            return None
        return min(
            pending,
            key=lambda task: (
                REVIEW_PHASE_ORDER[task.kind],
                -task.priority,
                task.task_id,
            ),
        )

    def decide(self, task_id: str, decision: str, operator: str, note: str = "") -> None:
        if task_id not in self.tasks:
            raise KeyError(task_id)
        self.tasks[task_id].decide(decision, operator, note)

    def reopen(self, task_id: str, operator: str, reason: str = "") -> None:
        if task_id not in self.tasks:
            raise KeyError(task_id)
        self.tasks[task_id].reopen(operator, reason)

    def last_decided_task(self) -> Optional[ReviewTask]:
        decided = [
            task
            for task in self.tasks.values()
            if task.state in {"decided", "deferred"} and task.decided_at is not None
        ]
        if not decided:
            return None
        return max(decided, key=lambda task: (task.decided_at or 0.0, task.task_id))

    def summary(self) -> Dict[str, Any]:
        counts = {"pending": 0, "decided": 0, "deferred": 0}
        pending_by_kind = {kind: 0 for kind in REVIEW_PHASE_ORDER}
        deferred_by_kind = {kind: 0 for kind in REVIEW_PHASE_ORDER}
        for task in self.tasks.values():
            counts[task.state] += 1
            if task.state == "pending":
                pending_by_kind[task.kind] += 1
            elif task.state == "deferred":
                deferred_by_kind[task.kind] += 1
        current = self.next_task()
        last_decided = self.last_decided_task()
        return {
            "total": len(self.tasks),
            "pending": counts["pending"],
            "decided": counts["decided"],
            "deferred": counts["deferred"],
            "pendingByKind": pending_by_kind,
            "deferredByKind": deferred_by_kind,
            "currentKind": current.kind if current else None,
            "currentTaskId": current.task_id if current else None,
            "lastDecidedTaskId": last_decided.task_id if last_decided else None,
        }

    def to_dict(self) -> Dict[str, Any]:
        current = self.next_task()
        return {
            "schema": "go2.review_queue.v1",
            "summary": self.summary(),
            "current": current.to_dict() if current else None,
            "tasks": [
                task.to_dict()
                for task in sorted(
                    self.tasks.values(),
                    key=lambda item: (-item.priority, item.task_id),
                )
            ],
        }
