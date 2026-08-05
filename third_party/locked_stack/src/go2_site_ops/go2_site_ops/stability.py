"""Dynamic-map evidence and operator review task generation.

GLIM supplies registration and DUFOMap supplies the static/dynamic geometric
classification.  Native ray traversal is retained only to explain whether a
candidate was present, confirmed clear, or occluded in each acquisition.  It
does not own the production classification.
"""

from __future__ import annotations

from collections import deque
import hashlib
import json
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

from .review import ReviewQueue, ReviewTask


VoxelKey = Tuple[int, int, int]


@dataclass(frozen=True)
class StabilityPolicy:
    resolution_m: float = 0.25
    minimum_stable_hits: int = 2
    minimum_stable_persistence: float = 0.70
    minimum_transient_observations: int = 2
    maximum_transient_persistence: float = 0.40
    minimum_review_component_voxels: int = 2

    def __post_init__(self) -> None:
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if self.minimum_stable_hits < 2:
            raise ValueError("minimum_stable_hits must be at least two")
        if self.minimum_transient_observations < 2:
            raise ValueError("minimum_transient_observations must be at least two")
        for name in (
            "minimum_stable_persistence",
            "maximum_transient_persistence",
        ):
            value = float(getattr(self, name))
            if value < 0.0 or value > 1.0:
                raise ValueError("%s must be within [0, 1]" % name)
        if self.maximum_transient_persistence >= self.minimum_stable_persistence:
            raise ValueError("transient/stable persistence bands must not overlap")
        if self.minimum_review_component_voxels < 1:
            raise ValueError("minimum_review_component_voxels must be positive")


@dataclass
class VoxelEvidence:
    key: VoxelKey
    hit_sessions: Set[str] = field(default_factory=set)
    observable_sessions: Set[str] = field(default_factory=set)
    engine_classification: Optional[str] = None

    @property
    def persistence(self) -> Optional[float]:
        if not self.observable_sessions:
            return None
        return len(self.hit_sessions) / float(len(self.observable_sessions))

    def classification(self, policy: StabilityPolicy) -> str:
        if self.engine_classification is not None:
            return self.engine_classification
        persistence = self.persistence
        if persistence is None:
            return "unobserved"
        if (
            len(self.hit_sessions) >= policy.minimum_stable_hits
            and persistence >= policy.minimum_stable_persistence
        ):
            return "stable"
        if (
            self.hit_sessions
            and len(self.observable_sessions) >= policy.minimum_transient_observations
            and persistence <= policy.maximum_transient_persistence
        ):
            return "transient_candidate"
        return "uncertain"

    def to_dict(self, policy: StabilityPolicy) -> Dict[str, Any]:
        return {
            "key": list(self.key),
            "classification": self.classification(policy),
            "hitSessions": sorted(self.hit_sessions),
            "observableSessions": sorted(self.observable_sessions),
            "persistence": self.persistence,
        }


class StabilityEvidenceMap:
    def __init__(self, policy: Optional[StabilityPolicy] = None):
        self.policy = policy or StabilityPolicy()
        self.voxels: Dict[VoxelKey, VoxelEvidence] = {}
        self.session_ids: Set[str] = set()
        self.algorithm_contract: Optional[Dict[str, Any]] = None
        self.observation_schema = "go2.stability_observations.v1"

    def add_session(
        self,
        session_id: str,
        *,
        hit_voxels: Iterable[VoxelKey],
        observable_voxels: Iterable[VoxelKey],
    ) -> None:
        normalized_session = str(session_id).strip()
        if not normalized_session:
            raise ValueError("session_id is required")
        if normalized_session in self.session_ids:
            raise ValueError("session evidence already exists: %s" % normalized_session)
        hits = {self._normalize_key(key) for key in hit_voxels}
        observable = {self._normalize_key(key) for key in observable_voxels}
        if not hits <= observable:
            raise ValueError("every hit voxel must also be observable")
        self.session_ids.add(normalized_session)
        for key in observable:
            evidence = self.voxels.setdefault(key, VoxelEvidence(key=key))
            evidence.observable_sessions.add(normalized_session)
        for key in hits:
            self.voxels[key].hit_sessions.add(normalized_session)

    @classmethod
    def from_observation_payload(
        cls,
        payload: Mapping[str, Any],
        *,
        allow_legacy: bool = True,
    ) -> "StabilityEvidenceMap":
        schema = payload.get("schema")
        if schema == "go2.dynamic_map_observations.v2":
            return cls._from_dynamic_map_payload(payload)
        if schema != "go2.stability_observations.v1":
            raise ValueError("unsupported stability observation schema")
        if not allow_legacy:
            raise ValueError(
                "legacy OctoMap persistence evidence is diagnostic-only and cannot drive review"
            )
        policy_payload = payload.get("policy", {})
        if not isinstance(policy_payload, dict):
            raise ValueError("stability policy must be an object")
        policy = StabilityPolicy(
            resolution_m=float(payload.get("resolutionM", 0.25)),
            minimum_stable_hits=int(policy_payload.get("minimumStableHits", 2)),
            minimum_stable_persistence=float(
                policy_payload.get("minimumStablePersistence", 0.70)
            ),
            minimum_transient_observations=int(
                policy_payload.get("minimumTransientObservations", 2)
            ),
            maximum_transient_persistence=float(
                policy_payload.get("maximumTransientPersistence", 0.40)
            ),
            minimum_review_component_voxels=int(
                policy_payload.get("minimumReviewComponentVoxels", 2)
            ),
        )
        sessions = payload.get("sessions")
        if not isinstance(sessions, list) or len(sessions) < 2:
            raise ValueError("stability evidence needs at least two acquisitions")
        evidence = cls(policy)
        for record in sessions:
            if not isinstance(record, dict):
                raise ValueError("stability session record must be an object")
            hit_voxels = record.get("hitVoxels")
            observable_voxels = record.get("observableVoxels")
            if not isinstance(hit_voxels, list) or not isinstance(observable_voxels, list):
                raise ValueError("stability session needs hitVoxels and observableVoxels")
            evidence.add_session(
                str(record.get("acquisitionId", "")),
                hit_voxels=hit_voxels,
                observable_voxels=observable_voxels,
            )
        return evidence

    @classmethod
    def _from_dynamic_map_payload(
        cls,
        payload: Mapping[str, Any],
    ) -> "StabilityEvidenceMap":
        engine = payload.get("engine")
        if not isinstance(engine, dict):
            raise ValueError("dynamic map evidence has no algorithm contract")
        required_engine = {
            "schema": "go2.dynamic_map_engine_contract.v1",
            "id": "dufomap",
            "name": "DUFOMap",
            "version": "1.1.1",
            "sourceUrl": "https://github.com/KTH-RPL/dufomap",
            "publicationDoi": "10.1109/LRA.2024.3387658",
            "license": "BSD-3-Clause",
            "registrationProvider": "GLIM",
            "inputPointFrame": "map",
            "inputPoseFrame": "map_to_lidar_link",
        }
        for key, expected in required_engine.items():
            if engine.get(key) != expected:
                raise ValueError("unapproved dynamic map engine contract field: %s" % key)
        parameters = engine.get("parameters")
        runtime = engine.get("runtime")
        meanings = engine.get("classificationMeaning")
        if not isinstance(parameters, dict) or not isinstance(runtime, dict) or not isinstance(meanings, dict):
            raise ValueError("dynamic map engine contract is incomplete")
        if runtime.get("pythonPackage") != "dufomap==1.1.1":
            raise ValueError("dynamic map runtime is not pinned")
        contract_hash = engine.get("contractSha256")
        if not isinstance(contract_hash, str) or len(contract_hash) != 64:
            raise ValueError("dynamic map engine contract has no SHA-256 identity")
        unhashed = dict(engine)
        unhashed.pop("contractSha256", None)
        computed_hash = hashlib.sha256(
            json.dumps(
                unhashed,
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=False,
                allow_nan=False,
            ).encode("utf-8")
        ).hexdigest()
        if computed_hash != contract_hash:
            raise ValueError("dynamic map engine contract hash mismatch")

        try:
            resolution = float(payload["resolutionM"])
            minimum_component = int(parameters.get("minimumReviewComponentVoxels", 2))
        except (KeyError, TypeError, ValueError):
            raise ValueError("dynamic map review resolution is invalid")
        policy = StabilityPolicy(
            resolution_m=resolution,
            minimum_review_component_voxels=minimum_component,
        )
        sessions = payload.get("sessions")
        if not isinstance(sessions, list) or len(sessions) < 2:
            raise ValueError("dynamic map evidence needs at least two acquisitions")
        evidence = cls(policy)
        evidence.algorithm_contract = dict(engine)
        evidence.observation_schema = "go2.dynamic_map_observations.v2"
        for record in sessions:
            if not isinstance(record, dict):
                raise ValueError("dynamic map session record must be an object")
            hit_voxels = record.get("hitVoxels")
            observable_voxels = record.get("observableVoxels")
            if not isinstance(hit_voxels, list) or not isinstance(observable_voxels, list):
                raise ValueError("dynamic map session needs hitVoxels and observableVoxels")
            evidence.add_session(
                str(record.get("acquisitionId", "")),
                hit_voxels=hit_voxels,
                observable_voxels=observable_voxels,
            )

        static_payload = payload.get("staticVoxels")
        dynamic_payload = payload.get("dynamicVoxels")
        unknown_payload = payload.get("unknownVoxels", [])
        if not isinstance(static_payload, list) or not isinstance(dynamic_payload, list) or not isinstance(unknown_payload, list):
            raise ValueError("dynamic map classifications must be voxel arrays")
        static_voxels = {evidence._normalize_key(key) for key in static_payload}
        dynamic_voxels = {evidence._normalize_key(key) for key in dynamic_payload}
        unknown_voxels = {evidence._normalize_key(key) for key in unknown_payload}
        if static_voxels & dynamic_voxels or static_voxels & unknown_voxels or dynamic_voxels & unknown_voxels:
            raise ValueError("dynamic map voxel classifications overlap")
        observed_hits = {
            key for key, record in evidence.voxels.items() if record.hit_sessions
        }
        if not (static_voxels | dynamic_voxels | unknown_voxels) <= observed_hits:
            raise ValueError("dynamic map classifies geometry absent from sealed acquisitions")
        for key in static_voxels:
            evidence.voxels[key].engine_classification = "stable"
        for key in dynamic_voxels:
            evidence.voxels[key].engine_classification = "transient_candidate"
        for key in unknown_voxels:
            evidence.voxels[key].engine_classification = "uncertain"
        for record in evidence.voxels.values():
            if record.engine_classification is None:
                record.engine_classification = "uncertain"
        return evidence

    def summary(self) -> Dict[str, int]:
        counts = {
            "stable": 0,
            "transient_candidate": 0,
            "uncertain": 0,
            "unobserved": 0,
        }
        for evidence in self.voxels.values():
            counts[evidence.classification(self.policy)] += 1
        return counts

    def review_queue(self) -> ReviewQueue:
        queue = ReviewQueue()
        for index, component in enumerate(self.transient_components(), start=1):
            bounds_min, bounds_max = self._component_bounds(component)
            center = tuple(
                (bounds_min[axis] + bounds_max[axis]) * 0.5 for axis in range(3)
            )
            comparison = self._component_session_comparison(component)
            hit_sessions = {
                row["acquisitionId"]
                for row in comparison
                if row["state"] == "present"
            }
            observable_sessions = {
                row["acquisitionId"]
                for row in comparison
                if row["state"] in {"present", "clear"}
            }
            clear_sessions = {
                row["acquisitionId"]
                for row in comparison
                if row["state"] == "clear"
            }
            if not hit_sessions or not clear_sessions:
                # DUFOMap can flag geometry without another acquisition having
                # a fully visible absence.  That remains excluded/unknown and
                # never becomes a question the field operator has to guess.
                continue
            persistence = len(hit_sessions) / float(max(1, len(observable_sessions)))
            task_identity = hashlib.sha256(
                json.dumps(
                    {
                        "component": sorted(component),
                        "hitSessions": sorted(hit_sessions),
                        "observableSessions": sorted(observable_sessions),
                        "resolutionM": self.policy.resolution_m,
                    },
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
            ).hexdigest()[:20]
            priority = min(
                95,
                55 + len(component) + 5 * max(0, len(observable_sessions) - len(hit_sessions)),
            )
            queue.add(
                ReviewTask(
                    kind="object_persistence",
                    title="多次扫描不一致的区域 %d" % index,
                    question="请核对现场：这块几何是场地的长期结构，还是本次采集才出现？",
                    map_center=center,
                    bounds_min=bounds_min,
                    bounds_max=bounds_max,
                    evidence={
                        "voxelCount": len(component),
                        "hitSessionCount": len(hit_sessions),
                        "observableSessionCount": len(observable_sessions),
                        "hitSessions": sorted(hit_sessions),
                        "observableSessions": sorted(observable_sessions),
                        "persistence": round(persistence, 4),
                        "rule": "absence_only_counts_when_observable",
                        "decisionBasis": (
                            "dufomap_static_dynamic_plus_observable_absence"
                            if self.algorithm_contract is not None
                            else "legacy_octomap_persistence_diagnostic_only"
                        ),
                        "algorithmContract": self.algorithm_contract,
                        "acquisitionComparison": comparison,
                    },
                    priority=priority,
                    task_id="stability-%s" % task_identity,
                )
            )
        return queue

    def _component_session_comparison(
        self,
        component: Set[VoxelKey],
    ) -> List[Dict[str, Any]]:
        """Explain a candidate using conservative, component-level evidence.

        A session is labelled ``clear`` only when every voxel in the proposed
        object component was observable and none was hit.  Partially visible
        sessions remain in the audit record but never become absence evidence
        in the operator UI.
        """
        total = len(component)
        rows: List[Dict[str, Any]] = []
        for session_id in sorted(self.session_ids):
            observable_count = sum(
                session_id in self.voxels[key].observable_sessions
                for key in component
            )
            hit_count = sum(
                session_id in self.voxels[key].hit_sessions
                for key in component
            )
            if hit_count:
                state = "present"
            elif observable_count == total:
                state = "clear"
            else:
                state = "partial"
            rows.append(
                {
                    "acquisitionId": session_id,
                    "state": state,
                    "observableVoxelCount": observable_count,
                    "hitVoxelCount": hit_count,
                    "componentVoxelCount": total,
                }
            )
        return rows

    def transient_components(self) -> List[Set[VoxelKey]]:
        pending = {
            key
            for key, evidence in self.voxels.items()
            if evidence.classification(self.policy) == "transient_candidate"
        }
        components: List[Set[VoxelKey]] = []
        while pending:
            start = pending.pop()
            component = {start}
            frontier = deque([start])
            while frontier:
                current = frontier.popleft()
                for neighbor in self._neighbors(current):
                    if neighbor not in pending:
                        continue
                    pending.remove(neighbor)
                    component.add(neighbor)
                    frontier.append(neighbor)
            if len(component) >= self.policy.minimum_review_component_voxels:
                components.append(component)
        return sorted(
            components,
            key=lambda component: (-len(component), min(component)),
        )

    def to_dict(self) -> Dict[str, Any]:
        payload = {
            "schema": (
                "go2.dynamic_map_evidence.v2"
                if self.algorithm_contract is not None
                else "go2.stability_evidence.v1"
            ),
            "resolutionM": self.policy.resolution_m,
            "sessionIds": sorted(self.session_ids),
            "summary": self.summary(),
            "voxels": [
                self.voxels[key].to_dict(self.policy)
                for key in sorted(self.voxels)
            ],
        }
        if self.algorithm_contract is not None:
            payload["algorithmContract"] = self.algorithm_contract
            payload["productionDecisionBasis"] = "approved_dynamic_map_engine"
        else:
            payload["productionDecisionBasis"] = "diagnostic_only"
        return payload

    @staticmethod
    def _normalize_key(key: Sequence[int]) -> VoxelKey:
        if len(key) != 3:
            raise ValueError("voxel key must have three coordinates")
        values = tuple(int(value) for value in key)
        if any(float(value) != float(original) for value, original in zip(values, key)):
            raise ValueError("voxel coordinates must be integers")
        return values  # type: ignore[return-value]

    @staticmethod
    def _neighbors(key: VoxelKey) -> Iterable[VoxelKey]:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    yield (key[0] + dx, key[1] + dy, key[2] + dz)

    def _component_bounds(
        self,
        component: Set[VoxelKey],
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        resolution = self.policy.resolution_m
        minimum = tuple(min(key[axis] for key in component) * resolution for axis in range(3))
        maximum = tuple(
            (max(key[axis] for key in component) + 1) * resolution
            for axis in range(3)
        )
        return minimum, maximum  # type: ignore[return-value]
