"""Machine-checkable target coordinate-frame contract for the Go2 stack."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Set


SCHEMA = "go2.coordinate_contract.v1"
REQUIRED_ROLES = {"map", "odom", "base", "lidar", "imu"}
EXPECTED_EDGES = {
    ("map", "odom"): ("dynamic", "global_localizer"),
    ("odom", "base"): ("dynamic", "local_lio"),
    ("base", "lidar"): ("static", "mount_calibration"),
    ("lidar", "imu"): ("static", "sensor_internal_calibration"),
}


class ContractError(ValueError):
    """Raised when a coordinate contract can permit ambiguous geometry."""


def canonical_json_bytes(value: Mapping[str, Any]) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def content_sha256(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def _require_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ContractError("%s must be a non-empty string" % label)
    return value.strip()


@dataclass(frozen=True)
class CoordinateContract:
    raw: Mapping[str, Any]

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CoordinateContract":
        if not isinstance(data, dict):
            raise ContractError("coordinate contract root must be an object")
        contract = cls(dict(data))
        contract.validate()
        return contract

    @classmethod
    def load(cls, path: Path) -> "CoordinateContract":
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise ContractError("cannot read coordinate contract %s: %s" % (path, exc))
        return cls.from_dict(data)

    @property
    def digest(self) -> str:
        return content_sha256(self.raw)

    @property
    def contract_id(self) -> str:
        return str(self.raw["contract_id"])

    @property
    def role_frames(self) -> Dict[str, str]:
        frames = self.raw["frames"]
        return {role: str(frames[role]["frame_id"]) for role in REQUIRED_ROLES}

    def validate(self) -> None:
        if self.raw.get("schema") != SCHEMA:
            raise ContractError("unsupported coordinate contract schema")
        _require_string(self.raw.get("contract_id"), "contract_id")
        if self.raw.get("deployment_target") != "replacement_navigation_stack":
            raise ContractError("coordinate contract must target the replacement stack")

        frames = self.raw.get("frames")
        if not isinstance(frames, dict) or set(frames) != REQUIRED_ROLES:
            raise ContractError(
                "frames must define exactly these roles: %s"
                % ", ".join(sorted(REQUIRED_ROLES))
            )
        role_frames: Dict[str, str] = {}
        for role, definition in frames.items():
            if not isinstance(definition, dict):
                raise ContractError("frame role %s must be an object" % role)
            role_frames[role] = _require_string(
                definition.get("frame_id"),
                "frames.%s.frame_id" % role,
            )
        if len(set(role_frames.values())) != len(role_frames):
            raise ContractError("frame ids must be unique")

        edges = self.raw.get("tf_edges")
        if not isinstance(edges, list) or len(edges) != len(EXPECTED_EDGES):
            raise ContractError("tf_edges must define the four target-tree edges")
        seen_edges: Set[Any] = set()
        seen_children: Set[str] = set()
        for index, edge in enumerate(edges):
            if not isinstance(edge, dict):
                raise ContractError("tf_edges[%d] must be an object" % index)
            parent_role = _require_string(edge.get("parent_role"), "edge parent_role")
            child_role = _require_string(edge.get("child_role"), "edge child_role")
            pair = (parent_role, child_role)
            if pair in seen_edges:
                raise ContractError("duplicate TF edge %s->%s" % pair)
            if child_role in seen_children:
                raise ContractError("TF child role %s has multiple parents" % child_role)
            seen_edges.add(pair)
            seen_children.add(child_role)
            expected = EXPECTED_EDGES.get(pair)
            actual = (edge.get("kind"), edge.get("authority"))
            if expected is None or actual != expected:
                raise ContractError(
                    "unexpected TF edge %s->%s kind/authority %r"
                    % (parent_role, child_role, actual)
                )
        if seen_edges != set(EXPECTED_EDGES):
            raise ContractError("TF edges do not form the required target tree")

        self._validate_topics(role_frames)
        self._validate_artifacts()
        self._validate_invariants()

    def _validate_topics(self, role_frames: Mapping[str, str]) -> None:
        topics = self.raw.get("topic_frames")
        if not isinstance(topics, list):
            raise ContractError("topic_frames must be a list")
        seen: Set[str] = set()
        required_semantics = {
            "raw_lidar": "lidar",
            "raw_imu": "imu",
            "undistorted_lidar": "lidar",
            "body_obstacle_cloud": "base",
            "global_map_cloud": "map",
        }
        found: Set[str] = set()
        for index, topic in enumerate(topics):
            if not isinstance(topic, dict):
                raise ContractError("topic_frames[%d] must be an object" % index)
            name = _require_string(topic.get("topic"), "topic name")
            semantic = _require_string(topic.get("semantic"), "topic semantic")
            role = _require_string(topic.get("frame_role"), "topic frame_role")
            if name in seen:
                raise ContractError("duplicate topic frame declaration: %s" % name)
            seen.add(name)
            if role not in role_frames:
                raise ContractError("topic %s uses unknown frame role %s" % (name, role))
            expected_role = required_semantics.get(semantic)
            if expected_role is not None:
                found.add(semantic)
                if role != expected_role:
                    raise ContractError(
                        "topic semantic %s must use %s, not %s"
                        % (semantic, expected_role, role)
                    )
        missing = set(required_semantics) - found
        if missing:
            raise ContractError("missing required topic semantics: %s" % sorted(missing))

    def _validate_artifacts(self) -> None:
        artifacts = self.raw.get("artifact_frames")
        expected = {
            "optimized_map": "map",
            "localization_map": "map",
            "route": "map",
            "coverage_grid": "map",
            "local_costmap": "odom",
        }
        if artifacts != expected:
            raise ContractError("artifact_frames must be exactly %r" % expected)

    def _validate_invariants(self) -> None:
        invariants = self.raw.get("invariants")
        if not isinstance(invariants, dict):
            raise ContractError("invariants must be an object")
        required_true = (
            "map_gravity_aligned",
            "raw_sensor_data_immutable",
            "route_fixed_in_map",
            "forbid_posthoc_map_leveling",
            "forbid_runtime_route_anchoring",
            "require_validated_mount_for_motion",
        )
        false_rules = [name for name in required_true if invariants.get(name) is not True]
        if false_rules:
            raise ContractError("required invariants disabled: %s" % false_rules)

    def topic_frame_id(self, semantic: str) -> str:
        role_frames = self.role_frames
        for topic in self.raw["topic_frames"]:
            if topic["semantic"] == semantic:
                return role_frames[topic["frame_role"]]
        raise KeyError(semantic)

    def edge_authority(self, parent_role: str, child_role: str) -> str:
        for edge in self.raw["tf_edges"]:
            if (
                edge["parent_role"] == parent_role
                and edge["child_role"] == child_role
            ):
                return str(edge["authority"])
        raise KeyError((parent_role, child_role))
