from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RobotPersona:
    schema: str
    name: str
    developer: str
    role: str
    introduction: str
    visual_grounding_rule: str
    motion_policy: str

    def platform_prompt_fragment(self) -> str:
        return (
            f"你的名字是{self.name}，由{self.developer}开发，是{self.role}。"
            f"自我介绍时自然表达：{self.introduction}。"
            f"视觉规则：{self.visual_grounding_rule}。"
            f"运动规则：{self.motion_policy}。"
        )


def load_persona(path: Path) -> RobotPersona:
    value = json.loads(path.read_text(encoding="utf-8"))
    required = {
        "schema",
        "name",
        "developer",
        "role",
        "introduction",
        "visual_grounding_rule",
        "motion_policy",
    }
    missing = sorted(required - value.keys())
    if missing:
        raise ValueError("persona fields missing: " + ", ".join(missing))
    if value["schema"] != "gogoguard.robot_persona.v1":
        raise ValueError("unsupported persona schema")
    fields = {name: value[name] for name in required}
    if any(not isinstance(item, str) or not item.strip() for item in fields.values()):
        raise ValueError("persona fields must be non-empty strings")
    return RobotPersona(**fields)
