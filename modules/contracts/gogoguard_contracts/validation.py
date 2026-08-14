from __future__ import annotations

from datetime import datetime
from typing import Any


def is_safe_external_id(value: Any, *, maximum_length: int = 128) -> bool:
    """Accept protocol IDs, including Chinese site labels, without controls."""

    if not isinstance(value, str) or not 1 <= len(value) <= maximum_length:
        return False
    if not value[0].isalnum():
        return False
    return all(character.isalnum() or character in "._:-" for character in value)


def validate_platform_mission_message(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("platform mission message must be an object")
    schema = payload.get("schema")
    if schema not in {
        "gogoguard.checkpoint_verdict.v1",
        "gogoguard.announcement_completed.v1",
    }:
        raise ValueError("platform mission message schema is invalid")
    for name in ("missionId", "checkpointId"):
        if not is_safe_external_id(payload.get(name)):
            raise ValueError(f"platform mission {name} is invalid")
    attempt = payload.get("attempt")
    if isinstance(attempt, bool) or not isinstance(attempt, int) or not 1 <= attempt <= 6:
        raise ValueError("platform mission attempt is invalid")
    if schema == "gogoguard.checkpoint_verdict.v1":
        for name in ("verdictId", "mapVersion", "routeId"):
            if not is_safe_external_id(payload.get(name)):
                raise ValueError(f"checkpoint verdict {name} is invalid")
        if payload.get("action") not in {"continue", "retake", "skip"}:
            raise ValueError("checkpoint verdict action is invalid")
        expires_at = payload.get("expiresAt")
        if not isinstance(expires_at, str) or not expires_at:
            raise ValueError("checkpoint verdict expiresAt is invalid")
        try:
            parsed = datetime.fromisoformat(expires_at.replace("Z", "+00:00"))
        except ValueError as exc:
            raise ValueError("checkpoint verdict expiresAt is invalid") from exc
        if parsed.tzinfo is None:
            raise ValueError("checkpoint verdict expiresAt must include a timezone")
    else:
        if not is_safe_external_id(payload.get("announcementId")):
            raise ValueError("announcement completion identity is invalid")
        if payload.get("status") not in {"completed", "interrupted", "assumed"}:
            raise ValueError("announcement completion status is invalid")
        duration = payload.get("durationMs")
        if (
            duration is not None
            and (
                isinstance(duration, bool)
                or not isinstance(duration, int)
                or duration < 0
            )
        ):
            raise ValueError("announcement completion duration is invalid")
    return dict(payload)
