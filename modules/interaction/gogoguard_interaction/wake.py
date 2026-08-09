from __future__ import annotations

import re
import threading
import time
from dataclasses import dataclass, replace

from gogoguard_contracts import (
    ConversationWakeState,
    ConversationWakeStatus,
    utc_now,
)


NON_WORD = re.compile(r"[\s，。！？、,.!?;；:：~～]+")


@dataclass(frozen=True)
class WakePolicy:
    canonical_phrase: str = "小玖小玖"
    aliases: tuple[str, ...] = ("小玖小玖", "小九小九")
    explicit_sleep_phrases: tuple[str, ...] = (
        "结束对话",
        "不用了",
        "你休息吧",
        "退出对话",
    )
    idle_timeout_s: float = 30.0
    acknowledgement: str = "我在"

    def __post_init__(self) -> None:
        if not self.canonical_phrase.strip() or not self.aliases:
            raise ValueError("wake phrase must be configured")
        if self.idle_timeout_s < 5.0:
            raise ValueError("wake idle timeout must be at least 5 seconds")


class WakeConversationGate:
    """Conversation gate driven by platform ASR events, independent of media live state."""

    def __init__(self, policy: WakePolicy = WakePolicy()) -> None:
        self.policy = policy
        self._lock = threading.Lock()
        self._status = ConversationWakeStatus(wake_phrase=policy.canonical_phrase)
        self._deadline: float | None = None

    def status(self) -> ConversationWakeStatus:
        with self._lock:
            return replace(self._status)

    def handle_transcript(self, text: str, *, now: float | None = None) -> dict:
        if not isinstance(text, str) or not text.strip():
            raise ValueError("transcript must be non-empty")
        instant = time.monotonic() if now is None else now
        normalized = NON_WORD.sub("", text)
        with self._lock:
            if self._status.state == ConversationWakeState.SLEEPING:
                if not any(NON_WORD.sub("", phrase) in normalized for phrase in self.policy.aliases):
                    return {"accepted": False, "action": "ignore", "state": self._status.state.value}
                self._status.state = ConversationWakeState.AWAKE
                self._status.wake_sequence += 1
                self._status.awakened_at = utc_now()
                self._status.last_activity_at = self._status.awakened_at
                self._status.sleep_reason = None
                self._status.observed_at = utc_now()
                self._deadline = instant + self.policy.idle_timeout_s
                return {
                    "accepted": True,
                    "action": "wake",
                    "state": self._status.state.value,
                    "wakeSequence": self._status.wake_sequence,
                    "acknowledgement": self.policy.acknowledgement,
                }

            if any(NON_WORD.sub("", phrase) in normalized for phrase in self.policy.explicit_sleep_phrases):
                return self._sleep_locked("explicit_end")
            self._status.last_activity_at = utc_now()
            self._status.observed_at = utc_now()
            self._deadline = instant + self.policy.idle_timeout_s
            return {"accepted": True, "action": "continue", "state": self._status.state.value}

    def tick(self, *, now: float | None = None) -> dict | None:
        instant = time.monotonic() if now is None else now
        with self._lock:
            if self._status.state != ConversationWakeState.AWAKE or self._deadline is None:
                return None
            if instant < self._deadline:
                return None
            return self._sleep_locked("idle_timeout")

    def sleep(self, reason: str = "platform_stop") -> dict:
        if not reason.strip():
            raise ValueError("sleep reason must be non-empty")
        with self._lock:
            return self._sleep_locked(reason.strip())

    def can_forward_conversation(self) -> bool:
        return self.status().state == ConversationWakeState.AWAKE

    def _sleep_locked(self, reason: str) -> dict:
        self._status.state = ConversationWakeState.SLEEPING
        self._status.sleep_reason = reason
        self._status.observed_at = utc_now()
        self._deadline = None
        return {"accepted": True, "action": "sleep", "reason": reason, "state": self._status.state.value}
