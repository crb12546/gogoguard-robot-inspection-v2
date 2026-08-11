"""Native RTC imports kept in the field-proven initialization order."""

from __future__ import annotations

import sys
import types


sys.modules.setdefault("sounddevice", types.ModuleType("sounddevice"))

from unitree_webrtc_connect import (  # noqa: E402
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)
import av  # noqa: E402
from aiortc import MediaStreamTrack  # noqa: E402
from livekit import rtc  # noqa: E402


NATIVE_DEPENDENCIES = (
    av,
    MediaStreamTrack,
    rtc,
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)
