from __future__ import annotations

import asyncio
import audioop
import json
import queue
import sys
import threading
import time
import types
from fractions import Fraction
from typing import Any, Callable

from gogoguard_contracts import MediaConnectionReceipt, RealtimeMediaSessionRequest

from .interaction_media import (
    Go2VolumeController,
    InteractionHardwareProfile,
    SpeakerJitterBuffer,
    decode_s24_3le_stereo_to_s16_mono,
)


class LiveKitGo2Transport:
    """Thread-owned LiveKit/Go2 bridge. Optional native dependencies load on start."""

    def __init__(
        self,
        *,
        profile: InteractionHardwareProfile,
        unitree_aes_128_key: str,
        volume_controller: Go2VolumeController | None = None,
    ) -> None:
        if not unitree_aes_128_key:
            raise ValueError("per-device Unitree AES key is required")
        self.profile = profile
        self._unitree_key = unitree_aes_128_key
        self._volume_controller = volume_controller
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._session_stop: asyncio.Event | None = None
        self._shutdown = threading.Event()
        self._request_lock = threading.Lock()
        self._request: RealtimeMediaSessionRequest | None = None
        self._microphone_track = None
        self._microphone_muted = threading.Event()
        self._ready: queue.Queue[MediaConnectionReceipt | BaseException] = queue.Queue(maxsize=1)
        self._playback_control_handler: Callable[[dict[str, Any]], Any] | None = None
        self._wake_transcript_handler: Callable[[dict[str, Any]], Any] | None = None
        self._playback_state_handler: Callable[[bool], Any] | None = None
        self._health_handler: Callable[[bool, str | None], Any] | None = None
        self._ever_connected = False
        self._reconnect_count = 0
        self.speaker = SpeakerJitterBuffer(
            sample_rate_hz=profile.speaker_rate_hz,
            frame_ms=profile.speaker_frame_ms,
            prebuffer_ms=profile.speaker_prebuffer_ms,
            max_buffer_ms=profile.speaker_max_buffer_ms,
        )

    def set_playback_control_handler(self, handler: Callable[[dict[str, Any]], Any]) -> None:
        self._playback_control_handler = handler

    def set_wake_transcript_handler(self, handler: Callable[[dict[str, Any]], Any]) -> None:
        self._wake_transcript_handler = handler

    def set_playback_state_handler(self, handler: Callable[[bool], Any]) -> None:
        self._playback_state_handler = handler

    def set_health_handler(self, handler: Callable[[bool, str | None], Any]) -> None:
        self._health_handler = handler

    def connect(self, request: RealtimeMediaSessionRequest) -> MediaConnectionReceipt:
        if self._thread and self._thread.is_alive():
            raise RuntimeError("media transport is already running")
        if self._volume_controller is not None:
            self._volume_controller.set_and_verify(self.profile.speaker_default_volume)
        self._ready = queue.Queue(maxsize=1)
        self._shutdown.clear()
        self._ever_connected = False
        self._reconnect_count = 0
        with self._request_lock:
            self._request = request
        self._thread = threading.Thread(
            target=self._thread_main,
            args=(request,),
            name="gogoguard-livekit-go2",
            daemon=True,
        )
        self._thread.start()
        try:
            result = self._ready.get(timeout=15)
        except queue.Empty as exc:
            self.disconnect()
            raise TimeoutError("real media did not become ready within 15 seconds") from exc
        if isinstance(result, BaseException):
            self.disconnect()
            raise result
        return result

    def disconnect(self) -> None:
        self._shutdown.set()
        if self._loop is not None and self._session_stop is not None:
            self._loop.call_soon_threadsafe(self._session_stop.set)
        if self._thread is not None:
            self._thread.join(timeout=8)
            if self._thread.is_alive():
                raise TimeoutError("media transport did not stop within 8 seconds")
        self._thread = None
        self._loop = None
        self._session_stop = None
        with self._request_lock:
            self._request = None
        self._microphone_track = None
        self._microphone_muted.clear()
        self.speaker.flush()

    def refresh(self, request: RealtimeMediaSessionRequest) -> None:
        """Keep the newest ephemeral token in memory for a future reconnect."""

        with self._request_lock:
            current = self._request
            if current is None:
                raise RuntimeError("media transport is not running")
            if current.room != request.room or current.robot_id != request.robot_id:
                raise ValueError("refreshed media request changes session identity")
            self._request = request

    def set_microphone_muted(self, muted: bool) -> None:
        if muted:
            self._microphone_muted.set()
        else:
            self._microphone_muted.clear()
        track = self._microphone_track
        loop = self._loop
        if track is not None and loop is not None:
            loop.call_soon_threadsafe(track.mute if muted else track.unmute)

    def stop_playback(self) -> None:
        self.speaker.flush()

    def resume_playback(self) -> None:
        self.speaker.set_enabled(True)

    def status(self) -> dict[str, Any]:
        return {
            "running": bool(self._thread and self._thread.is_alive()),
            "microphoneMuted": self._microphone_muted.is_set(),
            "reconnectCount": self._reconnect_count,
            "speaker": self.speaker.status(),
        }

    def handle_data_message(
        self, payload: Any, *, participant_identity: str
    ) -> bool:
        """Dispatch the two frozen agent-to-robot DataChannel schemas only."""

        if not participant_identity.startswith("agent:") or not isinstance(payload, dict):
            return False
        schema = payload.get("schema")
        if schema == "gogoguard.playback_control.v1":
            handler = self._playback_control_handler
            if handler is not None:
                handler(payload)
            elif payload.get("action") == "stop":
                self.speaker.flush()
            return True
        if schema == "gogoguard.wake_transcript.v1":
            text = payload.get("text")
            if payload.get("action") != "wake_transcript" or not isinstance(text, str):
                return False
            if not text or len(text) > 4096:
                return False
            handler = self._wake_transcript_handler
            if handler is None:
                return False
            handler({"action": "wake_transcript", "text": text})
            return True
        return False

    def _thread_main(self, request: RealtimeMediaSessionRequest) -> None:
        backoff_s = 1.0
        while not self._shutdown.is_set():
            with self._request_lock:
                current = self._request or request
            try:
                asyncio.run(self._run(current))
                if self._shutdown.is_set():
                    return
                raise ConnectionError("media session ended unexpectedly")
            except BaseException as exc:
                if not self._ever_connected:
                    if self._ready.empty():
                        self._ready.put(exc)
                    return
                self._reconnect_count += 1
                self._notify_health(False, f"{type(exc).__name__}: media reconnect pending")
                if self._shutdown.wait(backoff_s):
                    return
                backoff_s = min(backoff_s * 2.0, 10.0)

    def _notify_health(self, recovered: bool, reason: str | None) -> None:
        handler = self._health_handler
        if handler is None:
            return
        try:
            handler(recovered, reason)
        except Exception:
            pass

    async def _run(self, request: RealtimeMediaSessionRequest) -> None:
        sys.modules.setdefault("sounddevice", types.ModuleType("sounddevice"))
        try:
            import av
            from aiortc import MediaStreamTrack
            from livekit import rtc
            from unitree_webrtc_connect import (
                UnitreeWebRTCConnection,
                WebRTCConnectionMethod,
            )
        except ImportError as exc:
            raise RuntimeError("realtime media dependencies are unavailable") from exc

        profile = self.profile
        speaker_buffer = self.speaker

        class SpeakerTrack(MediaStreamTrack):
            kind = "audio"

            def __init__(self) -> None:
                super().__init__()
                self.started: float | None = None
                self.samples_sent = 0

            async def recv(self):
                if self.started is None:
                    self.started = time.monotonic()
                target = self.started + self.samples_sent / profile.speaker_rate_hz
                delay = target - time.monotonic()
                if delay > 0:
                    await asyncio.sleep(delay)
                pcm = speaker_buffer.read_frame()
                frame = av.AudioFrame(
                    format="s16", layout="mono",
                    samples=profile.speaker_rate_hz * profile.speaker_frame_ms // 1000,
                )
                frame.planes[0].update(pcm)
                frame.sample_rate = profile.speaker_rate_hz
                frame.pts = self.samples_sent
                frame.time_base = Fraction(1, profile.speaker_rate_hz)
                self.samples_sent += frame.samples
                return frame

        room = rtc.Room()
        go2 = None
        audio_source = None
        video_source = None
        tasks: list[asyncio.Task] = []
        mic_ready = asyncio.Event()
        video_ready = asyncio.Event()
        agent_audio_ready = asyncio.Event()
        self._loop = asyncio.get_running_loop()
        self._session_stop = asyncio.Event()
        playback_active = False
        playback_active_until = 0.0

        def set_playback_active(active: bool) -> None:
            nonlocal playback_active
            if playback_active == active:
                return
            playback_active = active
            handler = self._playback_state_handler
            if handler is not None:
                handler(active)

        async def playback_monitor() -> None:
            nonlocal playback_active_until
            while not self._session_stop.is_set():
                if playback_active and time.monotonic() >= playback_active_until:
                    set_playback_active(False)
                await asyncio.sleep(0.02)

        async def consume_agent_audio(track) -> None:
            nonlocal playback_active_until
            stream = rtc.AudioStream(
                track, sample_rate=profile.speaker_rate_hz,
                num_channels=1, frame_size_ms=profile.speaker_frame_ms, capacity=100,
            )
            try:
                async for event in stream:
                    pcm = bytes(event.frame.data)
                    speaker_buffer.append(pcm)
                    if audioop.rms(pcm, 2) >= 90:
                        playback_active_until = time.monotonic() + 0.35
                        set_playback_active(True)
                    if self._session_stop.is_set():
                        break
            finally:
                await stream.aclose()

        @room.on("track_subscribed")
        def on_track_subscribed(track, publication, participant):
            if (
                participant.identity.startswith("agent:")
                and publication.name == "agent-voice"
                and track.kind == rtc.TrackKind.KIND_AUDIO
            ):
                agent_audio_ready.set()
                tasks.append(asyncio.create_task(consume_agent_audio(track)))

        @room.on("data_received")
        def on_data_received(packet):
            try:
                raw = bytes(packet.data)
                if len(raw) > 65536:
                    return
                payload = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                return
            participant = getattr(packet, "participant", None)
            identity = str(getattr(participant, "identity", ""))
            try:
                self.handle_data_message(payload, participant_identity=identity)
            except Exception:
                # A malformed or stale control message must not terminate media.
                return

        async def publish_microphone() -> None:
            process = await asyncio.create_subprocess_exec(
                "arecord", "-q", "-D", profile.microphone_device,
                "-t", "raw", "-f", profile.microphone_format,
                "-r", str(profile.microphone_rate_hz), "-c", str(profile.microphone_channels),
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
            )
            input_bytes = profile.microphone_rate_hz * profile.microphone_channels * 3 * profile.speaker_frame_ms // 1000
            try:
                while not self._session_stop.is_set():
                    data = await process.stdout.readexactly(input_bytes)
                    pcm = decode_s24_3le_stereo_to_s16_mono(data, gain=profile.microphone_gain)
                    if self._microphone_muted.is_set():
                        pcm = bytes(len(pcm))
                    await audio_source.capture_frame(rtc.AudioFrame(
                        data=pcm, sample_rate=profile.microphone_rate_hz,
                        num_channels=1, samples_per_channel=len(pcm) // 2,
                    ))
                    mic_ready.set()
            except asyncio.IncompleteReadError as exc:
                if not self._session_stop.is_set():
                    error = (await process.stderr.read()).decode("utf-8", errors="replace")[-300:]
                    raise RuntimeError("BOYA capture ended: " + error) from exc
            finally:
                if process.returncode is None:
                    process.terminate()
                await process.wait()

        async def publish_camera() -> None:
            container = await asyncio.to_thread(
                av.open, profile.camera_rtsp,
                options={"rtsp_transport": "tcp"}, timeout=(4.0, 4.0),
            )
            decoder = container.decode(video=0)

            def next_i420() -> bytearray:
                frame = next(decoder)
                converted = frame.reformat(
                    width=profile.video_width, height=profile.video_height,
                    format="yuv420p",
                )
                return bytearray(converted.to_ndarray().tobytes())

            try:
                while not self._session_stop.is_set():
                    try:
                        buffer = await asyncio.to_thread(next_i420)
                    except StopIteration:
                        raise RuntimeError("Z1Pro stream ended")
                    video_source.capture_frame(rtc.VideoFrame(
                        profile.video_width, profile.video_height,
                        rtc.VideoBufferType.I420, buffer,
                    ))
                    video_ready.set()
            finally:
                container.close()

        try:
            go2 = UnitreeWebRTCConnection(
                WebRTCConnectionMethod.LocalSTA,
                ip=profile.go2_ip,
                aes_128_key=self._unitree_key,
            )
            await go2.connect()
            go2.pc.addTrack(SpeakerTrack())
            await room.connect(request.url, request.token, rtc.RoomOptions(auto_subscribe=True))

            audio_publication = None
            if request.publish_audio:
                audio_source = rtc.AudioSource(profile.microphone_rate_hz, 1, queue_size_ms=120)
                self._microphone_track = rtc.LocalAudioTrack.create_audio_track("robot-microphone", audio_source)
                audio_publication = await room.local_participant.publish_track(
                    self._microphone_track,
                    rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_MICROPHONE),
                )
                tasks.append(asyncio.create_task(publish_microphone()))
                go2.audio.switchAudioChannel(True)

            video_publication = None
            if request.publish_video:
                video_source = rtc.VideoSource(profile.video_width, profile.video_height)
                camera = rtc.LocalVideoTrack.create_video_track("z1pro-camera", video_source)
                video_publication = await room.local_participant.publish_track(
                    camera,
                    rtc.TrackPublishOptions(
                        source=rtc.TrackSource.SOURCE_CAMERA,
                        video_codec=rtc.VideoCodec.H264,
                        video_encoding=rtc.VideoEncoding(
                            max_framerate=profile.video_fps,
                            max_bitrate=profile.video_bitrate,
                        ),
                    ),
                )
                tasks.append(asyncio.create_task(publish_camera()))

            tasks.append(asyncio.create_task(playback_monitor()))
            if request.publish_audio:
                await asyncio.wait_for(mic_ready.wait(), timeout=5)
            if request.publish_video:
                await asyncio.wait_for(video_ready.wait(), timeout=8)
            if request.subscribe_audio:
                await asyncio.wait_for(agent_audio_ready.wait(), timeout=5)
            receipt = MediaConnectionReceipt(
                participant_id=request.robot_id if request.robot_id.startswith("robot:") else f"robot:{request.robot_id}",
                video_published=video_publication is not None,
                audio_published=audio_publication is not None,
                audio_subscribed=request.subscribe_audio and agent_audio_ready.is_set(),
                data_connected=request.publish_data,
            )
            if not self._ever_connected:
                self._ever_connected = True
                self._ready.put(receipt)
            else:
                self._notify_health(True, None)
            stop_wait = asyncio.create_task(self._session_stop.wait())
            tasks.append(stop_wait)
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            if stop_wait not in done:
                for task in done:
                    exception = task.exception()
                    if exception is not None:
                        raise exception
                raise ConnectionError("a required realtime media task ended")
        finally:
            self._session_stop.set()
            set_playback_active(False)
            for task in tasks:
                if not task.done():
                    task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            if go2 is not None:
                try:
                    go2.audio.switchAudioChannel(False)
                except Exception:
                    pass
            self._microphone_track = None
            self._loop = None
            try:
                await room.disconnect()
            except Exception:
                pass
            if audio_source is not None:
                await audio_source.aclose()
            if video_source is not None:
                await video_source.aclose()
            if go2 is not None:
                try:
                    await go2.disconnect()
                except Exception:
                    pass
