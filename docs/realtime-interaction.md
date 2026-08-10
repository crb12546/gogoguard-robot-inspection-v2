# Realtime interaction runtime

## Ownership

`interaction` owns the desired/actual LiveKit session, half-duplex state,
ordered playback interruption, the 小玖 persona contract and the wake/sleep
conversation gate. `device_io` owns BOYA capture, Z1Pro decode, Go2 playback,
VUI volume and bounded speaker buffering. The composition service is
`gogoguard-interaction-edge`.

The runtime imports no navigation implementation and has no motion command
API. `/api/v1/interaction` is read-only observability served from the status
file. Platform commands enter only through the local Unix socket.

## Commissioned device profile

- BOYA mini 2: `hw:CARD=B2,DEV=0`, S24_3LE stereo, 48 kHz, 4x gain, converted
  to s16 mono without changing the sample rate.
- Z1Pro: `rtsp://192.168.144.108/`, H.264 source, 1920x1080, requested 30 FPS
  and 2.5 Mbit/s LiveKit encoding.
- Go2: local WebRTC at `192.168.123.161`; `agent-voice` is played as 48 kHz
  mono 20 ms PCM through a 120 ms prebuffer and a bounded 3 second queue.
- Speaker volume: VUI 10/10 is applied and read back at every session start.

Exact track names are `robot-microphone`, `z1pro-camera` and `agent-voice`.

## Media state and wake state are separate

The LiveKit media connection may remain `live` while the conversation is
`sleeping`. The platform wake detector must not forward ordinary sleeping-state
speech into the conversational model. It opens the gate only for `小玖小玖`;
the ASR homophone `小九小九` is accepted. The acknowledgement is `我在`.

Each valid user turn refreshes a 30 second activity deadline. `结束对话`,
`不用了`, `你休息吧` or `退出对话` returns to sleeping immediately. This is a
platform ASR responsibility in the first production slice; a local offline KWS
model is a later enhancement and is not required for the first release.

## Local control contract

Default socket:

```text
/var/lib/gogoguard/interaction/control.sock
```

The existing GoGoGuard heartbeat adapter sends one JSON object plus newline and
reads one JSON response. Maximum request size is 64 KiB.

Status:

```json
{"action":"status"}
```

Start (the token is ephemeral and memory-only):

```json
{
  "id": "platform-command-id",
  "action": "start_live",
  "params": {
    "url": "wss://gogoguard.cn",
    "room": "patrol-test-001",
    "token": "<short-lived LiveKit JWT>",
    "publishVideo": true,
    "publishAudio": true,
    "subscribeAudio": true,
    "publishData": true
  }
}
```

Stop:

```json
{"id":"platform-command-id","action":"stop_live"}
```

Platform ASR wake event (send every finalized transcript; the robot-side gate
mirrors the authoritative wake state and returns the action the platform must
apply):

```json
{"id":"platform-event-id","action":"wake_transcript","text":"小玖小玖"}
```

On `action=wake`, publish the returned `acknowledgement` (`我在`) on
`agent-voice` and begin forwarding subsequent turns to the model. On
`action=ignore`, do not forward the sleeping-state utterance. On
`action=sleep`, stop forwarding conversation turns while leaving the media
session live. The local service also evaluates `wake_tick` every 500 ms so a
30 second idle conversation returns to `sleeping` even when no further ASR
event arrives.

The same-room `start_live` is idempotent. A newer valid token replaces the
in-memory reconnect token without restarting a healthy room connection. The
public status never contains the URL or token. Transport errors retain only an
error class and stable error code.

The current IP-only development endpoint requires the robot-local
`GOGOGUARD_INTERACTION_ALLOW_INSECURE_WS=1` setting. A platform payload cannot
downgrade TLS. Production keeps the setting at `0` and requires `wss`.

## Secret and activation policy

The image contains no LiveKit API secret, GoGoGuard development secret,
DashScope key or Unitree per-device key. The commissioned robot provisions only
its Unitree AES key in root-readable `/etc/gogoguard/runtime.env`; install mode
is 0600. LiveKit JWTs arrive through the existing authenticated heartbeat and
remain in memory.

The formal service ships disabled:

```text
GOGOGUARD_INTERACTION_ENABLED=0
```

Commissioning changes it to `1` only after the per-device key and platform
control bridge are present. The edge service treats an enabled interaction
process as required: an unexpected daemon exit causes the managed container to
restart instead of presenting stale readiness.

## Acceptance still required

Offline tests and disposable probes do not prove the formal runtime on the
robot. Acceptance order is:

1. Build the ARM64 image and verify all Python/native imports in-image.
2. Install without activating interaction; verify patrol behavior is unchanged.
3. Enable interaction and perform a static, zero-motion media start/stop.
4. Verify 小玖 identity, wake/sleep behavior, visual freshness refusal and
   ordered interruption.
5. Run a 10 minute session, then run interaction concurrently with an accepted
   patrol while measuring CPU, memory, temperature, network quality and route
   control cadence.
