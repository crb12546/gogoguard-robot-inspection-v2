# Realtime interaction stage handoff (2026-08-09)

## Decision

The standalone realtime-interaction delivery stops at this checkpoint. Do not
schedule another interaction-only GoGoGuard or robot acceptance run. The next
phase starts from the latest field-accepted patrol revision, integrates this
checkpoint, and sends one combined patrol + interaction integration request to
GoGoGuard.

This is a sequencing decision, not a rejection of the implementation. Preserve
the branch and evidence. Do not deploy this branch directly to the robot and do
not treat the disposable probes as a release.

## Preserved implementation checkpoint

- Branch: `agent/realtime-dialogue-v1`
- Worktree at closure:
  `runtime-data/worktrees/realtime-dialogue-v1`
- Base before future patrol integration: `f86d488`
- Product modules: `interaction`, `contracts`, `device_io`, local
  `interaction_edge` composition, and read-only `site_console` status.
- The formal code contains the commissioned BOYA, Z1Pro and Go2 speaker
  profile; LiveKit session state; bounded audio playback; half duplex;
  reconnect; token validation/redaction; versioned Xiaojiu persona; and a
  wake/sleep state contract. It imports no navigation implementation and
  exposes no motion command API.
- Full local suite: 99 tests passed. Python compilation, repository knowledge,
  container-contract validation and `git diff --check` passed.
- A complete Linux/ARM64 build was attempted. The existing ROS/patrol native
  layers progressed successfully to the final Python dependency layer, where
  download from `files.pythonhosted.org` timed out. No formal image receipt was
  produced and no retry is required during this closed stage.
- The formal code was never installed on or connected to the robot.

## Real device evidence retained from the disposable probe

The final pre-formalization probe ran for 185.83 seconds with real BOYA mini 2,
Z1Pro, Go2 speaker and LiveKit. It completed 18 turns, published 9,000
microphone frames and 3,495 1080p frames, received 9,212 platform voice frames,
handled 18 ordered stop messages, and dropped no speaker-buffer audio. Platform
first-audio latency was P50 0.712 seconds and P95 1.248 seconds with no current
error or reconnect. Peak temperature was 57.75 C. It sent zero motion commands.

These measurements prove hardware and disposable media feasibility only. They
are not acceptance of the formal runtime or concurrent patrol.

## GoGoGuard P1.5 response recorded

Source received on 2026-08-09:
`/Users/mac/Desktop/GOGOGUARD_P1.5平台侧交付与自测证据.md`.

Accept for later joint re-verification:

- versioned `xiaojiu-inspection-v1` revision 1 persona with deployment digest;
- identity answers for Xiaojiu and Beijing 0019 Technology Co., Ltd.;
- three-layer visual-freshness protection, including model-session rebuild to
  discard old images and a three-second stale-frame threshold;
- platform desired-live state, short-lived LiveKit JWT issue/refresh,
  `start_live`/`stop_live`, and participant-SID reconnect ownership;
- added persona and visual freshness fields in agent health.

Do not mark the following complete:

1. **Wake gate.** The response contains no ordinary-speech rejection, wake,
   idle-sleep, explicit-sleep or self-wake evidence and exposes no wake health
   state. Its own identity tests received answers to ordinary `你好...`
   utterances without first saying `小玖小玖`, which is evidence that the
   deployed main path was still always conversational. First release ownership
   remains platform ASR/agent gating: sleeping speech must not reach the model;
   `小玖小玖` and ASR alias `小九小九` wake once with `我在`; 30 seconds
   idle or an explicit end phrase sleeps without leaving the LiveKit room.
2. **Robot heartbeat authentication.** A body `robotId` identifies a device but
   does not authenticate it. Before production, choose per-device HMAC with
   replay protection, per-device mTLS, or a rotatable robot-bound bearer token
   over TLS. Do not restore or reuse the development secret.
3. **Production TLS/SNI.** A read-only attempt from the Mac to the documented
   HTTPS agent-health URL ended in a TLS connection reset. This does not prove
   the server is wrong, but it means the claimed `wss://gogoguard.cn` route was
   not independently verified. Capture a receipt from the robot's real 4G
   network in the combined acceptance.

The draft platform reply is preserved at
`/Users/mac/Desktop/GOGOGUARD_P1.5平台交付验收与补充项.md`; do not send it as a
new standalone integration request unless the user explicitly reopens this
stage.

## Required next-phase order

1. Finish and field-accept the patrol work first. Freeze its release commit and
   receipts.
2. Create the combined phase from that patrol commit. Rebase or selectively
   integrate `agent/realtime-dialogue-v1`; resolve manifests, composition and
   deployment against current patrol behavior instead of deploying the old
   interaction base.
3. Inventory and implement the complete agreed capability set in one scope:
   patrol-safe always-available wake dialogue, Xiaojiu identity, current visual
   grounding, interaction observability, authenticated heartbeat control, and
   only explicitly authorized inspection context/tools. Any pause, resume or
   motion request must pass through the patrol safety owner and require its
   success receipt; conversation never owns `cmd_vel`.
4. Finish the robot heartbeat adapter: command-ID deduplication,
   `start_live`/refresh/`stop_live`, interaction state reporting, command result
   observability, and no token/URL persistence.
5. Require GoGoGuard to close wake gating and device authentication in the same
   combined integration request. Do not request another platform-only response
   cycle before the combined contract is ready.
6. Re-run the full suite and build a content-addressed Linux/ARM64 image. First
   install with interaction disabled and regress patrol. Then perform a static
   zero-motion interaction check.
7. Final joint acceptance: ten minutes of wake-driven audio/video dialogue,
   visual loss refusal, interruption, reconnect, and one accepted patrol running
   concurrently while recording CPU, memory, temperature, network quality,
   camera rate and patrol control cadence.

## Start-of-next-task instruction

Read the active product repository `AGENTS.md`, then the latest
`PROJECT_STATE.md`. Do not assume this old worktree is the new base. Locate the
latest accepted patrol commit first, inspect this checkpoint commit, and plan a
selective integration. No further facts are required from the closed Codex
conversation.
