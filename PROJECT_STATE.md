# Current project state

Updated: 2026-08-11

## New-task handoff — start here

This section is the compact authoritative handoff for the whole product, not
only the latest navigation problem. The dated sections below are evidence
history; do not treat an older "next experiment" paragraph as current when it
conflicts with this section or the final paragraph of this file.

### Overall product status

GoGoGuard V2 is a **field-commissioning system with one real mobility vertical
slice**, not a completed inspection product. The robot -> Mac -> cloud -> Mac ->
robot loop has produced real maps and completed real route patrols. V6 mobility
is now frozen as the operator-accepted release baseline. The previously
independent realtime-dialogue implementation has been merged. The selected-
patrol platform lifecycle, complete heartbeat, real battery observation, 10 Hz
map-bound pose uplink and true-stop/body-spin checkpoint slice are deployed in
the current platform-patrol-r5 release while retaining separate module
boundaries and no unsolicited motion. The map/route delivery bundle also exists
on the Mac. A frozen-contract recording-as-configuration checkpoint closure is
now implemented and fully tested locally, but it has not been built for ARM64,
deployed to the robot or physically accepted. There is not yet a robot-verified
end-to-end checkpoint mission.

| Product area | Current reality | Remaining product gap |
|---|---|---|
| Architecture and contracts | Three-end ownership, module boundaries, persisted contracts and generated repository index exist and are tested | Keep contracts stable while later slices integrate |
| Robot hardware and calibration | MID-360S, IMU, FAST-LIO, Z1Pro and Unitree motion boundary are deployed; replacement-sensor calibration is active | Long-duration full-load performance is not yet characterized |
| Recording and transfer | Real bags can be recorded, sealed, hashed, resumed and moved robot -> Mac | Operator recovery and large-transfer UX still need product polish |
| Cloud map production | Mac has submitted real recordings to the pinned Alibaba Cloud GLIM worker and received immutable artifacts; the official GLIM map editor now runs in an isolated cloud session through an SSH-tunneled noVNC window | One real operator-cleaned map still needs to be saved and accepted through the new workflow |
| Map and route assets | Maps have immutable history/labels; the Mac workbench shows gray map points, an editable blue route and an editable green allowed area, and has published a real generation-5 Nav2 keepout candidate. Local generation 7 also binds recorded checkpoints to the optimized execution route and creates a checksummed PCD/route/allowed-area/checkpoint/sample bundle with resumable SaaS upload | The generation-7 path is not deployed or platform-accepted; activation remains platform/operator owned |
| Localization | FAST-LIO plus fixed-map VGICP has localized successfully during real patrols; the active V6 image preserves the trusted anchor, requires three consistent recovery matches and processes only the newest pending LiDAR cloud. The operator accepted the first V6-r4 patrol, and synchronized evidence proves repeated localization holds resumed instead of entering the former stale-frame lockout | Repeatability and a complete retained full-route trace are still needed |
| Navigation and motion | Nav2/MPPI has completed real routes; the deployed V6 runtime uses one MPPI controller, SmacPlanner2D for wider bypasses, one 0.48 m safety circle and the green allowed-area mask. The operator accepted the first V6-r4 patrol for release | Sustained cruise speed, passable bypass/rejoin and repeatability remain commissioning work |
| Field workstation and UI | The Mac page is the working delivery console and includes GLIM/map/route/allowed-area preparation. Local code now adds hold-to-adjust Z1Pro controls, recording-time checkpoint/sample capture, checkpoint audit and resumable platform upload progress at `http://127.0.0.1:8080/` | New controls are locally tested only and remain an engineering/field UI until robot and operator acceptance |
| Development evidence | Robot runtime traces, parameter receipts and Mac-side replay/incident infrastructure exist | Complete automatic IncidentBundle coverage and production retention policy remain partial |
| Inspection actions/evidence | Existing realtime Z1Pro video is integrated; the deployed runtime can true-stop and body-spin. Local code additionally records the camera pose/sample, restores recorded body/gimbal pose, announces, exposes `capture_ready`, accepts continue/retake/skip and resumes the retained suffix | The complete closure is not deployed; physical checkpoint, Z1Pro movement and platform-captured evidence acceptance remain pending |
| Mission/task system | The deployed platform adapter may start/stop only the already selected map-bound route. Local code implements the frozen reliable checkpoint event sequence, announcement wait, verdict dedupe/ack, retake limit, timeout and mission-context pose stream | Deploy and run one real frozen MissionPlan; complete mission/report aggregation remains platform work |
| SaaS integration | Complete null-safe heartbeat, real Unitree battery, realtime interaction, selected-patrol lifecycle and map-bound LiveKit pose-stream code are deployed. Local code adds the frozen asset init/chunk/status/complete uploader and checkpoint event/verdict adapter without automatic activation | Real SaaS upload verification, activation, MissionPlan delivery and checkpoint event/evidence receipt are not yet accepted |
| Realtime dialogue and teleoperation | The formal LiveKit/BOYA/Z1Pro/Go2 `interaction` module and P1.5 heartbeat bridge are deployed on the frozen V6 base; real static audio/video publication, agent-audio subscription, DataChannel wake and physical playback reached `live` with zero motion. The current r5 release retains Go2 volume 10, applies peak-limited 3x downlink PCM gain and records source/output RMS. A bounded latest-only robot pose publisher reuses that DataChannel | Operator listening acceptance of the new gain, reliable audible wake acknowledgement, latency closure, a localization-on pose-stream receipt and concurrent patrol acceptance remain pending; remote teleoperation is not a product capability |

### Proven end-to-end product flow

The following is real field capability, although not every step has final UX or
repeatability acceptance:

1. Robot publishes Livox/IMU/camera/odometry and records a mapping bag while the
   operator drives with the handheld remote.
2. Robot seals the RecordingBundle; Mac discovers and resumes its download.
3. Mac submits the immutable bundle to the Alibaba Cloud GLIM adapter and
   receives map PLY/JSON/SVG plus the optimized trajectory.
4. Mac stores map history and labels, visualizes map artifacts, derives the
   recorded map-bound route and publishes the selected candidate to the robot.
5. Robot runs FAST-LIO, fixed-map localization, Nav2/MPPI, local costmap,
   obstacle classification and the Unitree receiver without requiring cloud or
   SaaS connectivity.
6. Mac starts/stops the engineering patrol workflow and reads live status,
   parameters and bounded diagnostic evidence. A route has completed at 100%
   and a genuinely sealed path has stopped correctly.

This proves the core mobility loop. It does **not** yet prove a complete
inspection mission with checkpoint actions, business task state or SaaS result
delivery.

### Development journey compressed from this Codex task

This task spans the product reset and many real commissioning cycles. Preserve
these decisions and lessons when continuing in a new conversation:

1. **Six parallel conversations exposed the original governance failure.** The
   product topics looked separate, but their code changed shared deployment,
   runtime and contracts without one integration owner. The response was not
   dozens of microservices: it was one modular monorepo, explicit ownership,
   one integration branch, generated module knowledge and evidence-backed
   deployment receipts.
2. **The product goal was narrowed before more code was written.** The first
   deliverable is a robot that can record a site, produce a usable map, localize
   in it and repeatedly patrol a route. Camera inspection actions, SaaS tasks,
   realtime dialogue and remote movement matter, but they are separate layers
   that should be added only after the mobility foundation is measurable.
3. **The old application was rejected, but its capabilities were not.** The old
   AI-written repository had never completed a deployment, so V2 did not keep
   its application structure. Livox, FAST-LIO, GLIM, small_gicp, Nav2/MPPI and
   Unitree sources were frozen by commit/tree hash and selectively composed
   behind new module boundaries.
4. **The deployment architecture was corrected to three ends.** Robot runs one
   ROS 2 Humble edge container and owns live sensor/motion traffic. Mac is the
   delivery workstation and Git/release boundary. Alibaba Cloud runs only the
   pinned GLIM job. This replaced early behavior that made the robot pull code
   or upload large recordings directly and made field failures hard to see.
5. **The real hardware/software baseline was commissioned.** The Mac gained an
   ARM64 builder and checksummed offline release flow. The JetPack 5 / Ubuntu
   20.04 robot runs the Ubuntu 22.04 / Humble product container under an enabled
   systemd service. The 100 TOPS Orin NX has run the current sensor stack at low
   memory use; the 40 TOPS option has not been benchmarked and must not be
   claimed equivalent.
6. **Sensor and observation capability became real.** Livox, internal IMU,
   FAST-LIO, the fixed 32.242667 degree mount calibration, Z1Pro RTSP -> WebRTC
   preview and Unitree SDK motion boundary were integrated. A failed original
   MID-360 was replaced by MID-360S `ARMCP6B0035634`; identity/config changed
   while the operator-confirmed rigid geometry was retained. Host clock jumps,
   NTP startup and IMU display under-counting were separated from actual sensor
   faults.
7. **The mapping workflow moved from fragile upload to a product data flow.**
   Real bags were recorded and sealed; early `load failed`, broken-pipe,
   progress and direct-upload problems led to resumable robot -> Mac transfer,
   Mac -> cloud orchestration and immutable map return. GLIM has produced real
   maps, Mac displays 2D/3D assets and history, and selected map/route candidates
   can be published to the robot. On 2026-08-10 the official GLIM editor became
   an isolated cloud tool opened from the Mac, and independent blue-route plus
   green-allowed-area editing became part of the workstation.
8. **Localization and patrol moved through real failures, not a demo.** Early
   runs exposed non-convergence, incorrect initial assumptions, narrow-door
   interpretation, stale selected maps, duplicate/orphan processes, misleading
   `timeout` UI, unavailable stop control and motion-control contention. The
   system gained fixed-map VGICP status, guided asynchronous operations, one
   persistent supervisor, owned process cleanup and an explicit stop-and-release
   path.
9. **Obstacle handling was redesigned after patch-by-patch tuning failed.**
   Logs showed that controller abort, localization loss, actuation stall and
   true obstruction had been conflated. V2 now classifies them separately,
   requires healthy temporal costmap evidence and keeps one MPPI controller for
   both the blue route and SmacPlanner2D bypasses. A real changed-room table
   formed a sealed dead end; stopping was correctly verified. A passable
   bypass/rejoin is still not field-accepted.
10. **Debugging became a first-class development capability.** The Mac gained
    parameter views, operation receipts, incident synchronization/replay and
    layered runtime evidence; the robot records raw/smoothed/collision/final
    command chains, odometry, localization, costmap and controller state.
    Diagnostics are development infrastructure and may be reduced later, but
    current algorithm decisions must use these traces instead of operator
    impressions or repeated code grep alone.
11. **Recent field results prove both progress and remaining limits.** Real
    patrols have completed, localization has tracked accurately, a real dead end
    stopped correctly, and remote-control release passed. The latest speed
    release shortened the same route by about 9% and reached 0.551 m/s peak,
    while actual mean remained about 0.30 m/s. That is a partial improvement,
    not completion of speed commissioning.
12. **Parallel feature work now has an explicit integration boundary.**
    Realtime dialogue was developed on its own branch and has now been merged
    into the V6 integration branch as the independent `interaction` module. It
    ships disabled by default, is explicitly enabled in the current platform-
    patrol-r5 joint-test configuration and cannot issue motion commands.
    Checkpoint work composes it through contracts instead of merging media or
    cloud-model code into navigation.

The recurring lesson is to diagnose the whole ownership and data path before
changing a local parameter. A field symptom may be algorithmic, orchestration,
deployment, stale-state, UX or evidence-quality failure; the system now has
boundaries and receipts to distinguish them.

### Product and execution boundaries

- The current accepted mobility loop is record map -> cloud GLIM ->
  review/publish -> fixed-map localization -> recorded-route Nav2/MPPI patrol.
  Realtime media is now present in the same source tree but remains a separate
  module and activation path. Inspection, SaaS dispatch and remote
  teleoperation must not be mixed into navigation debugging.
- Robot owns live ROS, localization, Nav2, obstacle handling and Unitree motion.
  Mac owns `http://127.0.0.1:8080/`, history, transfer, review, orchestration and
  release publication. Alibaba Cloud owns only the pinned GLIM job. GitHub
  communicates with Mac only.
- Preserve the established Livox/FAST-LIO/GLIM/small_gicp/Nav2/Unitree stack.
  Change owned V2 composition and parameters from measured evidence; do not
  replace algorithms or add services/containers as a reflex.

### Current deployed and live snapshot

- Active product worktree: this repository on branch
  `agent/three-end-field-workstation`. Operator-accepted V6 was frozen in
  `922b301`; the independent interaction branch was merged in `6e842d1`; the
  combined realtime/platform release implementation is `090635c`. Deployment
  implementation and receipts through `69f4849` are published to
  `origin/agent/three-end-field-workstation`; always check `git status` for a
  later local handoff commit. Checkpoint inspection code is deployed but remains
  dormant until a platform MissionPlan and explicit physical test start it.
- Last verified robot release is
  `gogoguard-robot-inspection:v2-edge-20260811-platform-patrol-r5`, built from
  platform integration commit `495d41b`, Unitree battery correction `a85708b`
  and peak-limited threefold speaker gain commit `a1c279f`. Its Linux/ARM64
  local manifest-list ID is
  `sha256:56e79f9c253c1faad055b1bf3d54e3ae60cb1628527e91581e891c41dc935364`;
  the robot-loaded config ID is
  `sha256:adbac4e22f9e523be3c37e9dbe0eb0eb94e89f661bfa7ac0f06438557faf45f4`.
  The enabled service is active with zero restarts, the platform heartbeat is
  online through the frozen HTTP IP endpoint and all four media booleans are
  true. The r5 media receipt exposes `outputGain=3.0`, source/output RMS and
  zero speaker drops or underflows. The real Unitree LowState observer most
  recently reported 20 percent and 27.35 V.
  Nav2, patrol runtime and the Unitree motion bridge are stopped and UDP 5005
  is unbound. Always re-read live robot state before deployment or motion
  because the operator can change it between tasks.
- The 2026-08-10 V6 navigation and latest-only VGICP recovery code is now
  deployed and has received an operator-accepted physical patrol. The selected
  robot candidate is
  generation 5 for `map-6855ba54ae11`, bound to saved workspace revision 4 and
  its green allowed area. The KeepoutFilter assets were generated and verified; starting the
  runtime and completing further physical patrols remain explicit operator
  actions.
- The commissioned Mac workstation was restarted from this worktree and is
  listening on `http://127.0.0.1:8080/`. New navigation-workspace and
  GLIM-editor status APIs return valid JSON. `map-799f6f04e11d` workspace
  revision 6 saved successfully and was published to the robot as a matching
  generation-5 candidate.
- Alibaba Cloud has the official GLIM editor launcher and exporter with hashes
  `b73d22d8c75654ee931df5f962063378688f81a9b20490701c29aa87cb2fdb78`
  and `4e571d1816adad1e5438dffe18a3f14223e83021d90028c1ce727a08ed1907d9`.
  A real dump loaded in the editor, both VNC services bound only to cloud
  loopback, noVNC worked through a Mac SSH tunnel, and the export probe created
  PLY/JSON/SVG/build artifacts. Probe sessions were stopped and removed.

### 2026-08-10 map/localization/navigation simplification

- The map preparation workflow is now explicit: optionally clean transient
  people/cars with the official GLIM editor, then save one blue patrol route
  and one green allowed polygon. The original GLIM artifact is immutable;
  publishing a cleaned result creates a new child map version.
- Workspace validation rejects self-crossing green polygons, a route outside
  the green area, and a route closer to the boundary than the single 0.48 m
  robot radius. Candidate preparation produces a standard PGM/YAML keepout
  mask consumed by both local and global Nav2 costmaps.
- Fixed-map localization remains VGICP. Recovery candidates are compared with
  the last trusted `map -> odom` anchor; one plausible but wrong match can no
  longer recenter the search immediately. Three mutually consistent recovery
  candidates are required before committing a new anchor.
- Runtime navigation has one controller plugin, `FollowPath` using MPPI. A
  temporally verified occupied route asks Nav2 `SmacPlanner2D` for a path to a
  future point on the blue route; the same MPPI executes that path. The old
  custom A* -> RPP runtime handoff and layered stop/slow rectangles are retired.
- If Smac cannot yet find a path, the dog holds zero velocity and retries at
  the configured cadence until a path appears, runtime health is lost, or the
  operator explicitly stops. Collision Monitor retains one visible 0.48 m
  emergency circle; the Unitree receiver retains only authorization,
  freshness, finite/range and hardware-limit checks.
- Offline receipt: 100 Python unit/integration tests pass; Python compilation,
  UI smoke, cloud/robot deployment-contract checks, JSON validation, generated
  knowledge validation, JavaScript syntax, shell syntax, diff whitespace and a
  clean VGICP patch dry run all pass. A browser loaded the live Mac page with
  no console warnings/errors and verified the workbench layout after the
  history panel was bounded. This is an implementation/static receipt, not a
  robot deployment or physical patrol receipt.
- Robot static deployment receipt: Linux/ARM64 release
  `gogoguard-robot-inspection:v2-edge-20260810-v6-r1` was packaged as a
  1,172,066,816-byte archive with SHA-256
  `96ccafcc27fd14a31b0e6aefddfe4f48202ce3fda800b193f20fa026d42f5f45`,
  verified on both Mac and robot, installed and activated. The running image
  config digest is
  `sha256:2c2ab1d140b92bd1e9efd88c8d769b20edcb157ca41934482ea9b742f2cb83ed`.
  Startup reported LiDAR 9.9 Hz, IMU 57.7 Hz and odometry 10.9 Hz with one
  navigation supervisor. Patrol runtime and motion bridge remained stopped,
  final command remained null and UDP 5005 remained unbound; no physical
  motion was authorized or commanded.
- First V6 field start was blocked before motion because the selected
  generation-4 candidate had no `allowed_area_mask`. The operator then drew a
  green area, but saving failed with `EACCES`: the new workspace store had
  incorrectly placed mutable `navigation-workspace.json` inside the deliberately
  read-only GLIM `artifacts` directory. The correction stores workspace state
  in the writable map-job root, reads the old location for compatibility, and
  regression-tests a read-only artifact directory. Runtime preflight now
  rejects an old candidate before starting the Unitree bridge and tells the
  operator to save and republish instead of exposing a raw missing-key error.
  The Mac UI now treats a map as published only when candidate generation is
  at least 5 and its workspace hash matches the saved blue/green workspace, so
  the old same-map generation-4 candidate can no longer hide the republish
  action. Robot map publication includes the separately stored workspace in
  the checksummed import bundle, allowing the deployed V6 compatibility reader
  to prepare the generation-5 mask without making the Mac GLIM artifacts
  writable.
- The next field publish exposed a second ownership error: immutable GLIM map
  files for `map-799f6f04e11d` had already been committed on the robot, while a
  newer mutable `navigation-workspace.json` remained in the staging area. The
  edge importer rejected that revision with `immutable staged artifact already
  exists with different content`. The corrected transfer contract keeps PLY,
  map JSON, overview and GLIM build metadata immutable but permits the
  separately named operator workspace to advance atomically. Repeated deploys
  update only that workspace; tests prove the committed map PLY inode and hash
  remain unchanged.
- Static deployment receipt for that correction: Linux/ARM64 release
  `gogoguard-robot-inspection:v2-edge-20260810-v6-r2` was packaged as a
  1,172,071,424-byte archive with SHA-256
  `9d2e10c46568d0b20007371cb4294eae5f9a021460634ded1ce374b23beed3b2`,
  verified on Mac and robot, installed and activated. The running image config
  digest is
  `sha256:c03956afae5004147c0825cc4870f0881a0e05c3df06c2ab300e4199d595d15a`.
  Startup reported LiDAR 10.5 Hz, IMU 66.0 Hz and odometry 9.6 Hz with one
  supervisor. Runtime and motion bridge remained stopped, the final command
  remained null and UDP 5005 remained unbound.
- Publication receipt: `map-799f6f04e11d` workspace revision 5 and workspace
  hash `826e1b06f15d1e21adf1b7aa5c4608c315e327e5d26dff335ae522d1ab2bd976`
  produced a generation-5 candidate with 24 route points, a 9.645 m route and
  PGM/YAML/JSON allowed-area-mask artifacts. The robot import descriptor
  includes the 1,696-byte workspace with file SHA-256
  `5ab102e14fedf4b812ffe6672d650856c4fe2d6442d49f8e586ce93c494b3b07`.
  No navigation or movement command was issued.
- The operator subsequently saved workspace revision 6 and started the runtime.
  The allowed-area mask server loaded its 176 x 226 PGM at 0.10 m/cell,
  KeepoutFilter and SmacPlanner2D configured, and VGICP tracked the exact map at
  about 0.87 confidence. Nav2 then failed while configuring MPPI CostCritic:
  the V6 simplification had replaced the local polygon footprint with the one
  0.48 m `robot_radius`, but retained `consider_footprint: true`. Humble rejects
  that contradictory radius-only/polygon-check combination. The lifecycle
  manager therefore aborted before FollowPath activation and no costmap frame
  existed; the later `COSTMAP_MISSING` patrol gate was the correct downstream
  symptom, not the cause. Both patrol requests remained unauthorized and sent
  no route.
- The V6-r3 correction sets CostCritic to consume the inflation-expanded
  circular costmap, keeps one 0.48 m robot radius and one 0.48 m emergency
  circle, and makes the strict Nav2 profile validator reject both polygon mode
  and a second footprint property. An isolated ARM64 ROS lifecycle receipt
  configured `controller_server`, MPPI and its sole `FollowPath` plugin
  successfully and reached `inactive`; the original fatal error was absent.
  This validates configuration without connecting a Unitree motion receiver.
- Static deployment receipt for the correction: Linux/ARM64 release
  `gogoguard-robot-inspection:v2-edge-20260810-v6-r3` was packaged as a
  1,172,071,936-byte archive with SHA-256
  `71008bdba3f0c9a70e9fec9a94bad84a016c588d0609404d54dc0897f5217d62`,
  verified on Mac and robot, installed and activated. The running image config
  digest is
  `sha256:36bac21cf840ac47985f8bb3853bdf09c1031f215cd4628a14dbe1dff7f636bb`.
  Startup reported LiDAR 9.8 Hz, IMU 74.2 Hz and odometry 9.8 Hz. Runtime and
  motion bridge remained stopped, UDP 5005 remained unbound and no real-robot
  runtime or patrol start was issued after deployment.

### 2026-08-10 V6-r4 latest-cloud localization correction

- The operator published generation 5 of `map-6855ba54ae11`, workspace
  revision 4, with a 318.781 m route, 722 raw waypoints, a 2181 x 3225
  allowed-area mask and a 2,104,686-point fixed map. The selected robot asset
  matched the Mac selection; this was not a stale-map or publication failure.
- Live evidence showed VGICP repeatedly finding the placed pose with 0.93--0.95
  inlier ratio, 0.033--0.043 fitness MSE and only 0.009--0.021 m correction.
  Recovery nevertheless cycled from two accepted confirmations to
  `stale_input` and zero confirmations. Large-map recovery registration took
  about 1.18--1.31 seconds while the accepted input-age limit remained 0.35
  seconds, so the single-threaded callback replayed queued clouds after each
  expensive registration. Restarting or resetting localization reproduced the
  same scheduler defect and could not correct it.
- The owned V2 localization patch now configures the cloud subscription as
  sensor QoS with `KeepLast(1)`. A stale queued frame is discarded before
  time-aligned odometry lookup, initial evidence accumulation or VGICP work;
  the discard updates diagnostic status but does not call the localization
  failure state machine and does not clear an existing recovery-confirmation
  cluster. The three required candidates therefore remain independent fresh
  frames, while the protection against the earlier 3.58 m false jump remains.
- Offline receipt: 102 tests pass; Python compilation, container-contract,
  generated-knowledge and diff checks pass; the owned patch applies cleanly to
  the frozen stack. The complete Linux/ARM64 image compiled all ten ROS
  packages, including `go2_map_manager`, and an in-image check verified the
  latest-only QoS, stale-discard status field and sealed V2 quality profile.
- Static deployment receipt: release
  `gogoguard-robot-inspection:v2-edge-20260810-v6-r4` has local manifest-list
  digest
  `sha256:b8ca1b65ede57fa0f7ca2cfefa3c1490b905717aa3681d245128fa4e9081269c`.
  Its 1,172,074,496-byte archive passed SHA-256 on Mac and robot with digest
  `a5e9a391da62974e741b577a198d48e5599a4fa15d70f2b35161b02b795d9e19`;
  the running robot config digest is
  `sha256:76aaf66dd2234694e564d9777a122d7c1051d1636f02616a67195aa7dbc33d56`.
  The service is active and enabled with LiDAR 10.0 Hz, IMU 42.9 Hz and
  odometry 10.0 Hz. Exactly one navigation supervisor is present; Nav2 and the
  Unitree motion bridge remain stopped, UDP 5005 is unbound and no movement or
  localization-start command was issued after deployment. Real fresh-three-
  frame localization remains the next operator-authorized receipt.

### 2026-08-10 V6-r4 operator field acceptance

- The operator subsequently ran the V6-r4 candidate and reported the run as
  satisfactory, authorizing this navigation/map revision as the release
  baseline for the next combined inspection and realtime-interaction phase.
- Mac-synchronized production IncidentBundles cover route progress from 0.6%
  through approximately 32% on runtime instance
  `a9fe102280064f3486a4d4477a85f81c`, map `map-6855ba54ae11` and route
  `route-6855ba54ae11-workspace-r4`. They show multiple short
  `LOCALIZATION_NOT_TRACKING` or `LOCALIZATION_POSE_STALE` holds followed by
  automatic continuation, with `resumeCount` increasing to at least 6. This
  is direct evidence that the old queued-cloud behavior no longer clears the
  recovery cluster into a permanent lockout.
- Those synchronized bundles are partial, contain only one sampled decision
  each and do not include the final runtime trace. They independently prove
  recovery behavior only through the retained 32% window; they do not prove a
  100% 318 m route completion. The release decision therefore records the
  operator's full-run acceptance separately from the bounded machine evidence
  instead of overstating the retained receipt.
- No additional navigation parameter change is justified by this acceptance.
  The V6 mobility implementation is frozen as the integration baseline;
  realtime dialogue, checkpoint actions and platform judgment must be added as
  separate modules and must not take ownership of `cmd_vel`.

### Latest verified patrol and current diagnosis

- The first physical `complete-r1` patrol selected the 318.997 m,
  1,119-waypoint `map-6855ba54ae11` route, which runtime densified to 2,541
  poses. From 20:03:54 to the operator stop at 20:07:59 CST, it reached only
  index 364 (14.3%) with 275.43 m remaining. The last place at which the
  operator could not regain localization was therefore not the route endpoint.
- Runtime intentionally entered `HOLDING` eight times with
  `LOCALIZATION_NOT_TRACKING`. This is the configured localization-loss
  motion hold, not a periodic stop: localization becomes unusable after about
  1.0 s without an accepted pose, enters recovery after about 2.5 s, and the
  route suffix is resubmitted after 0.5 s of stable recovery. The earlier
  holds recovered and continued, which matches the operator observation.
- The triggering localization behavior was not healthy. Across the trace the
  localizer reported 56 `not_converged`, 23 `translation_jump`, 65
  `localization_input_stale` and 12 `localization_input_timeout` results.
  These short failures exhausted the freshness window and caused the eight
  protective holds; they were not obstacle, A*, MPPI or collision-monitor
  decisions.
- The final unrecovered hold began at 20:07:02 around map pose
  `(3.098, -40.227)`. During the stopped interval FAST-LIO odometry moved only
  0.066 m, but recovery accepted two VGICP matches and recentered the map pose
  at `(4.779, -43.366)`, a 3.58 m change. Recovery mode does not apply the
  normal 0.75 m tracking jump limit; it accepted this result because it was
  still inside the broad 4.0 m recovery radius plus margin.
- That false in-zone acceptance became the lockout center. The following 15
  recovery results had strong geometry (roughly 0.86--0.90 inlier ratio and
  0.046--0.065 fitness MSE) but were all rejected as
  `initial_result_outside_declared_zone`. The last accepted localization aged
  to about 49.3 s before the operator stopped. `COSTMAP_ROBOT_OUTSIDE` and
  controller transform-extrapolation messages were downstream consequences,
  not the initiating failure.
- The field root cause is therefore defined as: patrol-time localization
  recovery can accept a false but in-radius pose jump, propagate it as the new
  recovery center, and then reject subsequent likely-correct matches outside
  that shifted zone. Navigation's localization hold acted protectively once
  the pose became untrustworthy and should not be classified as the root
  cause. No runtime or parameter correction has been made yet.
- Complete trace and checked evidence were synchronized under
  `workstation-data/field-runs/20260809-120334-complete-r1-map685`. All 56
  captured files passed their IncidentBundle hashes. The eight bundles remain
  partial because the persisted diagnostic profile still has zero-second
  pre/post windows; the complete runtime trace supplied the localization,
  odometry and runtime sequence used above.
- The first physical `continuous-r1` patrol on `map-799f6f04e11d` regressed
  severely versus `cruise-r1`: the operator stopped it after 86.9 seconds at
  94.7% and about 0.55 m remaining. Nav2 emitted 52
  `Optimizer fail to compute path` aborts; all 52 decisions reported a healthy
  costmap and `routeObstructed=false`, and no localization-loss state occurred.
- The failures aligned with route curvature: the first occurred at 44.0%, just
  after an 84.7 degree corner at 41.8% route length; the second sustained stall
  occurred at the final 31.1 degree bend around 94.4%. One valid interval moved
  from 45.3% to 92.0% in about 10.5 seconds with a 0.548 m/s five-second mean
  command, proving the receiver and robot can execute the higher straight-line
  speed.
- The field root cause is V4's non-curvature-aware MPPI speed shaping, not an
  obstacle stop or receiver cap. `VelocityDeadbandCritic` was configured with
  a 0.60 m/s forward deadband while `vx_max`/`vx_std` rose to 0.90/0.40. This
  penalizes the low predicted velocities required through corners and terminal
  convergence; MPPI's CostCritic then found all sampled trajectories colliding.
  Continuous recovery amplified the regression by removing the retry bound and
  clearing/reissuing the same suffix approximately every 1.1 seconds.
- A local profile V5 correction is now prepared but not deployed. It removes
  `VelocityDeadbandCritic`, treats 0.60 m/s only as MPPI's forward ceiling,
  restores the route-completing 0.40 rad/s turn, 0.40 m/s detour and 0.90 m/s2
  acceleration/deceleration envelope, and retains V4 continuous recovery plus
  the independent 0.90 m/s receiver hard ceiling. Existing V4 profiles with
  the exact failed speed tuple migrate deterministically; custom profiles are
  preserved. One complete-route physical receipt is still required.
- The correction is committed as `e651e84f92ae084a03031beb8098972863f2fcd2`
  and built for Linux/ARM64 as
  `gogoguard-robot-inspection:v2-edge-20260809-complete-r1`. The local image
  manifest-list ID is
  `sha256:2f4b77eba52151813a3d6143f2bee9bdbb498973b47b67f0dc7ca6a5c99ed495`
  (1,171,981,741 bytes). In-image checks confirmed profile V5, no
  `VelocityDeadbandCritic`, MPPI/smoother 0.60 m/s, detour/turn 0.40 and
  acceleration/deceleration 0.90 m/s2. The 1,172,021,760-byte immutable
  release archive has SHA-256
  `2bbda1667274637db28c291411d948c156219da37ea9db1fecb850cd11f15448`.
  The archive passed SHA-256 on the robot and was installed successfully.
- `complete-r1` is now the active enabled robot release. The running ARM64
  image/config ID is
  `sha256:2e704e529849848bd31ec795c093fc0974973a474d6ba27f35a9c7dd94917183`.
  Static startup reported LiDAR 10.0 Hz, IMU 98.8 Hz and odometry 10.0 Hz with
  no new service-log errors. The API exposes profile V5 revision 4 with
  target/headroom/detour/turn/lateral values 0.60/0.90/0.40/0.40/0.20 and
  acceleration/deceleration 0.90/0.90 m/s2. Candidate
  `map-799f6f04e11d` remains selected. Nav2 and the Unitree motion bridge are
  stopped and UDP 5005 is unbound, so this is a static deployment receipt, not
  a motion or route-completion receipt.
- Evidence was synchronized to
  `workstation-data/field-runs/20260809-105714-continuous-r1` and 52 Mac-side
  IncidentBundles passed all 364 file hashes. The bundles exposed a separate
  deployment defect: the persisted production diagnostic profile remained at
  zero-second pre/post windows, so every recovery produced a one-sample partial
  bundle missing command and localization topics instead of the intended
  lightweight 15 s/5 s ring.
- The latest measured speed receipt used `map-799f6f04e11d`, not the candidate
  currently selected above. It completed the 9.805 m route normally at 100%,
  with localization confidence about 0.90, no detour and no stop.
- `cruise-r1` reduced same-route completion time from 33.20 to 30.20 seconds.
  Odometry-integrated mean speed was 0.297 m/s; final command mean/p90/max were
  0.342/0.435/0.551 m/s, and 72.6% of nonzero final commands remained below
  0.40 m/s.
- MPPI raw mean was 0.349 m/s and final mean was 0.342 m/s. Collision handling
  therefore removed only about 2%; the remaining speed limit is primarily in
  MPPI/path-follow behavior, not the receiver, Mac workstation, obstacle-stop
  policy or a DetourPath state.
- The exact trace is robot runtime log
  `/var/lib/gogoguard/navigation/logs/runtime-20260809-072829-2656-part001.jsonl`.
  The immediately preceding comparison trace is
  `runtime-20260809-071035-63558-part001.jsonl` in the same directory.

### Current deployment — static receipt passed; first field motion receipt failed

- The navigation-profile V4 change set is deployed as
  `gogoguard-robot-inspection:v2-edge-20260809-continuous-r1`. The managed
  service is enabled and active. Deployment authorization did not include
  starting localization/Nav2, the Unitree motion bridge or physical motion;
  all three remained stopped and UDP 5005 remained unbound throughout the
  receipt. The first later physical route acceptance failed with the 52-abort
  recovery loop documented above; `continuous-r1` is not field-accepted.
- The controller now distinguishes a 0.60 m/s measured cruise objective from
  0.90 m/s controller/receiver headroom, migrates the old 0.90 m/s2
  acceleration to 2.00 m/s2, adds 2.50 m/s2 deceleration, and gives MPPI an
  explicit forward-velocity objective instead of treating 0.60 only as a cap.
- MPPI abort, costmap interruption, actuation stall and unsuccessful local A*
  are recoverable task states. Retry counters are diagnostic only; the runtime
  retains route progress and continues rate-limited MPPI/A* search until route
  completion or explicit operator stop. Immutable binding, invalid schemas,
  non-finite data, Unitree-reported faults and the receiver watchdog remain
  hard boundaries.
- The short-dropout defect that left an accepted goal stuck in `HOLDING` is
  corrected. Evidence-recorder health is observable but no longer gates
  motion. Production evidence now keeps a bounded lightweight 15 s pre/5 s
  post ring and seals `HOLDING`, `RECOVERING` and `SEARCHING_PATH` incidents.
- The transient StopZone correction rejects the calibrated floor-return band
  and requires nine coherent in-zone points rather than four. It does not
  remove Collision Monitor, footprint collision checks or the Unitree command
  watchdog.
- Candidate generation 4 selects `go2-vgicp-orin-v2`: a 24 m scan and 32 m
  local target replace the V1 35 m/45 m compute envelope after the incident
  reached about 144k target points and 526 ms. Quality thresholds remain
  unchanged and V1 remains readable. New field timing and localization-quality
  receipts are required before promotion.
- Offline receipts pass 87 unit tests, Python compilation, Site Console UI
  smoke and container static validation. The complete Linux/ARM64
  image `gogoguard-robot-inspection:v2-edge-20260809-continuous-r1` built all
  ROS packages with local image/manifest-list ID
  `sha256:59b1eff3aa1d394a963e2a1bcbe95430badf020d92a911ef64730501573d83dd`
  (1,171,982,504 bytes). In-image checks confirmed the installed localizer and
  receiver, `go2-vgicp-orin-v2` at 24/32/4 m, 0.60 m/s cruise objective,
  0.90 m/s forward headroom, 2.00/2.50 m/s2 acceleration/deceleration and a
  0.75 s replan cadence.
- The 1,172,022,784-byte release archive passed SHA-256 on Mac and robot with
  digest `6ad4843cdab418831b63bc40b42eb94ccdac80e254dfe54703676979a928ad4e`.
  The robot loaded Linux/ARM64 image/config ID
  `sha256:da0c1c6b7d90f5ba36a89db25f9ae75b98ba16ea39cd3ca97e44aeed9296caf6`.
  A persisted 0.90 m/s experiment initially migrated as the V4 cruise target;
  the deployment receipt corrected the active profile to revision 4 with
  target/headroom/lateral values 0.60/0.90/0.20 m/s. The selected
  `map-6855ba54ae11` candidate was regenerated in place as generation 4 with
  `go2-vgicp-orin-v2`; its runtime-profile binding digest is
  `49419c72179a0ea33ac9f4df0b3fd36ec7b9796f8c1425ccbdb8790e017a5e1f`.
  Service startup reported about 10.06 Hz LiDAR, 200 Hz raw IMU and 10 Hz
  odometry with no core-log errors. This is not evidence that the route can
  yet complete at 0.60 m/s; the required next acceptance is one full route,
  then three consecutive full routes, with automatic recovery and evidence
  review.

### Development roadmap and current priorities

1. **P0 — close the deployed realtime joint acceptance without changing V6.**
   Obtain the platform-side video/participant receipt, make `小玖小玖` reliably
   produce an audible `我在`, isolate the full-volume audio-quality and latency
   cause, and verify token refresh, reconnect and `stop_live`. Then run the
   ten-minute interaction-plus-patrol resource acceptance while retaining V6
   navigation, localization, collision and motion ownership unchanged.
2. **P1 — complete mobility repeatability and use the new map assets on a real
   site.** Retain V6-r4 as the accepted baseline while collecting a complete
   full-route trace and a physically passable Smac bypass/rejoin receipt. Clean
   one GLIM map when
   needed, draw and save its green allowed area, review the blue route and
   publish the resulting immutable child/candidate. Then improve version
   comparison and approval wording from operator feedback rather than adding
   more map layers speculatively.
3. **P2 — build actual inspection execution.** Implement checkpoint actions,
   photo/video evidence, inspection result contracts, and the mission state
   machine that composes route travel with those actions. Keep algorithm
   evidence separate from business inspection evidence while sharing immutable
   identifiers and timestamps.
4. **P3 — connect external product surfaces.** Integrate the existing SaaS
   through a narrow device-agent task/result boundary. Before mission binding,
   agree and implement a Mac-owned upload for the selected immutable map PCD,
   map-bound `go2.route.v1`, editable allowed-area geometry and a checksummed
   portable manifest; the current Mac -> robot publication does not publish
   these assets to SaaS. Integrate realtime dialogue and later remote
   teleoperation as separate capabilities; neither may bypass navigation
   ownership or silently take motion control.
5. **P4 — close delivery engineering.** Productize Mac installation/startup,
   define release promotion and rollback, push/review the integration branch,
   and add full-load performance/soak acceptance on the 16 GB Orin NX.

Before any physical test, confirm the operator's intended map and route,
confirm the robot is standing in that map, and obtain explicit authorization in
the current task. A deployment or diagnostic request never authorizes motion by
itself.

### Parallel Codex and Git boundary

- `agent/realtime-dialogue-v1` at `3215515` is a preserved historical
  checkpoint whose implementation was already merged by `6e842d1`. Do not
  deploy that old worktree or continue product work there; the active combined
  source is `agent/three-end-field-workstation`.
- `agent/realtime-interaction-spike` is attached to the superseded
  `gogoguard_robot_inspection_v2` worktree. It is not evidence of current robot
  runtime behavior and must not be deployed.
- All current navigation, interaction, platform and checkpoint integration work
  stays on `agent/three-end-field-workstation` with one declared integration
  owner for shared contracts, container composition and deployment files. A
  task names its primary module, updates its manifest and `PROJECT_STATE.md`,
  passes the required checks, and records commit plus image digest after a real
  deployment. Never infer deployed state from branch HEAD.

## Verified external facts

- Robot: Unitree Go2 expansion dock, 100 TOPS class, Jetson Orin NX 16 GB.
- Robot host: JetPack 5 / Ubuntu 20.04 / L4T R35.3.1; Docker was previously
  verified on the robot.
- Target robot userspace: one Linux ARM64 Ubuntu 22.04 / ROS 2 Humble container.
- Sensor: replacement Livox MID-360S `ARMCP6B0035634` at `192.168.1.134`.
  Camera: Z1Pro.
- Cloud mapping: existing Alibaba Cloud Ubuntu 24.04 x86_64 / ROS 2 Jazzy GLIM
  worker. It first passed a stationary native PointCloud2 + Imu smoke dataset
  and has since processed real operator-driven robot recordings into maps used
  for fixed-map localization and patrol. Map quality review remains an operator
  acceptance step.
- SaaS: existing GoGoGuard platform is retained. Its realtime heartbeat and
  interaction command bridge are deployed for joint testing, but production
  authentication/TLS, map/route/allowed-area asset delivery and the V2 mission
  task/result boundary remain open and are not part of the accepted mobility
  loop.

## V2 implementation state

- New clean repository created locally.
- Local demo vertical slice is implemented: live trajectory and point cloud,
  recording, sealed SHA-256 bundle, asynchronous map job, PLY/JSON/SVG output,
  and result display in the same page.
- 2026-08-05 local checks: 10 unit/vertical-slice tests pass; Python compilation,
  UI contract, HTTP start/record/stop/map flow, and container file contract pass.
- The local HTTP receipt returned 340 live points, 4 recorded preview samples,
  a sealed bundle, and a 340-point demo map. It is explicitly demo evidence,
  not GLIM or robot evidence.
- Product direction corrected on 2026-08-05: the established algorithm stack
  is retained and selectively migrated. V2 rewrites composition and module
  boundaries, not Livox/FAST-LIO/GLIM/small_gicp/Nav2 capabilities.
- The first capability source is locked to old-repository commit
  `3a7597c2af2f06efbe605b4e1ec216450db05d44`; its Livox, FAST-LIO and mapping
  capture paths are clean in the old worktree. Exact Git tree IDs are recorded
  in `dependencies/capability_migration.lock.json`.
- A 406-file, 4.3 MB exact capability snapshot is now present under
  `third_party/locked_stack`; its aggregate SHA-256 is enforced by
  `make knowledge-check`. It contains only Livox SDK2/Driver2, FAST-LIO,
  mapping capture and the old calibration source package.
- The coordinate/calibration subset has been ported into
  `modules/calibration`. The frozen 32.242667 degree mount pitch, robot/sensor
  identity, MID-360 internal IMU transform and composed FAST-LIO base output are
  covered by migration tests.
- The ARM64 Dockerfile compiles the locked Livox SDK2, Livox Driver2 and
  FAST-LIO trees and composes driver, calibration TF, FAST-LIO and Site
  Console. The 2026-08-05 native ARM64 build completed successfully. Its local
  manifest-list digest is
  `sha256:e5ed6cc638b87cfaa0baa37461198c3bc289f4ae8d9b8ea8997db1f2eea2f558`;
  the ARM64 config digest deployed on the robot is
  `sha256:5d996c27735c2fb30125f5ec8c87676c24db7d7fd9d0c02b9b678796e2853ff6`.
- Repository knowledge is now generated from per-module JSON manifests.
  `make knowledge-check` validates module paths/dependencies, the generated
  index and the immutable source snapshot.
- Homebrew, Docker CLI, Buildx and Colima are installed on the Mac. The local
  builder is native ARM64 with 6 CPUs, 10 GiB RAM and a 60 GiB container disk.
- The image contains executable Livox Driver2 and FAST-LIO nodes with no
  missing dynamic libraries. ROS package discovery, Python package imports,
  entrypoint syntax and containerized Site Console HTTP status all pass.
- The first unbounded C++ build exhausted the local builder. Native build
  parallelism is now deliberately limited to two jobs for reproducibility on
  both the builder and the 16 GB Orin NX.
- V2 is deployed as `gogoguard-edge.service`. Production policy is to keep it
  running and enabled at boot. Routine tests do not stop it; an update or
  explicit recovery may perform one managed restart.
- The robot maintenance endpoint `unitree@192.168.123.18` is connected over
  the Mac adapter at `192.168.123.222/24`; three probes had 0% loss and about
  0.9 ms average latency. Key-based SSH succeeds.
- A 2026-08-05 read-only robot audit confirms NVIDIA Orin NX, aarch64, 8 CPUs,
  15 GiB RAM, Ubuntu 20.04.5, L4T R35.3.1, Docker 24.0.5 and 406 GiB free on
  the root filesystem. The `unitree` account needs sudo for Docker.
- The robot wired interface owns `192.168.1.5/24`, and MID-360 at
  `192.168.1.161` answers with 0% loss. No Livox, FAST-LIO or Nav2 process was
  running during the initial audit; the three existing SaaS services remained
  active throughout V2 deployment.
- Real ROS measurements on the robot are approximately 10.07 Hz PointCloud2,
  200.10 Hz IMU and 10.07 Hz `/Odometry`. FAST-LIO initialized and converged,
  and the fixed robot/sensor-bound mount calibration was active.
- A five-minute stationary window passed without runtime error logs. The last
  180 seconds ended 1.48 cm apart with per-axis spans of 4.66--5.18 cm.
- The running container used about 254 MiB (1.65% of robot memory) and 1.34 CPU
  cores; CPU temperature was about 51 C and GPU load was zero. This supports
  the current Livox/FAST-LIO slice on this Orin NX but is not yet a benchmark
  for concurrent camera AI, speech or remote control.
- Real recording `20260805T122228Z-33e75803` sealed successfully: 29.70 seconds,
  150.1 MiB rosbag, 6,539 messages and 171,089,287 bundle bytes. All three
  manifest hashes were recomputed successfully.
- Site Console currently under-counts the IMU messages it consumes even though
  `ros2 topic hz` verifies about 200 Hz. Treat that number as an instrumentation
  defect, not sensor loss.
- The existing Alibaba Cloud host is connected at `39.96.72.215`. The
  2026-08-06 target architecture moves its SSH adapter and key to the Mac
  workstation; the next robot release removes the robot-side cloud mount.
- `/opt/go2/bin/gogoguard-map-job` is installed as a narrow adapter over the
  existing pinned Jazzy GLIM pipeline. It runs the pipeline as unprivileged
  `go2mapping` and returns the V2 `map.json`, `map.ply`, `overview.svg` and GLIM
  build receipt contracts.
- End-to-end job `map-7b26b091874f` passed from robot recording through rsync,
  cloud GLIM and artifact return. The stationary 29.70 second bag produced 267
  optimized poses, one submap and 557 displayed points; the result is an
  engineering candidate, not a usable site map.
- The uncompressed 150.1 MiB public-cloud upload took about 237 seconds and UI
  progress stayed at 10% during rsync. Transfer progress/compression is a known
  experience issue, not a mapping correctness failure.
- The commissioned Z1Pro at `192.168.144.108:554` now appears in Site Console
  through an on-demand WebRTC preview. The pinned MediaMTX v1.20.0 ARM64 gateway
  runs inside the same primary container and remuxes the camera H.264 stream
  without transcoding.
- Real browser sessions passed with the camera's native 1920x1080, 30 FPS,
  H.264 High Profile level 5.2 stream in Chromium and Safari/WebKit. Both peer
  connections established over LAN UDP/ICE and reported zero discarded output
  frames during the receipt window.
- With live camera readers, the complete edge container used about 299 MiB
  (1.94% of robot memory). A one-shot whole-container CPU reading was 1.41
  cores; this includes Livox, FAST-LIO, the UI and video forwarding and is not a
  camera-only benchmark.
- The camera preview is a `device_io` observation capability. Versioned patrol
  snapshots/video remain a separate future `inspection` responsibility.
- V2 has controlled robot motion during the recorded field patrols documented
  below; release installation and static acceptance do not command motion.
- First release target: live device status, 2D trajectory, 3D point-cloud
  preview, start/stop recording, sealed RecordingBundle, cloud map job, and
  visible 2D/3D result.

## Current deployed release

- Active V2 implementation commit: `a615ab9a32fa46dbb73a4f6fc30db6cce2de8054`.
- Robot image config digest:
  `sha256:593a5336f355277e72b513e006a34a61e335b4dc94ae3468c48b52c92a016ab1`
- Robot service: `enabled`, `active`, live status at port 8080
- Active image: `gogoguard-robot-inspection:v2-edge-20260809-cruise-r1`; robot
  runtime no longer mounts cloud SSH material and its map worker is `none`.
- The active release archive SHA-256 is
  `d80fb512d63474e25a3782761f28b851bd526a07ed6e4f5a1c93d59075678142`.
  The corrected host service uses the commissioned robot's Docker-compatible
  `docker stop -t 15` form, treats Docker's normal stop exit 143 as success,
  starts only after the NTP gate, and remains enabled across reboots.
  The final restart receipt recorded `gogoguard-edge.service: Succeeded`, then
  passed the stable-NTP gate and returned to `active` without a failure result.
- On 2026-08-08 the failed original MID-360 was replaced by MID-360S
  `ARMCP6B0035634`. A non-mutating SDK identity probe found it at
  `192.168.1.134`; a direct ROS probe measured about 10.06 Hz point cloud and
  200 Hz IMU. The operator confirmed the rigid mount and installed angle are
  unchanged, so the commissioned 32.242667 degree transform is retained in a
  new sensor-bound calibration record instead of pretending the replacement
  is the original hardware.
- Release `v2-edge-20260808-mid360s-r1` moves the deployed Livox network config
  out of the immutable source snapshot, binds the runtime to the new IP and
  sensor identity, and carries the replacement calibration digest
  `c9b5ade49f128e88df4868c95ad065fafac450217c829497066f2cf491e4a59b`.
  The ARM64 image manifest-list digest is
  `sha256:ee40e8b1fc54ec7331ea5b7cc6da6e8bc6e1f5d1aa04588034e175b891981f88`.
  After checksummed installation and stable-NTP restart, the service remained
  `enabled` and `active`; live ROS measurements were about 10.06 Hz point
  cloud, 200 Hz IMU and 10.06 Hz FAST-LIO odometry. The device API reported
  online, the calibration publisher reported the new serial and digest, and
  Nav2 plus the Unitree motion bridge remained stopped. No motion command was
  sent.
- The first delivery-runtime start for generation-3 `map-648e606fb9a5`
  exposed an integration regression before localization or planning began:
  its 0.60 m/s route profile was rejected by a retained 0.50 m/s runtime
  contract. The asynchronous start was then marked complete too early and the
  Unitree bridge retained UDP 5005, so repeated starts failed with `errno=98`
  while the workstation reduced the incident to a generic `timeout`.
- Release `v2-edge-20260807-delivery-r2` preserves the locked capability
  snapshot and applies an explicit V2 compatibility overlay for the 0.60 m/s
  route contract. Nav2 must now remain alive through a readiness window before
  start completes; both immediate launch failure and later runtime exit reap
  the owned Unitree bridge. The UI keeps the exact asynchronous operation
  failure visible. Offline evidence is 43 passing tests plus compilation,
  knowledge, container-contract, UI and in-image overlay checks.
- The r2 archive passed SHA-256 on the robot and the managed restart returned
  to `enabled` and `active`. The device reported 9.9 Hz LiDAR and 9.9 Hz
  odometry with one supervisor. Nav2, the motion bridge and UDP 5005 were all
  stopped, proving the old orphan was removed; no motion command was sent.
  The current selected candidate remains `map-648e606fb9a5` (18.116 m, 65
  waypoints). Runtime start and patrol are not yet field-accepted on r2.
- 2026-08-06 boot receipt: MID-360 network passed; raw ROS rates were 10.07 Hz
  PointCloud2, about 200 Hz IMU and about 9.8 Hz `/Odometry`; Z1Pro WebRTC and
  workstation proxy status passed. No motion command was sent.
- The Mac workstation completed the new robot-to-Mac-to-cloud flow for the 92
  and 485 sample historical recordings. Jobs `map-e4895ff7ba37` and
  `map-13bbdfcf2824` returned local 2D SVG, 3D JSON and PLY artifacts. A new
  39-second recording also completed as `map-648e606fb9a5`.
- Browser verification showed the 2D overview, rotatable 3D point cloud,
  camera, live trajectory and separate available-map/task history sections.
- The first patrol attempt for `map-648e606fb9a5` was accepted by Nav2 but
  aborted before motion. Livox PointCloud2, Livox IMU and FAST-LIO odometry
  were about 28 seconds ahead of the host clock, so the controller could not
  transform the robot pose into the global plan frame. No nonzero final motion
  command was emitted and obstacle handling was not the cause.
- The clock skew was traced to startup ordering: the Livox no-sync timestamp
  mapper anchored before the host completed its first real NTP synchronization
  and retained the pre-correction epoch. A managed edge-service restart after
  NTP synchronization temporarily recovered sensor/odometry timestamps to
  within about 0.35 seconds of the host. The old navigation fault was cleared;
  no patrol was started during recovery.
- A second patrol was accepted after that recovery and emitted a nonzero
  command within about 0.15 seconds. It then failed Nav2's progress checker:
  the 0.10 m/s command changed posture but odometry advanced only 0.064 m in
  15 seconds, below the configured 0.20 m requirement. Obstacle handling was
  not the cause of this attempt either.
- Release `v2-edge-20260806-r2` is deployed with three field corrections:
  stable-NTP startup gating plus runtime Livox epoch rebasing; fail-closed
  rejection of future-stamped obstacle clouds; and a commissioned Go2 forward
  gait range of 0.20--0.45 m/s with a route speed limit of 0.40 m/s. The motion
  bridge also performs StandUp and BalanceStand before Nav2 starts.
- Post-deployment receipt: NTP reported synchronized, the startup gate reported
  stable realtime, and raw Livox PointCloud2, Livox IMU and FAST-LIO odometry
  stamps stayed aligned with the host instead of retaining the old 28-second
  future offset. Device status reported online at about 10 Hz LiDAR and 10 Hz
  odometry. No Nav2 runtime or motion bridge was started.
- Candidate generation 2 was regenerated for `map-648e606fb9a5`: 65 waypoints,
  18.116 m route, 0.40 m/s route limit and 0.20 m/s MPPI minimum forward
  sample.
- The first moving generation-2 patrol ran for 51.4 seconds with healthy
  fixed-map localization and reduced the remaining route to 6.59 m. At a
  passable narrow section, Collision Monitor held `SlowZone` continuously for
  14.9 seconds: the average 0.341 m/s Nav2 command became 0.153 m/s at the
  configured 45 percent ratio. The Go2 advanced only 0.074 m and Nav2 correctly
  aborted on the 0.20 m / 15 s progress contract. StopZone did not trigger,
  localization remained accepted, and clock skew was not involved.
- A navigation-only correction is deployed from that trace: preserve StopZone,
  narrow SlowZone from +/-0.65 m to +/-0.55 m (body half-width plus 0.35 m),
  and raise slowdown from 45 to 70 percent. The installed r3 profile, active
  service, stable-NTP restart and generation-2 candidate were verified. The
  repeat patrol then completed the 18.116 m recorded route in 91.16 seconds
  with Nav2 accepting the goal and the first final nonzero command emitted
  200.7 ms after the request (`ROUTE_COMPLETE`).
- Outdoor GLIM job `map-28acad0c6b0e` completed from recording
  `20260806T161544Z-47354601`: 660 samples over 263 seconds produced 2,648
  optimized poses, 156 submaps and 3,096,005 displayed points. Its recorded
  route is 191.09 m with 645 generation-2 waypoints. Map quality and patrol on
  this new outdoor version have not yet been field-accepted.
- Release r4 makes workstation-to-robot map deployment idempotent. The robot
  exposes existing immutable artifact sizes and SHA-256 values, the Mac skips
  exact copies, and repeated PUT handling consumes the full HTTP request body.
  With the 52,916,127-byte outdoor map already committed, two consecutive
  prepare operations completed without transfer or `Broken pipe`; both
  returned candidate `map-28acad0c6b0e`. Nav2 and the motion bridge remained
  stopped throughout this receipt.
- Mac workstation commit `5f52229` separates the 3-second robot health-query
  timeout from slow map and navigation operations. Map deployment/preparation
  uses a 60-second bound; navigation start, stop, localization reset and patrol
  operations use a 30-second bound. This corrects false UI `timeout` reports
  for operations which safely continue on the robot after the HTTP client has
  already given up.
- GLIM job `map-e9bc37247129` completed and is the selected navigation
  candidate: 713,497 points, an 82.966 m route and 285 generation-2 waypoints.
  Its four immutable robot artifacts total 14,774,221 bytes. After clearing the
  stopped old runtime, preparation through the updated Mac endpoint returned
  HTTP 201 in 1.72 seconds; Nav2 and the motion bridge remained stopped.
- Robot motion, continuous fixed-map localization and one complete indoor
  recorded-route patrol are now field-verified.

## 2026-08-08 obstacle-detour trace and pending correction

- The replacement MID-360S recording produced new GLIM map
  `map-6a79b7795da6` and a 9.521 m generation-3 route. At patrol start the
  fixed-map pose was 0.057 m from the route origin with 3.53 degrees yaw error;
  Nav2 accepted the goal, motion authorization opened, and localization stayed
  `TRACKING` at roughly 0.90 confidence.
- The robot advanced about 0.82 m before stopping. Collision Monitor did not
  trigger StopZone, the final safety gate reported no obstacle stop, and sensor
  or localization freshness was not the cause. Left/front-left costmap pressure
  increased as the robot approached the operator-placed obstacle.
- MPPI aborted the original FollowPath. Bounded A* found a valid local detour
  in 56 ms and rejoined at route index 18, but MPPI then aborted execution of
  that detour as well. This is a controller execution defect, not proof that the
  physical corridor was impassable.
- The 0.20 m/s minimum forward gait remains backed by field evidence: earlier
  0.10 and 0.153 m/s commands changed posture but made only 0.064--0.074 m
  progress in about 15 seconds. The pending correction therefore does not set
  MPPI's global minimum to zero. Normal patrol remains MPPI Omni at
  `vx >= 0.20 m/s`; only a bounded A* detour selects Nav2 Regulated Pure Pursuit,
  which can rotate with `vx=0` to the detour heading and then advance at the
  effective 0.20 m/s gait before rejoining the recorded route.
- The deployed V2 overlay aligns the Unitree receiver with the commissioned
  0.60 m/s forward and 0.20 m/s lateral planner, smoother and final-command
  caps; both values now survive the complete command chain. The
  official SDK exposes independent `Move(vx, vy, vyaw)` components but does not
  publish a numeric Go2 lateral limit; 0.20 m/s lateral and pure-yaw response
  therefore require a short field receipt before acceptance.
- Workstation and robot UI semantics now distinguish request receipt from
  route outcome, clear stale transient polling errors after a successful
  snapshot, display replanning/detour/blocked/fault separately, and no longer
  summarize a blocked patrol as `可运行`.
- Offline evidence is 48 passing tests, Python compilation, UI contract,
  container contract and an apply-checked immutable-source overlay. The
  complete ARM64 image is
  `gogoguard-robot-inspection:v2-edge-20260808-detour-r2`; its manifest-list
  digest is
  `sha256:86eba1feef46b340641212eef04dba6dca89ec13f0981d0940023c7413cc7e49`
  and ARM64 config digest is
  `sha256:5bf2737b34d7bc326c97012f4284f92a3fe572d73acf369f15b0cada9c7a9e37`.
  An in-image Nav2 lifecycle configure receipt created both `FollowPath`
  (MPPI) and `DetourPath` (Regulated Pure Pursuit) without plugin-load error.
- A real browser refresh against `http://127.0.0.1:8080/` displayed the live
  blocked candidate as `BLOCKED` / `路线受阻`, retained `定位可用`, and described
  the latest completed patrol operation as only `巡检请求已提交`; this verifies
  that request receipt is no longer presented as route completion.
- Release `v2-edge-20260808-detour-r2` was checksummed on the Mac and robot,
  loaded on the ARM64 target and activated through the managed systemd restart.
  The service returned to `active` and `enabled` after the stable-NTP gate. The
  running container reports the expected image/config digest, contains
  `DetourPath` and the `0.60/0.20 m/s` Unitree receiver contract, and used about
  351 MiB during the receipt. Both robot and workstation APIs reported the
  replacement MID-360S and odometry at about 10 Hz, the navigation supervisor
  running, and Nav2, the motion bridge and UDP 5005 stopped. Deployment did not
  command motion; physical clear-route and obstacle-detour acceptance remain
  pending.

## 2026-08-08 latest obstacle incident and deployed correction

- The latest patrol used replacement-sensor map `map-71b045489e8a`, a
  15.816 m generation-3 route with 57 source waypoints and 121 runtime points.
  It ran for 42.4 seconds and reached runtime index 94/121 (78.3 percent).
  Fixed-map localization stayed `TRACKING` at roughly 0.896 confidence.
- MPPI failed near the placed obstacle with `Optimizer fail to compute path`.
  The bounded A* planner found a detour in 4.28 ms and selected route index 107
  as the rejoin anchor. Regulated Pure Pursuit started and the robot advanced
  roughly 0.375 m during that detour window, but 29 final command samples were
  only 0.17 m/s: the configured 0.20 m/s detour was multiplied by the 0.85
  SlowZone ratio and fell below the field-proven effective Go2 gait floor.
- A second independent defect was present throughout the patrol. The rolling
  VoxelLayer observed the sensor origin at about -0.29 to -0.32 m while its
  configured lower Z bound was only -0.15 m. Nav2 repeatedly reported that it
  could not raytrace from the sensor origin. Obstacles could be marked but not
  cleared normally, allowing stale local-costmap obstacles to persist. The
  terminal outcome was `Failed to make progress`; StopZone and localization
  loss were not the cause.
- Commit `f727b68` raises the DetourPath input gait to 0.24 m/s so the existing
  0.85 SlowZone produces 0.204 m/s after the full command chain. It also sets
  the VoxelLayer vertical range to -0.60 through 1.48 m, covering both the
  measured sensor origin and the configured 1.45 m obstacle ceiling. Contract
  tests apply both navigation overlays and prove these bounds remain valid.
- Release `v2-edge-20260808-evidence-detour-r3` combines that correction with
  the bounded incident recorder. The ARM64 manifest-list digest is
  `sha256:29e0706c65a858a26a210d969a07b35f23e1fe556f4e508b113b267a2a7edfff`;
  the robot image config digest is
  `sha256:0553292ae9c376be999f879824e950002df3afd69741b1ad64c2dd3c9297e447`.
  The 1,171,185,664-byte release archive passed SHA-256 on both Mac and robot.
- The managed restart completed after the stable-NTP gate. The service is
  `enabled` and `active`, Livox reports about 10 Hz, the incident recorder and
  sole navigation supervisor are alive, and Nav2 plus the Unitree motion bridge
  remain stopped. Motion authorization is closed and the final command is
  zero; deployment did not command motion. Physical obstacle-detour acceptance
  remains pending.

## 2026-08-08 temporary development incident replay (deployed)

- The evidence gap exposed by the second obstacle test is now explicit: the
  deployed JSONL flight trace records PointCloud2 metadata but not XYZ bytes,
  the historical local costmap or synchronized camera frames. It can prove
  that the runtime returned `LOCAL_PATH_BLOCKED`, but cannot prove whether the
  physical scene actually had a passable corridor.
- A versioned `DiagnosticProfile` now separates `development`, `acceptance`
  and `production` resource envelopes. Development retains a bounded 15-second
  pre-trigger and 5-second post-trigger window with 10 Hz PointCloud2, local
  costmap, TF/localization/commands, H.264 camera segments and planner detail.
  Acceptance uses 2 Hz point cloud without camera. Production records none of
  those heavy streams. A time or patrol-count limit automatically returns the
  robot to production mode.
- The new recorder is a dormant process in the existing primary edge
  container, not a microservice or navigation dependency. It serializes only
  while a non-production profile and patrol are active, writes bounded data,
  never uploads during patrol, and is not part of the velocity command path.
  Camera capture is H.264 remux without decode or encode; replay, rendering and
  analysis remain on the Mac.
- `BLOCKED`, `FAULT` or a manual request freezes an immutable IncidentBundle
  containing checksummed MCAP sensor messages, a browser replay index,
  parameters, route/map references, decisions and available camera segments.
  The local A* overlay additionally publishes start/goal resolution, expanded
  cells, compute time, path or the explicit `invalid_grid`,
  `start_surrounded`, `goal_surrounded`, `expansion_limit` or `disconnected`
  outcome without changing the planner policy.
- The Mac workstation automatically discovers, resumes, hashes and archives
  robot incident files. Its browser UI controls the temporary diagnostic mode
  and expiry and replays video, 3D point cloud, costmap/original route/A*
  search, pose and decisions on one timeline. Missing evidence is displayed as
  missing instead of being inferred.
- The old metadata-only runtime trace can be imported as a truthful partial
  incident. A fixture reproduced the latest `BLOCKED / LOCAL_PATH_BLOCKED`
  endpoint and explicitly listed point-cloud XYZ, local costmap, camera and A*
  search cells as unavailable.
- Offline evidence is 54 passing tests, Python compilation, UI and container
  contract checks, clean sequential application of the delivery and incident
  ROS overlays, and direct verification of the Humble MCAP writer API in the
  completed ARM64 edge image. Real browser checks passed at 1280 px and 390 px
  with no horizontal overflow or console error.
- Built artifacts are
  `gogoguard-robot-inspection:v2-edge-20260808-incident-r1` for Linux ARM64
  (local image ID `sha256:95120e4e71af00c230cd6e324811feadd03214944cd72d957f5caad491916e25`)
  and `gogoguard-field-workstation:20260808-incident-r1` for the Mac
  workstation (local image ID
  `sha256:68b2c4cad1d6fc850403098a53fd4c8a64c03e81aca1ee7820aab92370621238`).
  In-image checks proved the recorder entry point and FFmpeg are installed,
  the production default disables heavy capture, the planner diagnostics
  overlay is present and a real `rosbag2_py` MCAP file can be written.
- The robot portion is deployed in release
  `v2-edge-20260808-evidence-detour-r3`; the commissioned Mac continues to run
  the same workstation source natively at port 8080. The API and browser now
  expose the profile and incident history successfully. Development capture is
  enabled for the next three patrols with the preset 15-second pre-trigger,
  5-second post-trigger, 10 Hz point cloud, local costmap, planner detail and
  camera window. The patrol counter automatically restores production mode.
- Immediately after deployment, the complete edge container used about 405 MiB
  of 15.03 GiB. That is a static receipt only; the first captured patrol must
  compare CPU, memory, disk throughput, LiDAR/odometry rate, controller cadence
  and temperature against the production baseline before the recorder is
  accepted for continued development use.

## 2026-08-09 fall-induced FAST-LIO recovery

- The newest completed GLIM asset is `map-8ddcf3f8c078`: 322,858 displayed
  points, a 63.733 m generation-3 route and 222 source control points. It was
  already selected and published to the robot when repeated attempts could not
  reach usable localization.
- The failure was below Nav2 and fixed-map localization. After the operator
  reported that the robot had fallen and been stood back up, the raw Livox
  stream remained healthy at about 10 Hz point cloud and 200 Hz IMU, but
  FAST-LIO stopped publishing `/Odometry`. Its trace changed at 00:15:57 from
  roughly 60,000 local-map points and hundreds of effective matches per scan to
  zero effective matches, then reported VoxelGrid integer-index overflow. The
  active navigation runtime consequently had no `odom` frame and stayed at
  `LOCALIZING / FASTLIO_ODOMETRY_MISSING`; motion authorization remained closed
  and every final command was zero.
- Repeated UI localization reset and runtime recovery could not repair this
  state because those operations restart the map-localization/Nav2 generation,
  not the container-level FAST-LIO process. A managed edge-service restart at
  00:24:40 cleared the corrupted odometry state. FAST-LIO returned to about
  700--800 effective matches per scan with 3--4 cm mean residual, and the API
  reported 10.0 Hz LiDAR plus 10.9 Hz odometry.
- After recovery the service was `active`, the selected candidate remained
  `map-8ddcf3f8c078`, the sole navigation supervisor was alive, and Nav2 plus
  the Unitree motion bridge were stopped. The recovery did not submit a patrol
  or command motion. One bounded development-capture patrol was enabled for the
  next field attempt; physical mount alignment and route execution remain to be
  checked by that attempt.

## 2026-08-08 latest-map selection and stale-runtime correction

- New recording `20260808T134839Z-53660034` completed cloud GLIM as
  `map-5b1f56fe3772`: 6,064 displayed points, a 20.082 m generation-3 route and
  74 control points. The immutable artifacts and candidate were already on the
  robot, but the persistent Nav2 process was still running previous
  `map-6a79b7795da6`.
- The UI treated “candidate files are published” as “the running Nav2 process
  uses this candidate” and exposed Start Patrol. The robot correctly rejected
  the request with `REQUESTED_MAP_VERSION_IS_NOT_ACTIVE`; this was a workflow
  state defect, not a GLIM, localization or path-planning failure.
- A live non-motion recovery stopped the old runtime and started the selected
  candidate. The receipt then showed candidate, runtime and localization all
  on `map-5b1f56fe3772`, `READY`, `TRACKING`, localization usable, zero final
  velocity and motion authorization closed. No patrol command was submitted by
  Codex.
- The Mac catalog now returns maps by `created_at` descending, displays local
  time, marks the newest map and stores an optional operator label separately
  from the immutable `map-*` identity. The browser provides inline rename,
  save and cancel controls.
- The patrol guide now compares the selected candidate with the map actually
  reported by the running runtime/localizer. On a mismatch it displays
  `运行旧地图`, hides Start Patrol and exposes `切换到所选地图并重启 Nav2`.
  Offline evidence is 55 passing tests, Python and JavaScript compilation, UI
  contract checks, and a live browser receipt for newest-first order, local
  timestamps, rename controls and an enabled patrol action only after all
  three map identities matched.

## 2026-08-09 navigation orchestration correction (deployed; patrol acceptance pending)

- The latest `map-8ddcf3f8c078` evidence changes the diagnosis from a generic
  obstacle problem to an orchestration defect. MPPI followed the route for
  48.6 seconds before a 0.509 ms future-TF lookup aborted FollowPath. The
  forward costmap and collision evidence were clear, but the deployed manager
  treated every non-success as a detour request.
- The deployed detour concatenated its local A* path with the complete
  remaining recorded route. Although it selected rejoin index 131, RPP kept
  ownership through route index 214. Its final 2.5 seconds still commanded
  0.24 m/s while net movement was only 0.051 m; this is actuation/progress
  evidence, not proof of a blocked route.
- The local branch now classifies independent localization, route-obstruction,
  controller and actuation evidence. A clear abort receives at most the
  configured MPPI suffix retries. Only consecutive occupied samples on the
  already footprint-inflated recorded route authorize A* and RPP. RPP receives
  only current pose to rejoin anchor; success explicitly returns the unfinished
  route suffix to MPPI. A detour failure cannot recursively create another
  detour.
- The misleading `blockedDecisionS` parameter is migrated to V2
  `progressTimeoutS`. It controls only Nav2's lack-of-progress observation
  window and defaults to 5.0 seconds. Humble's `PoseProgressChecker` now treats
  either 0.15 m translation or 0.15 rad rotation as progress, so RPP may align
  its heading without being killed by a translation-only watchdog. Obstruction cost/sample thresholds,
  transient MPPI retry count, detour attempt limit and independent 0.40 m/s
  detour speed now have explicit versioned parameters and workstation controls.
- Product-owned ROS runtime code now lives under
  `modules/navigation/ros/go2_nav2_runtime`; the commissioned Unitree receiver
  source lives under `modules/device_io/ros`. The robot image no longer needs
  build-time navigation overlays. Frozen sources remain unchanged as capability
  provenance.
- Navigation-focused offline tests cover the historical incident, clear-route
  retry, verified obstruction, RPP-to-MPPI handoff, actuation stall and legacy
  profile migration. The complete repository passes 69 tests, Python and
  JavaScript syntax compilation, generated-knowledge consistency, UI smoke
  and container contract checks.
- The deployed ARM64 correction is
  `gogoguard-robot-inspection:v2-edge-20260809-orchestration-r2`; its local OCI
  manifest-list digest is
  `sha256:9d693cfc3e78ebc37b22d1e16f5b176d4cfd5f9a9d5831d5bacaf505739253a1`
  and the robot-loaded ARM64 config digest is
  `sha256:e646d5541cffdb16e9f7d2987b74f89cceacc63985437118029153ccca841260`.
  All ten ROS packages compiled. An in-image import and Nav2 profile check
  reported `MPPI -> RESUME_MPPI_SUFFIX`, DetourPath 0.40 m/s, and the final
  Unitree binary contains the 0.60/0.20 m/s contract, and the incident recorder
  recognizes all three new recovery states.
- The first `orchestration-r1` robot start exposed a lifecycle defect before
  Nav2 launch: `StopMove` was called while the dog could still be outside Sport
  mode and returned `-1`, so the receiver correctly failed its zero-motion
  readiness check with exit 8. Release r2 prepares explicit StandUp and
  BalanceStand first, then requires StopMove and UDP bind before advertising
  readiness. A real r2 start passed; the motion bridge and Nav2 runtime stayed
  alive while the final command remained zero.
- Static post-deployment rates were approximately 10.06 Hz Livox point cloud,
  200 Hz IMU and 10.07 Hz FAST-LIO odometry. The migrated navigation profile
  was explicitly saved as V2 revision 2 with a 5.0-second progress window.
  Localization against `map-8ddcf3f8c078` did not converge because the operator
  confirmed the robot was not in that map's environment. No patrol goal was
  submitted. The wrong-map runtime was then stopped; both Nav2 and the motion
  bridge are inactive while the edge service and sensors remain online.

## Known field follow-ups

- Historical recordings, maps and navigation candidates remain truthfully
  bound to the original sensor `ARMCP1U0038561`. They are not automatically
  authorized for the replacement sensor. Either record a new map with
  `ARMCP6B0035634`, or implement and explicitly approve a geometry-equivalent
  sensor-replacement re-publication contract before reusing an old map.
- Collect a cold-boot receipt for the prepared Livox/NTP correction.
- Field-test localization and patrol on outdoor map `map-28acad0c6b0e`; the
  map and route are prepared but its Nav2 runtime has not yet been started.
- Correct startup localization ambiguity observed on `map-28acad0c6b0e`: while
  the robot was physically near the recorded origin, two resets locked to
  different poses roughly 4--6 m away and incorrectly reported high-confidence
  tracking. The 1.5 m route-start gate correctly denied motion, but startup
  candidate ranking needs a stronger route-start prior and ambiguity check.
- Preserve the exact Nav2/TF failure detail in the navigation status contract
  and display a specific sensor-clock error instead of the generic `FAULT`
  message. Correct the UI inconsistency that can display `定位可用` and
  `not_converged` at the same time.
- On the operator-selected matching map, run one
  clear-route patrol and one deliberately obstructed patrol. The clear route
  must remain on MPPI (or use only a bounded MPPI retry); the obstructed route
  must show costmap obstruction evidence, `DetourPath`, zero-forward heading
  alignment, effective forward gait, RPP success at the rejoin anchor and an
  explicit return to MPPI. After `BLOCKED` or `FAULT`, allow the 5-second
  post-trigger window to seal before reviewing the Mac copy.

## 2026-08-06 three-end architecture correction

- The production workflow is now explicitly robot + Mac workstation + cloud.
  The robot seals recordings and runs realtime algorithms; Mac owns UI,
  history, transfer, cloud orchestration and releases; cloud only runs GLIM.
- Edge Agent export and import contracts are implemented. Recording transfer is
  manifest-whitelisted, HTTP Range resumable and SHA-256 verified. Imported
  navigation maps are immutable and must identify as cloud GLIM artifacts.
- The Mac workstation persists recordings, map jobs and their relationship. It
  reports robot-to-Mac, Mac-to-cloud, GLIM and cloud-to-Mac stages with byte and
  transfer-rate fields. A failed map job can retry from existing partial data.
- The browser can select historical recordings/maps, view the 2D overview and
  rotate/zoom the 3D point cloud, then deploy a selected completed map to the
  robot before navigation preparation.
- A separate lightweight workstation Dockerfile/Compose definition and a
  Mac-to-robot image staging script are present. The robot default map worker
  is now `none`; it no longer needs GitHub, compilation, or a cloud SSH mount.
- The commissioned Mac runs the workstation natively so wired-LAN and cloud
  traffic use the Mac network directly; the 51 MB ARM64 workstation container
  remains a portable delivery form. Docker Compose 5.4.0 is installed.
- The Mac Ed25519 public key is now authorized on the Alibaba Cloud worker, and
  non-interactive SSH verified `/opt/go2/bin/gogoguard-map-job` is executable.
- This new flow has passed local Python compilation, UI contract tests and
  25 unit/integration tests, transfer/import integration, browser layout checks
  and a 48.3 MB compressed-layer ARM64 workstation image build. The native Mac
  workstation is running at port 8080 and correctly reports the powered-off
  robot as offline.
- The updated ARM64 robot image is built locally as
  `gogoguard-robot-inspection:v2-edge-20260806`, image ID
  `sha256:8ddb8f2ff3d78f4882ae613c4e4b706dc82434dcad234f51a5b4ac8e2fd88853`.
  Its imports, ROS entrypoint and edge-only map-worker contract passed inside
  the image. A checksummed 1.1 GiB export of the 5.03 GB virtual image is
  prepared under `release-cache/edge-20260806` and is intentionally outside
  Git. Its exported archive SHA-256 is
  `dd3a7a7d243ddf1a6c98c68a297e8f69a46b9aefc91b2c24f4e2644140266df7`.
- The Edge Agent and Mac workflow are deployed and have completed historical
  and short-recording GLIM round trips. The cloud progress wrapper still needs
  an explicit deployment receipt. Dynamic field acceptance is still pending.

## Next experiment

Record one intentional open-area mapping loop, review its GLIM artifacts on the
Mac, select and deploy that map to the robot, then verify fixed-map localization
before starting the Nav2 patrol experiment. Existing indoor/stationary maps are
engineering evidence and should not be treated as the field navigation map.

## 2026-08-07 delivery-closure refactor (deployed; dynamic acceptance pending)

- Today's field traces changed the next objective from incremental research to
  a complete operator loop for delivery. The selected real candidate remains
  `map-e9bc37247129` (713,497 points, 82.966 m, previously 285 source
  waypoints).
- Navigation process ownership moved out of the HTTP/UI process. One
  `gogoguard-navigation-supervisor` now owns one Unitree receiver and one
  localization/Nav2/MPPI generation. A filesystem lock rejects a second
  supervisor, UI shutdown no longer stops Nav2, and long start/stop/recovery
  actions return a durable operation id with accepted/running/complete/failed
  state instead of leaving the operator to infer whether a timed-out click ran.
- Candidate generation 3 forces existing map candidates to regenerate. The
  default delivery profile is 0.60 m/s forward, 0.40 rad/s turn, 0.20 m/s
  lateral authority, +/-0.30 m SlowZone, 85 percent slowdown, and a 2.5 second
  blocked decision target. These values passed configuration and offline
  runtime validation but require tomorrow's field receipt.
- Collision Monitor is now the only point-cloud obstacle safety owner. The
  final Unitree command gate retains explicit runtime authorization, the hard
  command watchdog, finite/range checks and absolute hardware caps; its
  duplicate localization and obstacle vetoes are disabled by composition.
- Patrol runtime now reports monotonic route index/percentage/remaining points.
  A short localization dropout enters `HOLDING` without destroying the Nav2
  goal. A sustained dropout cancels, waits for stable recovery, clears the
  local costmap and resumes from the unfinished route suffix rather than
  resending the full route or an empty path. Controller aborts after the
  blocked interval are reported as `BLOCKED` with a specific operator action.
- When the original path cannot progress, the delivery candidate performs a
  bounded A* search over Nav2's current footprint-inflated rolling costmap,
  chooses a short detour to a future route anchor, and gives the combined
  detour/unfinished suffix back to MPPI. Search is compute-bounded and limited
  to two attempts; no path becomes `BLOCKED` instead of minutes of silent
  waiting. Pure-grid wall/bypass/no-path tests pass; field acceptance is still
  pending.
- Navigation parameters are stored on the robot as validated, versioned JSON.
  The workstation Parameter Center exposes the common speed, avoidance,
  localization and rejoin controls, blocks half-applied changes while a runtime
  is active, and supports one-click rollback.
- The Diagnostics Center aggregates runtime, localization, obstacle/final
  command chain and recent log highlights. It is intended to replace ad-hoc
  per-incident grep as the first diagnostic step.
- The patrol UI is now a three-step next-action guide. The old misleading
  “使用当前地图与录制路线” button is gone: publication appears only when the
  selected Mac map differs from the robot candidate; otherwise the next action
  is directly “启动定位与 Nav2”. Every action includes operator guidance,
  operation progress and an automatic clean/recover entry for `FAULT` or
  `BLOCKED`.
- Offline evidence: 40 Python unit/integration tests passed; a real browser
  completed demo record/seal/map, parameter save/rollback, layered diagnostics,
  action visibility and 390 px responsive checks without console errors or
  horizontal overflow. The final Linux/ARM64 robot image compiled all ten ROS 2
  packages and passed in-image checks for the supervisor, patrol runtime,
  bounded detour module, Nav2 profile and parameter injection. It is tagged
  `gogoguard-robot-inspection:v2-edge-20260807-delivery`, local manifest-list ID
  `sha256:74f6860705865bd6b01b0bbeb1a4f5e8911cd476ca76b1ed6d837bb577230912`
  (1,146,800,740 bytes). The portable workstation image
  `gogoguard-field-workstation:20260807-delivery` also built and served its UI
  and offline robot-status contract. A checksummed 1.1 GiB robot release is
  prepared outside Git at `release-cache/edge-20260807-delivery`; archive
  SHA-256 is
  `de6929abd9807d25231b549a5b807114e91986e5e89021595904b52a39d9475c`.
  The robot image is now deployed; the workstation image remains a portable
  build while the commissioned Mac runs the same source natively.
- Deployment receipt: the archive passed SHA-256 again on the robot, Docker
  loaded config digest
  `sha256:7c431e41dadf51c37a60f282fdbcd8a9fa06fe723de4ac879197c1407be0e59d`,
  and the managed r4-to-delivery restart completed normally after the stable
  NTP gate. The service is `enabled` and `active`; LiDAR was 10.9 Hz and
  odometry 9.9 Hz. Exactly one navigation supervisor was present, while Nav2
  and the Unitree motion bridge remained stopped, so no motion was commanded.
- The six historical Mac GLIM jobs were copied into the active repository's
  ignored runtime-data root with absolute paths rebased. The selected
  `map-e9bc37247129` artifact hashes remained unchanged and its generation-3
  candidate was published to the robot at that time: 713,497 points, 82.966 m
  and 285 waypoints.
- Workspace-level `AGENTS.md` now routes new Codex tasks only to this field
  repository. The superseded V2 worktree and original Go2 repository remain
  present for rollback/provenance but are explicitly not current behavior or
  deployment sources.

## Next experiment (supersedes the older paragraph above)

Use the guided UI with the currently selected `map-71b045489e8a` to verify in
order: one clean localization/Nav2 start without timeout or UDP conflict,
localization convergence, first motion latency, 0.60 m/s clear-route travel,
passage through a normal corridor, specific `BLOCKED` behavior, pause/restart,
and the corrected obstacle detour returning to the recorded route. If a run
blocks or faults, wait for the evidence bundle to seal and inspect its point
cloud, local costmap, planner search, commands and camera window before changing
parameters. Record each result before calling the release field-accepted.

## 2026-08-09 planar-costmap and control-release correction (deployed; field verified)

- The latest matching-map patrol did not fail because the route was proven
  blocked. MPPI first commanded about 0.409 m/s and then reported
  `Optimizer fail to compute path`; the bounded detour subsequently reported
  `Failed to make progress`. At the same time, the 75 x 75 local costmap held
  1,034 lethal and 898 inscribed cells, with 535 of 625 cells inside the
  one-metre robot neighbourhood hard-blocked. This is inconsistent with the
  observed open space.
- Live TF showed the fixed-map result `map -> base_link` was near the intended
  origin while the 3D `odom -> base_link` translation carried approximately
  -2.99 m Z and a tilted frame. The rolling VoxelLayer used `odom` and a
  -0.60 m lower raytrace bound, so its LiDAR sensor origin fell outside its own
  vertical volume and stale obstacle cells accumulated. Increasing clearances
  or adding another detour retry would hide this coordinate-contract defect.
- The owned Nav2 profile now expresses the rolling local costmap in `map` and
  uses a planar `ObstacleLayer`. Startup clears the local costmap, waits for a
  newer frame and rejects patrol submission unless the new frame is fresh, is
  in `map`, contains the robot and leaves its cell traversable. Unhealthy
  costmap evidence is a dedicated failure class and cannot enter detour. A
  localization-loss recovery performs the same asynchronous clear/new-frame
  barrier before it resubmits the unfinished route.
- A route is considered obstructed only after the same condition persists
  across fresh costmap frames for `obstructionConfirmationS` (default 0.50 s).
  Actuation-stall classification now requires a complete observation window;
  a sub-second controller abort cannot be mislabeled as a motion stall.
- The workstation now exposes a persistent **停止巡检并释放遥控权** action whenever the
  runtime or Unitree motion bridge exists, including `FAULT` and `BLOCKED`.
  It invalidates an overlapping start/recovery, cancels patrol, revokes motion
  authorization, terminates the Nav2 runtime and SDK velocity receiver, sends
  `StopMove`, and returns a structured release receipt. This addresses the
  field symptom in which repeated SDK zero commands competed with the handheld
  remote after a failed patrol.
- Final receipts: 79 unit/integration tests pass, including initial and
  localization-resume costmap barriers,
  frame/robot-cell health, temporal obstruction, short-abort classification,
  recoverable idle costmap startup, paired-SDK StopMove, independent stop
  control lane and idempotent release receipt. Python compile,
  UI smoke, container contract validation, knowledge validation and diff checks
  pass. The Linux/ARM64 image built all ten ROS 2 packages and passed in-image
  Python imports, the strict Nav2 profile validator and installed
  planar-costmap/resume-runtime checks. The deployed image is
  `gogoguard-robot-inspection:v2-edge-20260809-planar-release-r4`, with local
  manifest-list digest
  `sha256:63aece679c19b1e34cf2243356da37deb081bbdb8823b2ad87e8edb298c5a390`,
  robot config digest
  `sha256:15e2b7c6951ea27ea4131bfbec4fe35d5302b9750ecafe700bf25665d6d785a1`
  and release archive SHA-256
  `83b4cc8e0ce2d9d0feff31a804fe3cfc64d53fd139ac838d044ae5b37918d7a2`.
- Static field receipt: localization reached `TRACKING` with approximately
  0.89 confidence; the planar `map` costmap stayed healthy and advanced from
  sequence 8 to 14 in three seconds; the robot cell cost was zero and the
  final command remained exactly zero before patrol authorization.
- Dynamic field receipt: `map-71b045489e8a` followed MPPI continuously from
  about 12% to 76.7% route progress with localization usable and a healthy
  costmap. It then reported `PATH_OBSTRUCTED` and revoked motion authority.
  The operator confirmed the room layout had changed: a table now created a
  real dead end where the recorded route previously passed. Bounded A* ran in
  2.12 ms but found no executable path back to the route, so stopping rather
  than submitting `DetourPath` was the correct outcome.
- Control-release receipt: runtime and SDK motion bridge both exited with code
  zero. The original one-shot receipt aborted because `ros2 run` mixed ROS and
  the paired Unitree native-library boundary. The release now invokes the
  installed SDK probe directly with the same library path as the receiver; a
  robot-side probe and the final workstation stop both returned
  `stopMoveConfirmed=true` and `remoteControlReleased=true`.

## 2026-08-09 map publication UI correction (Mac workstation)

- Recording `20260809T055524Z-236f3c74` sealed with 95 frames. Cloud GLIM job
  `map-799f6f04e11d` completed, and the resulting map plus the 9.805 m,
  37-waypoint route `route-799f6f04e11d-recorded` were published to the robot.
- The operator-facing publication error was false: the navigation status still
  contained an expired costmap age represented by Python as positive infinity.
  The HTTP server emitted bare `Infinity`, which browser `JSON.parse` correctly
  rejects even though Python's permissive decoder accepts it. Publication and
  artifact transfer had already succeeded.
- The shared JSON contract now converts every non-finite float (`NaN`, positive
  infinity and negative infinity) to JSON `null`; the HTTP boundary also uses
  `allow_nan=False` so a future unsanitized value fails during development
  instead of corrupting the browser response. The native workstation was
  restarted without changing the robot image or starting robot motion.
- Receipt: 80 tests pass. A strict JSON decoder accepts `/api/v1/navigation`,
  and browser verification shows `map-799f6f04e11d` as the current selected and
  published map with no parse error.

## 2026-08-09 cruise-speed contract correction (deployed; field measured)

- Three completed patrol traces on `map-799f6f04e11d` proved that the former
  speed control was misleading. At the original 0.60 m/s setting, final mean
  forward commands were 0.312--0.316 m/s with a 0.435 m/s maximum. Saving
  0.90 m/s changed the final mean to only 0.326 m/s and the maximum to 0.471
  m/s. The MPPI raw mean was 0.328 m/s and the post-collision final mean was
  0.326 m/s, so collision handling did not cause the observed low speed.
- Profile V2 also accepted commands the commissioned receiver can never
  execute: 0.90 m/s forward, 0.40 m/s lateral and 0.60 rad/s yaw versus the
  receiver's 0.60, 0.20 and 0.50 limits. Profile V3 aligns the UI, validation,
  MPPI, smoother and receiver limits and migrates impossible V1/V2 values.
- The correction treats 0.60 m/s as the clear-route cruise request,
  scales MPPI forward sampling to 0.30 m/s standard deviation at that request,
  raises the default batch from 700 to 1,000 within the existing compute
  budget, and raises PathFollow progress authority from 5.0 to 8.0. StopZone,
  SlowZone and collision thresholds are unchanged.
- Release `v2-edge-20260809-cruise-r1` was built for Linux/ARM64 from commit
  `a615ab9a32fa46dbb73a4f6fc30db6cce2de8054`. The 1,171,283,456-byte archive
  passed SHA-256 on Mac and robot with digest
  `d80fb512d63474e25a3782761f28b851bd526a07ed6e4f5a1c93d59075678142`.
  The robot loaded config digest
  `sha256:593a5336f355277e72b513e006a34a61e335b4dc94ae3468c48b52c92a016ab1`;
  its managed service is `enabled` and `active` with exactly one navigation
  supervisor. Static acceptance reported LiDAR 10.0 Hz, IMU 103.0 Hz and
  odometry 10.0 Hz. Nav2 and the Unitree motion bridge remained stopped, so no
  motion command was sent. The active V2 profile migrated to revision 3 with
  forward/detour/yaw/lateral limits 0.60/0.60/0.50/0.20 and MPPI batch 1,000.
  The first same-route speed receipt is recorded below.
- On `map-799f6f04e11d`, release `cruise-r1` completed the 9.805 m route in
  30.20 seconds versus 33.20 seconds on the immediately preceding release, a
  9.0% reduction. Odometry-integrated mean speed rose from 0.286 to 0.297 m/s.
  The final command's 90th percentile rose from 0.402 to 0.435 m/s and its
  maximum from 0.474 to 0.551 m/s, but mean final translational command remained
  0.342 m/s and 72.6% of nonzero commands were below 0.40 m/s. MPPI raw mean was
  0.349 m/s versus 0.342 m/s after collision handling, so the safety chain
  removed only about 2%; no detour or stop occurred and the route reached 100%.
  The release is measurably faster but does not yet deliver sustained 0.60 m/s
  travel. The remaining speed limit is primarily the MPPI/path-follow output,
  not the receiver or obstacle-stop policy.

## Next experiment (supersedes all older paragraphs)

Keep `v2-edge-20260809-cruise-r1` fixed and collect only the remaining acceptance
receipts: one obstacle that leaves a physically passable side corridor and
therefore exercises A* -> `DetourPath` -> MPPI rejoin; localization-loss route
resume; and handheld-remote movement immediately after a stop from an active
patrol and from `FAULT`. For speed work, explain why clear and nearly straight
segments still average only about 0.36--0.38 m/s before changing another
parameter; do not attribute the measured limit to collision handling. The
terminal dead-end stop and normal MPPI travel are already field verified; do
not weaken obstacle thresholds to make an actually sealed path appear passable.

## Next experiment (2026-08-10; supersedes all older paragraphs)

Freeze V6-r4 as the operator-accepted mobility baseline and integrate the
preserved realtime-dialogue branch without changing navigation ownership. The
next product slice is one combined, contract-driven checkpoint inspection:

1. merge and statically verify the independent `interaction` module with its
   default-disabled activation policy;
2. define explicit `mission`, `inspection`, `device_io`, `interaction` and
   platform boundaries, including capability declaration and unsupported
   hardware behavior;
3. prove one checkpoint sequence: navigation reaches the configured point,
   mission requests a true stop and receives its receipt, inspection captures
   the configured views, the platform stores/judges the evidence, and only a
   platform continue request can resume the remaining route; and
4. accept patrol and interaction first in separate zero-motion/static checks,
   then concurrently while retaining localization, command, media and thermal
   evidence.

Passable-obstacle/rejoin and a complete retained full-route trace remain useful
mobility repeatability receipts, but they are no longer blockers for freezing
the user-accepted V6 release baseline. Do not add another controller, safety
layer or direct interaction-to-motion path as a shortcut.

## 2026-08-09 realtime multimodal interaction (formal runtime offline; robot deployment pending)

- A disposable, ignored robot probe completed real BOYA mini 2 + Z1Pro + Go2
  speaker + LiveKit + Qwen-Omni loops before formalization. The final 185.83
  second run published 9,000 BOYA frames and 3,495 1080p video frames, received
  9,212 platform audio frames, handled 18 ordered playback-stop messages and
  reported zero speaker-buffer drops. Platform health ended idle with
  `currentError=null`, no reconnect, 18 new turns, first-audio latency P50
  0.712 seconds and P95 1.248 seconds. Process use was about 1.28 CPU cores,
  232 MiB RSS and 57.75 C maximum temperature. The run sent zero motion
  commands; motion authorization stayed false and the final command stayed
  zero. Those facts prove the disposable design and commissioned devices, not
  the new formal runtime.
- The formal `interaction` module now validates ephemeral `start_live` JWT
  identity, room, permissions and expiry; makes repeated commands idempotent;
  refreshes the in-memory reconnect token; redacts secrets from status/errors;
  implements half-duplex microphone mute and ordered reliable playback flush;
  and reports `gogoguard.interaction_session_status.v1` without importing
  navigation or cloud-model code.
- The `device_io` interaction boundary fixes the commissioned BOYA format at
  S24_3LE stereo 48 kHz with 4x gain, Z1Pro at 1920x1080 and requested 30 FPS,
  and Go2 output at 48 kHz mono 20 ms. A 120 ms prebuffer plus bounded 3 second
  queue exposes prebuffer, underflow, overflow and flush counters. An owned VUI
  helper applies and reads back the required 10/10 speaker volume at session
  start.
- The versioned robot persona is `小玖`, developed by
  `北京零零壹玖科技有限公司`, with an explicit rule that stale or absent video
  cannot support a current visual claim and that conversation has no motion
  authority. Media-live and conversation-wake states are separate. The first
  production wake contract accepts `小玖小玖` and the ASR homophone `小九小九`,
  acknowledges `我在`, sleeps after 30 seconds of inactivity and supports
  explicit end phrases. Platform-side ASR owns first-release wake detection;
  local offline KWS is a later enhancement.
- `gogoguard-interaction-edge` exposes only a local Unix JSON control socket
  for the existing authenticated GoGoGuard heartbeat adapter. It has no inbound
  public control port and no motion command surface. The formal service is
  image-present but defaults disabled; production requires local enablement and
  the per-device Unitree key. The image, repository or status never contains a
  GoGoGuard development secret, LiveKit signing secret, DashScope key or
  persisted LiveKit JWT. The runtime environment file is installed mode 0600.
- Offline interaction-focused evidence is 19 tests plus the unchanged project
  suite. Formal ARM64 image build, robot install, platform heartbeat bridge,
  小玖 persona deployment, visual-freshness refusal, ten-minute media run and
  concurrent patrol/resource acceptance remain pending. The current insecure
  IP WebSocket is allowed only by a robot-local development flag; production
  remains fail-closed to `wss` until GoGoGuard resolves its SNI path.

## 2026-08-09 realtime interaction standalone stage closed

- Product decision: stop realtime interaction as a standalone delivery. Do not
  schedule another interaction-only GoGoGuard response or robot run. Preserve
  `agent/realtime-dialogue-v1`, finish field acceptance of patrol first, then
  integrate from the latest patrol commit and request one combined patrol plus
  interaction acceptance. The complete handoff is
  `docs/stage-handoff-realtime-interaction-20260809.md`.
- GoGoGuard P1.5 evidence is accepted for later joint re-verification of the
  versioned Xiaojiu persona, identity answers, visual-context freshness/reset,
  desired-live/JWT refresh/stop flow, and participant-SID reconnect ownership.
  It does not close the wake requirement: the response contains no wake/sleep
  evidence or health fields, and its own identity tests answered ordinary
  speech without `小玖小玖`. It also describes heartbeat `robotId` as the only
  device recognition mechanism; that is identification, not authentication.
  Per-device authenticated and replay-resistant heartbeat plus a robot-4G
  production TLS/SNI receipt remain required.
- Local evidence at closure is 99 passing tests plus successful compilation,
  repository-knowledge validation, container-contract validation and diff
  checks. The Linux/ARM64 build reached the final Python dependency layer after
  the patrol/ROS native layers, then failed on a `files.pythonhosted.org` read
  timeout. No formal image was produced, no retry is required in this closed
  stage, and the formal runtime was never installed on the robot.
- Next phase must complete the combined capability set and heartbeat adapter,
  then build and regress patrol with interaction disabled before static and
  concurrent acceptance. Dialogue remains isolated from motion; any future
  patrol pause/resume or action request goes through the patrol safety owner and
requires an explicit success receipt.

## 2026-08-10 combined V6, realtime interaction and checkpoint foundation

- The operator accepted the latest V6 patrol for release. Its exact mobility
  source is frozen in `922b301`; no navigation/controller/safety parameter is
  changed by the checkpoint foundation below.
- Realtime interaction commit `3215515` was merged by `6e842d1`. The merged
  module preserves the already proven LiveKit video/audio, BOYA microphone,
  Go2 speaker, half-duplex interruption, reconnect, token validation, persona
  and data-channel status. It defaults disabled, imports no navigation
  implementation and exposes no motion command. This is source integration,
  not a new robot deployment or concurrent patrol receipt.
- The dog-side response to the GoGoGuard phase-2 alignment is
  `docs/GOGOGUARD_PHASE2_DOG_REPLY.md`. It deliberately distinguishes field
  evidence, offline implementation, hardware capability and product support.
  The platform can now define its wire API without assuming unavailable pause,
  gimbal, buffer or pose-stream behavior.
- Shared contracts now define a map/route-bound mission, ordered checkpoint
  views, a pause-correlated true-stop receipt, inspection-frame identity and
  mission status. The pure `mission` state machine uses monotonic route progress
  rather than a six-metre platform guess, requires a matching pause request and
  stable zero-motion evidence, resumes the retained route suffix, and permits
  offline continuation only when the downloaded mission preauthorizes it and
  evidence is durably buffered.
- The independent `inspection` module implements a bounded atomic JPEG/PNG
  queue with SHA-256, idempotent frame IDs, FIFO capacity enforcement, upload
  acknowledgement and tamper detection. The planned default is 2,000 frames / 4
  GiB, but capability remains false until a platform checkpoint adapter and
  robot storage/load acceptance exist.
- A commissioning-gated Z1Pro GCU codec/adapter lives in `device_io`. It matches
  the official V2.0.6 null, pitch and yaw packet CRC examples and parses the
  official response example with CRC validation. Official hardware limits are
  declared, but `gimbal.supported` remains false: the vendor's +/-0.01 degree
  figure is stabilization accuracy, not accepted repeat positioning, and the
  robot still needs a static pan/tilt/settle/repeatability test. The adapter
  never moves the dog.
- `GET /api/v1/capabilities` now serves the versioned fail-closed robot profile.
  It truthfully records that the existing interaction video and LiveKit data
  channel are available while the 10 Hz pose publisher is not yet integrated.
  The localization source is about 10 Hz and the existing navigation public
  status is about 5 Hz; the platform topic/serializer must reuse the existing
  LiveKit session after the wire contract is agreed.
- A separate `platform_edge` service now implements the documented P1.5
  five-second outbound heartbeat, sends read-only navigation/interaction/
  capability state, forwards only `start_live`, `stop_live` and
  `wake_transcript`, and persists only bounded command-ID hashes for
  deduplication. It rejects commands for another robot and every motion action;
  token, URL and room are never written to its ledger or status. The adapter
  supports an optional per-device bearer token, but the current platform test
  endpoint's robotId-only authentication remains unsuitable for production.
- No checkpoint adapter, gimbal motion, new container, robot install, runtime
  start or physical motion was performed in this foundation phase. The next
  implementation starts from the platform's interface agreement, then performs
  a dog-lying-down gimbal/static release check before any combined patrol.

## 2026-08-10 combined realtime release built; robot joint acceptance pending

- The release is intentionally derived from the field-accepted V6-r4 image ID
  `sha256:b8ca1b65ede57fa0f7ca2cfefa3c1490b905717aa3681d245128fa4e9081269c`.
  Its 32 rootfs layers are the exact prefix of all 44 combined-image layers.
  `Dockerfile.combined` performs no apt operation, so Ubuntu, ROS, Nav2, MPPI,
  localization and Unitree native binaries are not rebuilt or upgraded.
- The retained real-probe ARM64 wheelhouse contains exactly 29 pinned media
  distributions. Every wheel passed the tracked SHA-256 manifest before an
  offline install under `/opt/gogoguard/interaction-python`. Only the two
  realtime daemons receive this import path. Image checks prove the accepted
  system runtime still uses NumPy 1.21.5 while interaction uses NumPy 2.0.2.
- The release image is
  `gogoguard-robot-inspection:v2-edge-20260810-combined-live-r1`, Linux/ARM64,
  image ID
  `sha256:6027a7d17ff961da1772a308b074bb4b24568871c126d291b0f9b47dbd18f11c`
  and Docker size 1,388,559,726 bytes. In-image system/application/native-media
  imports, all three console entry points and `pip check` pass.
- The ignored immutable release directory is
  `release-cache/v2-edge-20260810-combined-live-r1/`. Its 1,388,612,096-byte
  image archive passed SHA-256 with
  `9e25dca2cbee5784cd81ccb60b0e65155231055094fc5545d7ef272d4fbce174`.
  The archive contains the installer, systemd unit, runtime files, clock gate,
  run wrapper and explicit combined-joint-test configuration helper.
- Installing a new release no longer overwrites an existing root-only robot
  environment. A disposable install test proves that commissioned Unitree and
  custom values are preserved, missing defaults are appended, file mode stays
  0600 and the service is not started. The explicit configuration helper
  refuses a missing Unitree key, enables secure HTTPS/WSS defaults when the key
  exists, preserves the key and still does not start the service.
- Final offline evidence at this checkpoint is 143 repository tests, 7/7
  in-image platform/Unix/interaction-flow tests, 19/19 in-image interaction and
  device-I/O tests, compilation, UI smoke, container contract, generated
  repository knowledge, shell/JSON checks and a clean diff check. The complete
  platform-facing handoff is
  `docs/GOGOGUARD_COMBINED_LIVEKIT_JOINT_TEST_HANDOFF.md`.
- The combined image has not been installed on the dog. Tomorrow's acceptance
  remains explicit: install with the service stopped; keep the dog lying down;
  configure interaction and heartbeat; prove static `start_live`/media/wake/
  interrupt/refresh/reconnect/`stop_live` with zero motion; only then run one
  accepted V6 patrol concurrently for ten minutes. Do not activate unfinished
  checkpoint, gimbal or pose-publisher capabilities during this acceptance.

## 2026-08-11 formal realtime runtime deployed; quality and wake UX open

- The interaction work remained isolated from the accepted mobility stack.
  The combined Dockerfile still extends the exact V6-r4 base digest; its 32
  rootfs layers are an exact prefix of the 44 r8 layers. No navigation,
  localization, route, Nav2, MPPI, collision or Unitree motion parameter was
  changed. All static tests ran with Nav2 and the motion receiver stopped, and
  UDP 5005 remained unbound.
- Real robot diagnostics found two independent native-runtime integration
  defects. Unitree WebRTC must initialize before AV/aiortc/LiveKit in this
  paired ARM64 environment; the previous order completed ICE but left the Go2
  DTLS/DataChannel in `connecting`. Separately, the globally sourced ROS
  `LD_LIBRARY_PATH` made the VUI helper load an incompatible ROS CycloneDDS and
  abort with `free(): invalid pointer`. Commit `0673849` centralizes the proven
  native import order and prepends `/opt/gogoguard/deps/lib` only for the VUI
  subprocess. The helper now retries bounded transient failures and verifies
  the requested volume instead of trusting its set return code.
- The platform-frozen temporary address profile now posts the complete
  five-second heartbeat to
  `http://39.96.37.187/api/v1/robot/heartbeat` and permits only the
  command-supplied `ws://` LiveKit URL. The current service command line proves
  the HTTP IP endpoint and explicit development allow flag are active. Secure
  HTTPS/WSS remains the production default after ICP recovery; current
  robotId-only platform identification is not production authentication.
- Release
  `gogoguard-robot-inspection:v2-edge-20260811-combined-live-r8` is deployed.
  Its Linux/ARM64 manifest ID is
  `sha256:856c5d19e94901d55ed1fcf9d1677a3125a3530359cfc9862cea836115b11590`,
  robot config digest is
  `sha256:3637efd4d79ccfeaee715bbf8e17e100f1992cfdb539c6df6833e20b6993f62e`,
  Docker size is 1,389,379,684 bytes and its 1,389,431,296-byte release archive
  passed SHA-256
  `f3483d61d3aa8b0d82f29574358adf4ae6ef09f424ce0ef14b49a0ce6fd3741c`.
  The active container has zero restarts.
- A platform `start_live` produced a real `robot:LLYJ0001` session in
  `patrol-test-001`. The formal status reached `audioPublished=true`,
  `videoPublished=true`, `audioSubscribed=true` and `dataConnected=true`; the
  operator physically heard agent audio. This closes formal static audio
  deployment and proves BOYA/Z1Pro/Go2/LiveKit composition at the dog. The
  platform still needs to return its authoritative participant/video-track
  receipt before server-side video acceptance is closed.
- The operator compared Go2 speaker volume 7 and 10. Seven was materially
  clearer but too quiet; the operator explicitly retained 10 as the field
  default. Full-volume output again sounded choppy and different replies had
  inconsistent loudness even though the dog-side speaker queue recorded zero
  drops and zero underflows. Do not hide this with a larger queue. The next
  diagnosis needs one pre-LiveKit TTS WAV plus platform RTP
  loss/jitter/concealment statistics, then a three-stage comparison of source
  TTS, dog-decoded PCM and physical Go2 output.
- The new timing evidence measured three accepted transcripts. The most recent
  transcript-to-first-active-audio time was 1,646 ms and the maximum was 2,284
  ms. Across the first 5,806 decoded agent frames only two inter-frame gaps
  exceeded 40 ms, maximum 80 ms; the speaker queue still had zero drops and
  underflows. A separate 30-packet 4G ping had zero loss but 33.493/90.954/
  331.087 ms min/mean/max RTT and 67.326 ms mdev. TCP 80 was established, TCP
  7881 remained `SYN-SENT`, UDP 7882 returned no STUN response and UDP 3478
  returned a valid matching STUN response. Platform timing and candidate-pair
  evidence are required before assigning the complete latency to network or
  model/TTS.
- Audible wake acknowledgement is not accepted. A first wake DataChannel event
  reached the dog and incremented the wake sequence, but the operator later
  said `小九小九` without a new DataChannel event reaching the dog; the gate
  remained sleeping. The platform must correlate ASR, wake-gate and DataChannel
  logs and must publish an audible `我在` when wake succeeds. A silent internal
  state change is not an acceptable operator experience.
- The static deployment regression is 150 repository tests plus Python
  compilation, repository-knowledge validation, container-contract validation,
  in-image NumPy/native import checks, exact V6 layer-prefix comparison and a
  clean diff check. Concurrent patrol plus interaction, ten-minute resource
  acceptance, token refresh, reconnect and stop-live acceptance remain open.
  The platform-facing evidence request is
  `/Users/mac/Desktop/GOGOGUARD_狗端实时对话联调回执-20260811.md` and contains no
  JWT, device key or password.

## 2026-08-11 external map-asset delivery gap recorded

- The original GoGoGuard phase-2 request expects a gravity-aligned PCD for 3D
  rendering and checkpoint placement, plus a version-bound route and poses in
  the same `map` frame. The previous dog-side response defined those formats
  but did not define how the SaaS receives the files.
- Current publication is only Mac -> robot. The Mac uploads immutable GLIM
  artifacts plus the editable navigation workspace; the robot then derives a
  binary PCD v0.7 (`float32 x/y/z/intensity`, metres, gravity-aligned z-up), a
  planar `go2.route.v1` (`x/y/yaw`, yaw in radians), the Nav2 keepout mask and a
  generation-5 candidate manifest. No current API publishes that candidate to
  the GoGoGuard SaaS, and large PCDs must not be sent through heartbeat or
  LiveKit.
- The preferred ownership is Mac field workstation -> SaaS: one selected map
  release produces a portable checksummed bundle containing `map.pcd`,
  `route.json`, editable allowed-area geometry, an optional preview and a
  relative-path manifest. The same `mapVersion`, workspace revision/hash and
  `routeId` bind platform checkpoints, live poses and robot execution. Map
  versions remain immutable; a changed blue/green workspace creates a new
  route revision on the same map.
- Before implementation, ask GoGoGuard to freeze the asset API: init/presigned
  or resumable upload, size limits, authentication, SHA-256 verification,
  completion/activation receipt, replacement/retirement semantics and which
  optional assets it wants. Until that contract exists, the operator has no
  legitimate "publish map to platform" action; manual file copying is only a
  temporary exchange, not product delivery.

## 2026-08-11 platform patrol/checkpoint composition deployed statically

- The V6 mobility envelope and its accepted controller/localization/safety
  parameters are unchanged. The platform adapter now allow-lists
  `start_patrol` and `stop_patrol`, but `start_patrol` can only activate the
  map and route already selected and published by the Mac workstation. Optional
  `mapVersion` and `routeId` mismatches fail before motion; arbitrary goals,
  paths, poses and velocities remain rejected.
- A platform start is a long supervisor operation. It starts the selected
  runtime if necessary, waits for actual localization usability and a healthy
  fresh costmap, then submits the existing Nav2 patrol. Stop still tears down
  the complete navigation/Unitree ownership chain and releases the handheld
  remote-control boundary.
- The five-second heartbeat has a stable full shape. Unknown pose, twist,
  battery and charging values are JSON `null`, never synthetic zero or a stale
  remembered percentage. A new Unitree `/lf/lowstate` observer supplies SOC,
  voltage and source sampling time; charging remains `null` until a docked
  physical receipt defines a trustworthy Unitree predicate.
- While the existing LiveKit session is active, `/localization/pose` is also
  published at up to 10 Hz as `gogoguard.robot_pose.v1`, bound to the same
  `mapVersion`, `routeId` and `map` frame. Delivery is unreliable/latest-only:
  slow networks drop an old position rather than queueing behind audio/video.
  Invalid or unusable localization is not published.
- Navigation candidate generation 6 produces a portable platform bundle with
  `map.pcd`, editable `route.json`, the exact interpolated
  `execution-route.json` used by the robot, `navigation-workspace.json`,
  allowed-area metadata, optional preview, relative-path manifest and SHA-256
  receipt. The Mac page exposes a direct download action. Platform checkpoints
  must use `execution-route.json.routeProgressIndex`; the editor route's sparse
  control-point number is not the execution index.
- A version-bound MissionPlan may contain ordered checkpoint IDs and execution
  route indexes. At each index the runtime cancels FollowPath, revokes motion
  authorization, proves both the command chain and Unitree measured motion are
  below the frozen thresholds continuously for at least 0.5 seconds, then runs
  the official Nav2 Spin behavior for exactly 2*pi radians through the existing
  costmap/collision/smoother/authorization chain. Success re-enters the existing
  fresh-costmap suffix-resume barrier; failure remains stopped/blocked and never
  silently skips the checkpoint.
- The completed static release receipt is 166 repository tests, Python
  compilation, UI smoke, generated-knowledge validation and container-contract
  validation. The final Linux/ARM64 image is
  `gogoguard-robot-inspection:v2-edge-20260811-platform-patrol-r4`, manifest-list
  ID `sha256:c8582c48eaec002c76e8740db96540083019825894ae9a1d08588fd7a962e3bf`,
  config ID `sha256:1c02faf94ee7e7b37b57932f871f0bba86c7f1bc3499a8b644f46aec8afd843c`,
  1,389,551,636 bytes and 44 layers. Its first 32 layers exactly match the
  accepted V6-r4 base. In-image checks proved that the installed ROS Python
  runtime, launch and YAML equal the owned source, the Nav2 `Spin` interface and
  `nav2_behaviors behavior_server` exist, the profile validates, launch
  arguments render and the new platform/pose/battery entrypoints import.
- Two real platform bundles were also generated and passed ZIP integrity:
  `map-6855ba54ae11` / `route-6855ba54ae11-workspace-r4`, 2,442 execution
  points, SHA-256
  `767920acadde6b0c53c3fe79e5ccae05dcd2ae75b48438145a2d31345d125cb8`;
  and `map-799f6f04e11d` / `route-799f6f04e11d-workspace-r6`, 72 execution
  points, SHA-256
  `ce84be14d56e0e801b9c1d93fdb040397fef11677f323c118d54b466bd39424e`.
  These are local handoff artifacts; no SaaS upload/activation API exists yet.
- Robot static receipt: the release archive passed SHA-256 on both Mac and
  robot, the service is enabled/active with zero restarts, the platform
  heartbeat is online, audio/video/subscription/DataChannel are all live and
  Livox was measured at about 10.07 Hz. The first battery deployment exposed
  that navigation domain 42 cannot see Unitree's bare-DDS domain 0; r4 isolates
  only the read-only observer on domain 0 and matches Unitree's RELIABLE
  depth-10 LowState subscription. It then produced a real 24 percent, 27.55 V
  sample. No Nav2, patrol runtime, Unitree receiver or physical motion was
  started during deployment.

## 2026-08-11 threefold downlink playback gain deployed statically

- Field listening established that Go2 hardware volume 10 alone was still too
  quiet. The `device_io` speaker boundary now applies a fixed 3x gain to the
  decoded 48 kHz mono s16 agent PCM immediately before the existing bounded
  speaker buffer. A per-frame peak limiter retains two percent headroom and
  reduces gain only when the requested multiplication would clip; the Unitree
  VUI hardware volume remains fixed at 10 with the existing set/read-back.
- The media receipt now records `outputGain`, last/maximum source RMS and
  last/maximum output RMS so the next field round can distinguish a quiet
  platform source from dog-side playback. This is an owned `device_io` change;
  it does not alter LiveKit signalling, platform contracts, navigation or
  motion authority.
- The release passed 167 repository tests, Python compilation,
  container-contract validation and generated-knowledge validation. The final
  Linux/ARM64 image is
  `gogoguard-robot-inspection:v2-edge-20260811-platform-patrol-r5`, manifest-list
  ID `sha256:56e79f9c253c1faad055b1bf3d54e3ae60cb1628527e91581e891c41dc935364`,
  robot config ID
  `sha256:adbac4e22f9e523be3c37e9dbe0eb0eb94e89f661bfa7ac0f06438557faf45f4`,
  1,389,551,074 bytes and 44 layers. Its first 32 layers exactly match the
  accepted V6-r4 base; an in-image sample proved s16 input 1000 becomes 3000.
- The checksummed offline release was staged and installed on LLYJ0001. The
  service is active with zero restarts, platform and LiveKit are online, all
  four media booleans are true, and `outputGain=3.0` is live. One observed
  platform-audio window measured source/output maximum RMS 6972/11310 with
  6,445 decoded frames, three frame gaps above 40 ms (maximum 79.2 ms), zero
  speaker drops and zero underflows. The dog remained stopped: Nav2, patrol and
  the Unitree motion bridge were absent and UDP 5005 was unbound. Operator
  listening acceptance of the new loudness remains pending.

## 2026-08-11 recording-as-configuration checkpoint closure implemented locally

- Primary ownership remains the Mac `field_workstation`. During the human-
  driven recording it now records an inspection checkpoint, the current
  odometry pose, the requested Z1Pro pan/tilt/roll, optional 360-degree body
  spin, note and a local JPEG configuration sample. The sealed recording keeps
  these files and hashes; deleting/re-recording a point does not renumber other
  checkpoint identities.
- Route candidate generation 7 maps each recording-time checkpoint through the
  GLIM optimized trajectory onto the exact interpolated execution route,
  preserves body-yaw intent relative to the route tangent and flags a point for
  operator review when it is more than one metre from the execution route. The
  platform ZIP now contains `checkpoints.json` and its local JPEG samples in
  addition to PCD, editable/execution routes and green allowed-area assets.
- The Mac upload adapter implements the frozen asset init/chunk/status/complete
  protocol with SHA-256 verification, resumable missing-chunk transfer and an
  environment-only `GOGOGUARD_DEVICE_TOKEN`. It deliberately stops after
  platform verification and never calls activation. The UI exposes checkpoint
  audit, download and upload progress only on the workstation-owned surface.
- The checkpoint runtime now preserves the remaining route while it proves a
  true stop, turns the body to the recorded absolute yaw, moves the Z1Pro,
  handles announcement completion, publishes `capture_ready`, waits for the
  platform verdict and accepts only correlated, unexpired continue/retake/skip
  controls. Retakes are bounded, duplicate controls/verdicts are idempotent and
  timeout resumes through the same fresh-costmap suffix barrier.
- Platform edge emits the frozen reliable event phases and verdict ACK, while
  the existing latest-only pose channel adds current mission/checkpoint/camera/
  spin context at up to 10 Hz only when localization is usable. Heartbeat and
  LiveKit DataChannel share the same inbox schemas. Tokens are not persisted or
  logged.
- The accepted V6 mobility implementation and all speed, acceleration,
  localization, MPPI, costmap and receiver limits are unchanged. This slice
  composes around navigation; it does not replace or retune the successful
  patrol path.
- Offline receipt: 182 repository unit/integration tests pass, Python
  compilation passes, JavaScript syntax and UI smoke pass, generated repository
  knowledge is current, container contract validation passes and `git diff
  --check` is clean. A browser drove the local demo through Z1Pro adjustment,
  checkpoint/sample recording and map sealing; that run exposed and closed a
  missing demo navigation-workspace proxy and API-to-HTML fallback. Formal
  route/checkpoint binding and upload are covered with trusted cloud-GLIM
  fixtures because demo maps are intentionally rejected as production assets.
- This is a **local implementation receipt only**. No ARM64 image was built, no
  release was installed on the robot, no physical motion was started and no
  platform asset/checkpoint mission was sent. Z1Pro motion, real JPEG capture,
  one end-to-end checkpoint and suffix completion still require field
  acceptance after an explicitly authorized deployment.

## Current next-task handoff (2026-08-11; supersedes older next experiments)

Start only from `gogoguard_robot_inspection_v2_field` on
`agent/three-end-field-workstation`. Treat `922b301` as the frozen V6 mobility
source, `6e842d1` as the historical interaction merge and platform-patrol-r4
as the patrol/checkpoint baseline. Treat platform-patrol-r5 with the digests
above as the latest verified robot deployment. The old
realtime worktrees are evidence/provenance only and must not be deployed.

Platform-patrol-r5 remains the installed and statically accepted robot release;
the recording-as-configuration closure above is newer local code and is not
deployed. Its next step is an explicitly authorized ARM64 build/install while
the dog is prone, followed by a no-motion service/heartbeat/camera/gimbal probe.
Physical acceptance must begin only when the operator has stood the dog up and
explicitly starts the experiment. First use the platform to start/stop one
selected route without checkpoints and verify pose/progress and remote release.
Then record/publish one conservatively placed checkpoint and verify true stop,
recorded body/camera pose, announcement, platform capture/verdict, bounded
retake or timeout, retained-suffix continuation and final route completion.

First have the operator listen to a normal multi-sentence platform response on
r5. Compare the new source/output RMS and speaker-buffer counters with the
physical result; do not increase hardware volume beyond 10. If loudness is
still inconsistent, freeze platform TTS loudness and correlate one round's
pre-LiveKit PCM, LiveKit loss/jitter/NACK/concealment and dog decoded timing.

In parallel, GoGoGuard must freeze the SaaS asset upload/receipt/activation API
for the already-produced portable bundle and accept the MissionPlan/pose wire
shapes, or return its adapter mapping. Audio remains a joint evidence task:
correlate one round's pre-LiveKit TTS PCM, LiveKit loss/jitter/NACK/concealment,
dog decoded-frame timing and physical result. Keep UDP media as the preferred
realtime path; the server already proved the selected pair is UDP 7882 direct,
while TURN remains fallback. Do not change the frozen V6 mobility parameters or
give interaction arbitrary motion authority while closing these receipts.

The last recorded robot state had platform-patrol-r5 active with zero restarts,
platform and interaction online, peak-limited 3x playback gain active and real
20 percent battery. Nav2, patrol
runtime and the Unitree motion bridge were stopped and UDP 5005 was unbound.
This is a receipt, not a promise of current live state: a new task must re-read
the robot before any deployment, and must obtain explicit authorization before
starting localization, Nav2 or physical motion.
