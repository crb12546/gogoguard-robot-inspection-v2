# Current project state

Updated: 2026-08-07

## Verified external facts

- Robot: Unitree Go2 expansion dock, 100 TOPS class, Jetson Orin NX 16 GB.
- Robot host: JetPack 5 / Ubuntu 20.04 / L4T R35.3.1; Docker was previously
  verified on the robot.
- Target robot userspace: one Linux ARM64 Ubuntu 22.04 / ROS 2 Humble container.
- Sensor: Livox MID-360. Camera: Z1Pro.
- Cloud mapping: existing Alibaba Cloud Ubuntu 24.04 x86_64 / ROS 2 Jazzy GLIM
  worker. A stationary native PointCloud2 + Imu smoke dataset previously
  completed GLIM and produced 12 review artifacts. Dynamic production mapping
  is not verified.
- SaaS: existing GoGoGuard platform is retained. V2 implements only its robot
  integration adapter.

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
- No V2 code has controlled robot motion.
- First release target: live device status, 2D trajectory, 3D point-cloud
  preview, start/stop recording, sealed RecordingBundle, cloud map job, and
  visible 2D/3D result.

## Current deployed release

- Active V2 implementation commit: `7ac9b3874fc3a5d52bbec62e1190ad8af30155d4`.
- Robot image config digest:
  `sha256:38ab20106377c491b8b07740419c13755a4cc08e3deeed4b2736027245b48357`
- Robot service: `enabled`, `active`, live status at port 8080
- Active image: `gogoguard-robot-inspection:v2-edge-20260807-r4`; robot runtime no
  longer mounts cloud SSH material and its map worker is `none`.
- The r4 release archive SHA-256 is
  `99636f88dfc2d573a62798a74ba7302b3b7c49162fff6f9602f0f0b3ddc586cc`.
  The corrected host service uses the commissioned robot's Docker-compatible
  `docker stop -t 15` form, treats Docker's normal stop exit 143 as success,
  starts only after the NTP gate, and remains enabled across reboots.
  The final restart receipt recorded `gogoguard-edge.service: Succeeded`, then
  passed the stable-NTP gate and returned to `active` without a failure result.
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

## Known field follow-ups

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

## 2026-08-07 delivery-closure refactor (not yet robot-deployed)

- Today's field traces changed the next objective from incremental research to
  a complete operator loop for delivery. The selected real candidate remains
  `map-e9bc37247129` (713,497 points, 82.966 m, previously 285 source
  waypoints). Nothing in this section is a new robot receipt until tomorrow's
  release is installed and exercised.
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
  `gogoguard-robot-inspection:v2-edge-20260807-delivery`, image ID
  `sha256:74f6860705865bd6b01b0bbeb1a4f5e8911cd476ca76b1ed6d837bb577230912`
  (1,146,800,740 bytes). The portable workstation image
  `gogoguard-field-workstation:20260807-delivery` also built and served its UI
  and offline robot-status contract. A checksummed 1.1 GiB robot release is
  prepared outside Git at `release-cache/edge-20260807-delivery`; archive
  SHA-256 is
  `de6929abd9807d25231b549a5b807114e91986e5e89021595904b52a39d9475c`.
  Neither image is robot-deployed yet.
- Workspace-level `AGENTS.md` now routes new Codex tasks only to this field
  repository. The superseded V2 worktree and original Go2 repository remain
  present for rollback/provenance but are explicitly not current behavior or
  deployment sources.

## Next experiment (supersedes the older paragraph above)

Prepare and stage the verified ARM64 release from this branch, install it after
the robot and Livox are connected, then use the guided UI with
`map-e9bc37247129` to
verify in order: unique process generation, localization convergence, first
motion latency, 0.60 m/s clear-route travel, passage through a normal corridor,
specific `BLOCKED` behavior, pause/restart, and route-suffix recovery after one
controlled localization interruption. Record each result before calling the
release field-accepted.
