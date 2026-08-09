# Current project state

Updated: 2026-08-09

## Verified external facts

- Robot: Unitree Go2 expansion dock, 100 TOPS class, Jetson Orin NX 16 GB.
- Robot host: JetPack 5 / Ubuntu 20.04 / L4T R35.3.1; Docker was previously
  verified on the robot.
- Target robot userspace: one Linux ARM64 Ubuntu 22.04 / ROS 2 Humble container.
- Sensor: replacement Livox MID-360S `ARMCP6B0035634` at `192.168.1.134`.
  Camera: Z1Pro.
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
- V2 has controlled robot motion during the recorded field patrols documented
  below; release installation and static acceptance do not command motion.
- First release target: live device status, 2D trajectory, 3D point-cloud
  preview, start/stop recording, sealed RecordingBundle, cloud map job, and
  visible 2D/3D result.

## Current deployed release

- Active V2 implementation commit: `3cf679716ce177e1e91064474852da0d8ddfa3d9`.
- Robot image config digest:
  `sha256:15e2b7c6951ea27ea4131bfbec4fe35d5302b9750ecafe700bf25665d6d785a1`
- Robot service: `enabled`, `active`, live status at port 8080
- Active image: `gogoguard-robot-inspection:v2-edge-20260809-planar-release-r4`; robot
  runtime no longer mounts cloud SSH material and its map worker is `none`.
- The active release archive SHA-256 is
  `83b4cc8e0ce2d9d0feff31a804fe3cfc64d53fd139ac838d044ae5b37918d7a2`.
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

## 2026-08-09 cruise-speed contract correction (offline; deployment pending)

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
- The offline correction treats 0.60 m/s as the clear-route cruise request,
  scales MPPI forward sampling to 0.30 m/s standard deviation at that request,
  raises the default batch from 700 to 1,000 within the existing compute
  budget, and raises PathFollow progress authority from 5.0 to 8.0. StopZone,
  SlowZone and collision thresholds are unchanged. Robot deployment and a
  measured clear-route speed receipt are still required.

## Next experiment (supersedes all older paragraphs)

Keep `planar-release-r4` fixed and collect only the remaining acceptance
receipts: one obstacle that leaves a physically passable side corridor and
therefore exercises A* -> `DetourPath` -> MPPI rejoin; localization-loss route
resume; and handheld-remote movement immediately after a stop from an active
patrol and from `FAULT`. The terminal dead-end stop and normal MPPI travel are
already field verified; do not weaken obstacle thresholds to make an actually
sealed path appear passable.
