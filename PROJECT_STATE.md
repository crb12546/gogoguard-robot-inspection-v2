# Current project state

Updated: 2026-08-05

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
- The existing Alibaba Cloud host is connected at `39.96.72.215`. A dedicated
  robot-side Ed25519 key with strict host verification is mounted read-only in
  the container; the cloud account password is not stored on the robot.
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

- V2 Git commit: `82135a6`
- Robot image config digest:
  `sha256:5d996c27735c2fb30125f5ec8c87676c24db7d7fd9d0c02b9b678796e2853ff6`
- Robot service: `enabled`, `active`, live status at port 8080
- Robot verification: stationary sensing, real recording, cloud GLIM round trip
  and Z1Pro LAN WebRTC preview passed; dynamic mapping and robot motion remain
  unverified

## Next experiment

Record one short human-remote-controlled out-and-back mapping loop and inspect
the returned GLIM candidate. Do not add autonomous motion to that experiment.
Separately correct the Site Console IMU instrumentation and add real rsync
progress or compressed transfer.
