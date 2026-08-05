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
  `sha256:d528f0a80bb880641e8f4afadf918027dca63661893fbc85b60887f4f4bb8786`;
  the ARM64 config digest deployed on the robot is
  `sha256:648cc1a3f3bcb71dfd4dd82446002501015029cd04d1de4083efcfed2920d225`.
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
- The cloud connection address is not present in this repository, the current
  Mac SSH config, or the relevant recent Codex task summaries. Those tasks also
  recorded that deployment credentials/entrypoint were unavailable. The cloud
  fixed job wrapper is not installed by V2 yet.
- No V2 code has controlled robot motion.
- First release target: live device status, 2D trajectory, 3D point-cloud
  preview, start/stop recording, sealed RecordingBundle, cloud map job, and
  visible 2D/3D result.

## Current deployed release

- V2 Git commit: `7eea253`
- Robot image config digest:
  `sha256:648cc1a3f3bcb71dfd4dd82446002501015029cd04d1de4083efcfed2920d225`
- Robot service: `enabled`, `active`, live status at port 8080
- Robot verification: stationary sensing and real recording passed; dynamic
  mapping, cloud GLIM round trip and motion remain unverified

## Next experiment

Restore the already-provisioned cloud GLIM host address and fixed job wrapper,
correct the Site Console IMU instrumentation, then record one short human-
remote-controlled out-and-back mapping loop. Do not add autonomous motion to
that experiment.
