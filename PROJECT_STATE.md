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
- 2026-08-05 local checks: 3 unit/vertical-slice tests pass; Python compilation,
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
- The ARM64 Dockerfile now defines compilation of the locked Livox SDK2,
  Livox Driver2 and FAST-LIO trees and one runtime composition for driver,
  calibration TF, FAST-LIO and Site Console. This is a build definition only;
  it has not been built on ARM64.
- Repository knowledge is now generated from per-module JSON manifests.
  `make knowledge-check` validates module paths/dependencies, the generated
  index and the immutable source snapshot.
- No V2 image has been built or deployed to the robot.
- Current Mac shell has no Docker executable, so the ARM64 image was not built.
- Livox/FAST-LIO sources are migrated and wired into the image definition but
  are not yet compiled into a V2 image. The definition is therefore still not
  deployable for real sensing.
- The old hardware record identifies the robot maintenance endpoint as
  `unitree@192.168.123.18`. On 2026-08-05 this Mac had no direct
  `192.168.123.0/24` interface and the endpoint did not answer two probes, so no
  SSH attempt or deployment was made.
- The cloud connection address is not present in this repository, the current
  Mac SSH config, or the relevant recent Codex task summaries. Those tasks also
  recorded that deployment credentials/entrypoint were unavailable. The cloud
  fixed job wrapper is not installed by V2 yet.
- No V2 code has controlled robot motion.
- First release target: live device status, 2D trajectory, 3D point-cloud
  preview, start/stop recording, sealed RecordingBundle, cloud map job, and
  visible 2D/3D result.

## Current deployed release

- V2 Git commit: none
- Robot image digest: none
- Robot verification: none

## Next experiment

Port the locked real Livox/FAST-LIO/mapping-capture closure into V2 modules,
build it in the ARM64 image, restore robot/cloud connectivity, and then perform
the zero-motion experiment in `docs/FIRST_ROBOT_EXPERIMENT.md`.
