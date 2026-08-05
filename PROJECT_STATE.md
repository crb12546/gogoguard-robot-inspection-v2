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
- No V2 image has been built or deployed to the robot.
- Current Mac shell has no Docker executable, so the ARM64 image was not built.
- Livox/FAST-LIO source revisions are recorded but are not yet compiled into the
  V2 image. The current container definition is therefore not deployable for
  real sensing yet.
- Robot and cloud connection addresses are not present in this repository or
  the current Mac SSH config. The cloud fixed job wrapper is not installed by
  V2 yet.
- No V2 code has controlled robot motion.
- First release target: live device status, 2D trajectory, 3D point-cloud
  preview, start/stop recording, sealed RecordingBundle, cloud map job, and
  visible 2D/3D result.

## Current deployed release

- V2 Git commit: none
- Robot image digest: none
- Robot verification: none

## Next experiment

Restore explicit robot/cloud connection configuration, complete and build the
locked Livox/FAST-LIO ARM64 image, then perform the zero-motion robot experiment
in `docs/FIRST_ROBOT_EXPERIMENT.md` while the robot is externally powered.
