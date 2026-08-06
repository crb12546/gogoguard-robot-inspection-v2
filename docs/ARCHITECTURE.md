# Product architecture and code ownership

The product is divided by replaceable responsibility, not process count. Most
robot modules run together in one Humble ARM64 container; a module is a code
and contract boundary, not automatically a microservice.

```text
Application layer
  field_workstation  mapping, route, history and engineering workflow
  SaaS adapter       remote tasks, media, voice and teleoperation (later)

Orchestration layer
  mission            inspection state machine (later)
  route              versioned map-bound route
  inspection         checkpoint camera/actions (later)

Robot capability layer
  localization       FAST-LIO odometry + GLIM-map VGICP localization
  navigation         Nav2 FollowPath + MPPI + collision monitoring
  data_capture       start/stop/seal sensor recording

Robot hardware boundary
  device_io          Livox, IMU, Z1Pro and Unitree adapters
  calibration        base/lidar/IMU transforms bound to robot identity

Cross-end infrastructure
  transfer           immutable robot <-> Mac artifacts with resume + hashes
  map_factory        Mac -> cloud GLIM -> versioned map on Mac
  evidence           append-only module events
  contracts          persisted and cross-module schemas
```

## Runtime flow

```text
robot device_io -> Edge Agent live status and direct LAN camera preview
robot data_capture -> sealed RecordingBundle
workstation transfer -> verified local recording version
workstation map_factory -> cloud GLIM -> verified map version
workstation UI -> 2D overview + interactive 3D cloud + history
selected map -> robot import -> localization -> Nav2 route execution
```

Live ROS traffic stays on the robot. The camera is viewed directly over the
LAN; sealed recordings move only after stop. Mac is not required while the dog
is localizing or moving. If a transfer fails, the same task resumes from its
partial file; it does not require recording the site again.

## Change rules

- Replace an algorithm behind its current boundary. A localization change must
  not require rewriting recording, history or cloud workflow.
- Product state uses `modules/contracts`; ROS messages stay inside robot
  adapters.
- Raw recordings, derived maps, routes and diagnostic events are independently
  versioned and reproducible.
- Only measured resource or fault-isolation needs justify another process or
  container.
- Robot releases are built and staged from Mac. The robot never pulls GitHub
  source and never compiles the application.
- Every capability has distinct offline, container, stationary-robot and
  moving-field receipts. One kind never implies another.
