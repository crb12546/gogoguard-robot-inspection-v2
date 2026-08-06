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

The browser never owns robot processes. A single on-robot navigation
supervisor owns exactly one Unitree bridge and one localization/Nav2/MPPI
generation. Browser actions are durable operations with an id and lifecycle;
refreshing or restarting the UI cannot create a second runtime generation.

During patrol, fixed-map localization projects the current pose onto the route
to produce monotonic progress. A transient localization dropout first enters
`HOLDING`; only a sustained dropout cancels the Nav2 goal. Recovery waits for a
stable localization window, clears stale local costmap data and resends only
the unfinished route suffix. Nav2 MPPI and Collision Monitor own local obstacle
avoidance. If FollowPath cannot progress, a bounded A* search over Nav2's
already footprint-inflated rolling costmap chooses a short path to a future
route anchor; MPPI tracks that path and then the unfinished route. It makes at
most two attempts and reports `BLOCKED` instead of waiting indefinitely. The
final Unitree bridge only validates authorization, command
freshness, finite values and absolute hardware limits; it is not a duplicate
obstacle planner.

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
