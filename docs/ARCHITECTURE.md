# Product architecture and code ownership

The product is divided by replaceable responsibility, not by process count.
Most modules run together in one robot process/container. A module is a code
boundary and contract boundary; it is not automatically a microservice.

```text
Product applications
  site_console      map/route field workflow and diagnostics
  SaaS adapter      remote jobs, media and teleoperation (later slice)
  diagnostics       engineering receipts (later slice)

Product orchestration
  mission           inspection state machine (later)
  route             route model, validation and execution (later)
  inspection        camera and inspection actions (later)

Robot capabilities
  map_factory       recording bundle -> versioned 2D/3D map
  localization      sensor stream -> current pose (FAST-LIO adapter first)
  navigation        goal -> safe velocity (later)
  data_capture      start/stop/seal/replay sensor evidence

Hardware and infrastructure
  device_io         Livox, IMU, camera and Unitree adapters
  evidence          append-only operational evidence
  contracts         cross-module messages and persisted schemas
```

## First vertical slice

```text
device_io
  -> contracts.SensorSnapshot
  -> site_console live 2D trajectory + 3D cloud
  -> contracts.CameraStreamStatus + Z1Pro WebRTC preview
  -> data_capture RecordingBundle
  -> map_factory MapJob
  -> cloud GLIM adapter
  -> site_console map result
```

Implemented now: `contracts`, `device_io`, `data_capture`, `evidence`,
`map_factory`, and `site_console`. The other names are reserved responsibilities,
not empty services that must be built in advance.

## Change rules

- A new sensor or algorithm implements an existing boundary first. For example,
  replacing FAST-LIO changes `device_io/localization` adapters, not recording,
  UI or cloud job state.
- Product state uses `modules/contracts`; ROS messages stay inside robot adapters.
- Raw recording, derived maps and diagnostic events are different artifacts and
  remain independently reproducible.
- A module can become a separate process only after measured resource, fault
  isolation or deployment needs justify it.
- Every capability has four possible receipts: offline test, container build,
  stationary robot observation, and moving field acceptance. They are never
  treated as equivalent.
