# Gogoguard Robot Inspection V2

Clean V2 implementation for the Go2 inspection product. It deliberately does
not build on the unverified AI candidate implementation in the previous
repository.

The first usable release provides one visible mapping loop:

```text
Go2 sensors -> live trajectory and point cloud -> sealed recording
-> existing Alibaba Cloud GLIM worker -> 2D/3D map in the same console
```

## Repository layout

```text
modules/contracts/       Stable cross-module data models
modules/device_io/       Demo and ROS 2 sensor gateways
modules/data_capture/    Recording lifecycle and sealed bundles
modules/evidence/        Structured run events
services/map_factory/    Demo and remote cloud mapping adapters
apps/site_console/       HTTP API and browser UI
deployment/              ARM64 container and robot installation
tests/                   Module and vertical-slice tests
```

## Run locally

The local mode has no third-party Python dependency and generates a real demo
trajectory, point cloud, recording bundle, map job, PLY map, and overview SVG.

```bash
make demo
```

Open <http://127.0.0.1:8080>. Runtime data is written under `runtime-data/`.

## Test

```bash
make test
make ui-smoke
```

## Runtime modes

- `demo`: generated sensor stream and local map builder; used for UI and
  contract development.
- `robot`: ROS 2 sensor subscriptions plus `ros2 bag record`.
- Map worker `demo`: local deterministic artifact generation.
- Map worker `ssh`: sealed bundle transfer to the existing cloud worker. The
  remote host and fixed remote command are configuration, never source code.

## Production topology

- Robot: one primary Humble ARM64 container, host networking, persistent data
  mounted at `/var/lib/gogoguard`.
- Cloud: existing Jazzy GLIM worker; no robot DDS or motion control.
- SaaS: existing GoGoGuard service; V2 adds an adapter after the mapping loop is
  verified.
- Mac: browser console, Codex development, and image publication.
