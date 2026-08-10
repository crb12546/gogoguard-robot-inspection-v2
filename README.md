# Gogoguard Robot Inspection V2

V2 keeps the established Livox, FAST-LIO, GLIM, small_gicp, Nav2/MPPI and
Unitree capabilities, but gives them explicit product boundaries. It does not
copy the old repository's mixed application structure wholesale.

The first usable release is one visible three-end mapping loop:

```text
Go2 sensors -> sealed recording on robot -> resumable copy to Mac workstation
-> Alibaba Cloud GLIM -> versioned 2D/3D map on Mac
-> selected map version deployed to robot -> localization and Nav2 patrol
```

## Repository layout

```text
modules/contracts/         Stable cross-module data models
modules/device_io/         ROS 2 and hardware adapters
modules/data_capture/      Robot recording lifecycle and sealed bundles
modules/transfer/          Robot/Mac immutable artifact exchange
modules/localization/      FAST-LIO and fixed-map localization boundary
modules/navigation/        Nav2 runtime boundary
modules/route/             Versioned map-bound route model
modules/evidence/          Structured run events
services/map_factory/      Mac-to-cloud GLIM orchestration
apps/site_console/         Shared HTTP surface and browser assets
apps/field_workstation/    Mac workflow and historical catalog
deployment/                Separate Mac and robot releases
tests/                     Module and vertical-slice tests
```

These are code and contract boundaries, not dozens of microservices. Production
uses one main container on the robot and one lightweight container on the Mac.

## Run locally

The dependency-free demo produces a synthetic recording and map:

```bash
make demo
```

Open <http://127.0.0.1:8080>. For the real three-end workflow on the
commissioned Mac:

```bash
make workstation
```

`make workstation-container` remains the portable container form.

The Mac catalog and artifacts persist under `workstation-data/`.

The operator flow, button meanings, recovery path and current default
parameters are documented in [docs/FIELD_WORKSTATION_GUIDE.md](docs/FIELD_WORKSTATION_GUIDE.md).

For a product-level, plain-language explanation of the complete inspection
flow, the algorithms behind each capability, current limitations, the 3D/stair
roadmap and the distinction between mature components and our orchestration,
start with [docs/INSPECTION_SYSTEM_TECHNICAL_GUIDE.md](docs/INSPECTION_SYSTEM_TECHNICAL_GUIDE.md).

## Verify

```bash
make test
make ui-smoke
make compile
make container-validate
make knowledge-check
```

## Production ownership

- Robot: Livox/IMU/camera capture, sealed rosbag, FAST-LIO, fixed-map
  localization, Nav2/MPPI/collision monitoring and Unitree motion. It has no
  GitHub access, cloud SSH key, or source compilation responsibility.
- Mac: field UI, history, resumable recording transfer, GLIM orchestration,
  map review, selected-map deployment and robot release publication. It
  contains no ROS 2 and is never in the robot's realtime motion loop.
- Cloud: the existing Jazzy GLIM worker. It receives one sealed recording and
  returns immutable map artifacts plus progress markers.
- SaaS: the existing GoGoGuard service, integrated after the field navigation
  loop is accepted.
