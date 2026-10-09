# GoGoGuard Robot Inspection V2

GoGoGuard V2 is the active field repository for a Unitree Go2 inspection
system. It preserves the commissioned Livox, FAST-LIO, GLIM, small_gicp,
Nav2/MPPI and Unitree capabilities behind explicit product modules.

The proven core flow is:

```text
robot records sensors and checkpoints
-> Mac resumes and verifies the sealed recording
-> Alibaba Cloud GLIM creates an immutable 3D map and optimized trajectory
-> Mac reviews the map and versions the blue route, green allowed area and checkpoints
-> robot localizes and patrols locally
-> GoGoGuard SaaS may dispatch the selected mission and receive media/status/evidence
```

This is a field-commissioning product, not a fully accepted inspection release.
Read [PROJECT_STATE.md](PROJECT_STATE.md) before assuming that local code is
built, deployed or robot-verified.

## Start here

For the frozen 2026-10-09 source snapshot, read the
[source handoff](docs/releases/source-freeze-20261009.md). Its immutable version
is `v2-source-20261009`; later development must use a separate commit/version.

1. [Documentation index](docs/README.md)
2. [Architecture and developer handoff](docs/INSPECTION_SYSTEM_TECHNICAL_GUIDE.md)
3. [Current deployed and field state](PROJECT_STATE.md)
4. [Generated module ownership index](docs/generated/repository-index.md)
5. [System audit and evidence notes](docs/system-audit/README.md)
6. [Interactive system guide](system-guide/index.html): download/clone the
   repository and open this file in a browser; it needs no server.

Historical handoffs are under `docs/archive/` and are not current operating
instructions.

## Repository layout

```text
architecture/modules/    Machine-readable module ownership manifests
modules/contracts/       Stable cross-module and persisted schemas
modules/calibration/     Authoritative sensor transforms
modules/device_io/       Livox, camera, gimbal, audio and Unitree adapters
modules/data_capture/    Recording, sealing and checkpoint samples
modules/transfer/        Verified robot/Mac artifact exchange
modules/localization/    FAST-LIO and fixed-map VGICP boundary
modules/route/           Map-bound route, allowed area and checkpoint assets
modules/navigation/      Nav2/MPPI runtime, recovery and stop receipts
modules/inspection/      Checkpoint frame/evidence handling
modules/mission/         Inspection task state machine
modules/evidence/        Events, incidents and replay evidence
modules/interaction/     LiveKit session, wake and audio safety
services/map_factory/    Mac/cloud GLIM orchestration and editor sessions
services/interaction_edge/ Robot realtime-media runtime adapter
services/platform_edge/  Heartbeat and allow-listed platform bridge
apps/site_console/       Shared narrow HTTP API and browser UI
apps/field_workstation/  Mac field workflow composition
deployment/              Cloud, container, robot and workstation releases
third_party/locked_stack/Content-locked capability build provenance
tests/                   Unit and vertical-slice regression tests
```

These are code responsibilities, not separate microservices. The robot runs
one primary ARM64 ROS 2 Humble container. The commissioned Mac normally runs
the workstation as a native process. Alibaba Cloud owns only the pinned GLIM
worker; GoGoGuard SaaS remains an external boundary.

## Run the workstation

For the commissioned Mac workflow:

```bash
make workstation
```

Open <http://127.0.0.1:8080>. The containerized workstation form remains
available with `make workstation-container`. A dependency-free synthetic UI
harness is available with `make demo`, but demo mode is never field evidence.

Real recordings, maps, incidents and platform bundles live under ignored
`workstation-data/` or `runtime-data/`. Image exports live under ignored
`release-cache/`; none are pushed to GitHub.

## Verify before handoff

```bash
make test
make compile
make ui-smoke
make container-validate
make knowledge-check
git diff --check
```

An ARM64 Docker build and robot deployment are separate, explicitly authorized
steps. The robot never runs `git pull` and never compiles this repository.

## Development rules

- Read `AGENTS.md` and the current project-state handoff first.
- Give each change one primary owning module; share only versioned contracts.
- Preserve the locked established algorithms unless measured field evidence
  and an explicit product decision justify replacement.
- Never commit credentials, recordings, maps, logs, exported images or runtime
  data.
- A passing test is offline evidence only. Record commit, image digest and real
  field receipt separately when deployment is actually authorized and run.
