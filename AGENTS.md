# Gogoguard Robot Inspection V2

## Product rule

Repackage the established Go2 capability stack into one reproducible inspection
product through real vertical slices. The first slice is: run the existing
Livox/FAST-LIO capability on the robot, show live trajectory and point cloud,
record a sealed dataset, submit it to the existing cloud GLIM worker, and show
the returned 2D/3D map.

## Repository rules

- Read `PROJECT_STATE.md` before changing code. It is the current factual state.
- Read `docs/generated/repository-index.md`, then the manifest under
  `architecture/modules` for the module being changed. Do not begin with a
  whole-repository archaeology pass when the index identifies the owner.
- Keep the old `gogoguard_dog_go2_real_code` repository read-only. It is the
  capability source for the frozen technology stack. Selectively port its
  Livox, FAST-LIO, GLIM, small_gicp, Nav2, Unitree and hardware integration
  code/configuration with provenance and tests. Do not wholesale-copy its large
  application files, mixed orchestration, generated output, or duplicate
  deployment entrypoints.
- V2 is a reorganization of proven/candidate capabilities, not a new algorithm
  project. Do not replace a frozen technology without an explicit product
  decision backed by real data.
- One task owns one primary module. State cross-module changes before making them.
- Modules communicate through `modules/contracts`; do not import another
  module's internal implementation.
- Do not add a service, container, queue, or database unless a current vertical
  slice requires it.
- The robot has one primary ROS 2 Humble runtime container. GLIM remains on the
  existing Alibaba Cloud Jazzy worker. GoGoGuard SaaS remains external.
- Demo mode is only a UI/contract regression harness. It is never a product
  milestone and never substitutes for the migrated capability stack.
- A passing unit test means only offline verification. Never describe code as
  robot-verified or production-ready without the corresponding real receipt.
- Do not connect to or move the robot unless the user explicitly asks during
  the current task. Mapping motion is performed by the human remote control.
- Keep `PROJECT_STATE.md` current whenever a verified capability, deployed
  digest, known failure, or next experiment changes.
- Update the owning module manifest when its entrypoint, dependency, contract,
  runtime or status changes. Run `make knowledge` after manifest changes;
  generated documentation is never edited manually.

## Required checks

Run before handing off changes:

```bash
python3 -m unittest discover -s tests -v
python3 -m compileall -q modules apps services
make knowledge-check
```

For UI changes also run `make ui-smoke`. For container changes run
`make container-validate`; building the ARM64 image requires Docker.
