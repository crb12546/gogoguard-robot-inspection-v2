# Gogoguard Robot Inspection V2

## Product rule

Repackage the established Go2 capability stack into one reproducible inspection
product through real vertical slices. The current slice is a three-end field
workflow:

1. The robot runs Livox/IMU/camera capture, sealed recording, FAST-LIO,
   fixed-map localization, Nav2/MPPI/collision handling and Unitree motion.
2. The Mac field workstation owns the browser workflow, history, resumable
   robot transfer, cloud orchestration, map review and release publication.
3. The existing Alibaba Cloud worker runs GLIM and returns immutable map
   artifacts. It does not own robot ROS traffic or motion.

The existing GoGoGuard SaaS is a later fourth integration boundary. It is not
part of the current record-map-localize-patrol acceptance loop.

Live ROS traffic stays on the robot. A sealed recording moves robot -> Mac ->
cloud; GLIM artifacts return cloud -> Mac; only a selected map version moves
Mac -> robot. The robot must remain able to localize and stop safely without
the Mac, cloud or SaaS being online.

## Start every Codex task here

- Read `PROJECT_STATE.md` section **New-task handoff — start here** first. Use
  its current snapshot and final next-work list; consult dated sections only
  for the evidence or provenance needed by the task.
- Read `docs/generated/repository-index.md`, then only the manifest for the
  primary module being changed.
- State the primary owning module and any necessary cross-module contract
  changes before editing.
- Never infer robot or cloud deployment from local tests. Check the recorded
  deployed digest and explicitly ask before connecting to or moving the robot.

## Repository rules

- Do not begin with a whole-repository archaeology pass when the generated
  index identifies the owner.
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
- The robot has one primary ROS 2 Humble runtime container and no GitHub or
  cloud SSH responsibility. The commissioned Mac runs the field workstation
  and publishes tested releases. GLIM remains on the existing Alibaba Cloud
  Jazzy worker. GoGoGuard SaaS remains external.
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

## Git and release rules

- GitHub communicates with the Mac only. The robot never runs `git pull` and
  never compiles the product; the cloud worker never pulls an arbitrary latest
  branch.
- `main` is the accepted baseline. Develop a coherent vertical slice on one
  short-lived integration branch and open one draft PR. Use extra branches only
  for genuinely independent module work, then merge them into that integration
  PR in dependency order.
- One Codex task owns one primary module and should produce an intentional,
  reviewable commit. Do not let multiple tasks edit the same composition,
  deployment or contract files without an explicit integration owner.
- Never commit recordings, maps, runtime data, logs, credentials, SSH keys or
  exported image archives. Persist them under ignored runtime/release roots.
- Every deployed robot release is identified by both Git commit and container
  image digest. Record deployment and field receipts in `PROJECT_STATE.md` only
  after they actually occur.

## Required checks

Run before handing off changes:

```bash
python3 -m unittest discover -s tests -v
python3 -m compileall -q modules apps services
make knowledge-check
```

For UI changes also run `make ui-smoke`. For container changes run
`make container-validate`; building the ARM64 image requires Docker.
