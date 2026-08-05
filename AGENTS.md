# Gogoguard Robot Inspection V2

## Product rule

Build one reproducible Go2 inspection product through real vertical slices. The
first slice is: connect to the robot, show live trajectory and point cloud,
record a sealed dataset, submit it to the existing cloud GLIM worker, and show
the returned 2D/3D map.

## Repository rules

- Read `PROJECT_STATE.md` before changing code. It is the current factual state.
- Keep the old `gogoguard_dog_go2_real_code` repository read-only. Do not import
  its application implementation into V2. Verified hardware facts, datasets,
  third-party revisions, SaaS protocol facts, and cloud runtime facts may be
  referenced explicitly.
- One task owns one primary module. State cross-module changes before making them.
- Modules communicate through `modules/contracts`; do not import another
  module's internal implementation.
- Do not add a service, container, queue, or database unless a current vertical
  slice requires it.
- The robot has one primary ROS 2 Humble runtime container. GLIM remains on the
  existing Alibaba Cloud Jazzy worker. GoGoGuard SaaS remains external.
- A passing unit test means only offline verification. Never describe code as
  robot-verified or production-ready without the corresponding real receipt.
- Do not connect to or move the robot unless the user explicitly asks during
  the current task. Mapping motion is performed by the human remote control.
- Keep `PROJECT_STATE.md` current whenever a verified capability, deployed
  digest, known failure, or next experiment changes.

## Required checks

Run before handing off changes:

```bash
python3 -m unittest discover -s tests -v
python3 -m compileall -q modules apps services
```

For UI changes also run `make ui-smoke`. For container changes run
`make container-validate`; building the ARM64 image requires Docker.

