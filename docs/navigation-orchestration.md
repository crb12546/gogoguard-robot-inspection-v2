# Navigation orchestration model

This document describes the generation-9 candidate around Nav2. It composes
mature Nav2 capabilities; it does not invent a localization, planner or
controller algorithm. The accepted robot baseline remains V6 until this
candidate receives an explicit deployment and field receipt.

## Normal execution path

```text
reviewed static map + green allowed area + current map pose
                              |
                              v
          next ordered checkpoint (or final route endpoint)
                              |
                       SmacPlanner2D
                              |
                         FollowPath
                           (MPPI)
                              |
                       Unitree receiver
```

The blue recording is no longer replayed sample by sample. It owns patrol
order, checkpoint anchors, the default start/end and human intent. For each
leg, SmacPlanner2D plans from the robot's current trusted map pose to the next
checkpoint or the final endpoint. The same MPPI controller follows that path
and continuously reacts to live obstacles.

This matters when a wall is wider than MPPI's local horizon: global planning
can see the reviewed map extent and choose a doorway or the far end of the
wall. MPPI is not asked to discover an unseen seven-metre-wide exit by standing
in place.

## The three map products remain separate

| Product | Owner | Purpose |
|---|---|---|
| immutable GLIM PCD | localization | VGICP geometric matching and map pose correction |
| reviewed static occupancy map | route/workstation | permanent walls, pillars and navigable topology for Nav2 |
| live obstacle layers | navigation | people, cars and other current obstacles |

The static navigation map is derived from an operator-selected PCD height
slice. It aggregates points into 0.10 m cells, drops isolated cell returns,
clears the physically recorded body corridor and then applies explicit manual
block/clear overrides. It never rewrites the localization PCD.

## What each layer owns

- The blue route owns task direction, checkpoint order and default endpoints.
- The green area is an absolute planning boundary through KeepoutFilter.
- The static map tells SmacPlanner2D where permanent geometry exists.
- The global live obstacle layer can invalidate a route because of a current
  person or parked vehicle and trigger a fresh global plan.
- The rolling local costmap gives MPPI the nearby static and live obstacles.
- MPPI owns smooth local tracking, lateral clearance and short reactions.
- The navigation-only obstacle filter removes returns that geometrically fall
  inside the rigid body/backboard volume; localization keeps the complete
  calibrated cloud.
- MPPI, both costmaps and Collision Monitor share one rectangular hard
  envelope: the measured body plus 0.10 m shoulder padding, giving x
  `[-0.43, 0.50]` and y `+/-0.30 m`. Inflation outside it is a soft planning
  preference, not another hard body.
- The Unitree receiver owns authorization, freshness and hardware limits, not
  path selection.

## Retry and localization behavior

A point goal does not become a permanent failure after a small retry count. If
Smac or MPPI cannot complete the leg while localization and costmaps remain
recoverable, the runtime holds zero velocity and replans at the configured
cadence. Retry ends when a legal path appears, the runtime develops a
non-recoverable fault, the mission completes, or the operator stops it.

FAST-LIO supplies continuous odometry. Fixed-map VGICP supplies the map anchor.
If map localization becomes temporarily unusable, motion is held, the last
trusted anchor is retained, recovery still requires three mutually consistent
matches, stale costmap data is cleared, and planning resumes from the current
trusted map pose to the unfinished goal. Route-index projection is not used to
guess the robot's motion direction during a point-goal leg.

## Checkpoint behavior

The next planning target is the next incomplete checkpoint's bound route index.
When the planned leg succeeds, that index is committed and the existing
true-stop/camera/platform workflow starts. After the checkpoint is completed,
the next leg is planned from the current map pose. After the final endpoint is
reached, the mission completes.

## Failure classes

| Class | Evidence | Response |
|---|---|---|
| `TRANSIENT_CONTROL` | controller abort, usable localization/costmap | hold and retry the current point goal |
| `LOCALIZATION_LOST` | fixed-map pose stale or unusable | retain anchor, recover, refresh costmap, replan current goal |
| `PATH_OBSTRUCTED` | current global path cannot be completed | keep requesting a legal Smac plan inside the green area |
| `COSTMAP_UNHEALTHY` | stale/wrong-frame grid or occupied robot cell | revoke motion and report the exact gate |
| `ACTUATION_STALL` | fresh nonzero command but no measured movement | stop with an actuation diagnosis |
| `SYSTEM_FAULT` | binding, process or transport failure | revoke motion authorization |

## Stop ownership

The workstation stop action cancels FollowPath and outstanding Smac planning,
revokes motion authorization, stops Nav2/localization and the Unitree velocity
receiver, issues a final `StopMove`, and releases SDK remote-control ownership.
Repeating stop is safe.

## Required field acceptance

Offline tests and a Mac real-map preview are not robot acceptance. Generation 9
is accepted only after all of these receipts exist:

1. A clear full patrol completes every checkpoint and endpoint.
2. A person crossing causes smooth avoidance or a temporary zero hold, then the
   same goal continues after the person leaves.
3. A passable parked obstacle produces a legal Smac path inside the green area
   and MPPI follows it.
4. A wall wider than the local MPPI window is bypassed through a mapped opening.
5. A genuine dead end holds zero and keeps replanning until clear or stopped.
6. A false VGICP jump is rejected while the trusted anchor remains active.
7. Operator stop leaves no controller, planner or motion bridge running.
