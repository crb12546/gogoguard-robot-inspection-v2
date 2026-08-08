# Navigation orchestration model

This document owns the product-level behavior that connects fixed-map
localization, Nav2 controllers, the local costmap and the Unitree motion
boundary. It does not redefine FAST-LIO, VGICP, MPPI, RPP, A* or Collision
Monitor algorithms.

## Product invariant

A patrol normally follows the released recorded route with MPPI at the route
speed. A local detour is a short-lived response to verified route obstruction;
it ends at a selected future route anchor and hands the unfinished route back
to MPPI. A localization, TF, controller or actuation failure is not an
obstacle unless obstacle evidence independently proves that it is one.

Only one controller owns a path at a time:

```text
FollowPath (MPPI) -> verified obstruction -> DetourPath (RPP)
DetourPath (RPP) -> rejoin anchor reached -> FollowPath (MPPI suffix)
```

## 2026-08-09 deployed-baseline audit

The deployed `evidence-detour-r3` behavior is not consistent with that
invariant:

1. `PatrolRuntimeManager._result_callback` sends every non-success,
   non-cancelled Nav2 result to `_try_local_replan()`. It has no failure class
   and does not first prove route obstruction.
2. `_try_local_replan()` accepts any fresh costmap in which A* can find a path.
   In open space this is normally true, so a transient controller or TF abort
   is likely to enter detour mode.
3. The detour goal concatenates the local A* path with the entire remaining
   recorded route. The recorded rejoin index is metadata only; there is no
   transition back to MPPI at that index.
4. After the patrol has existed for `blockedDecisionS`, an otherwise
   unclassified failure is reported as `LOCAL_PATH_BLOCKED`. This uses patrol
   age as obstacle evidence and conflates TF, controller, actuation and path
   failures.
5. Deployed product behavior is injected into the immutable capability snapshot by a
   build-time overlay that mixes speed compatibility, controller selection,
   VoxelLayer geometry and Unitree limits. The compatibility approach remains
   useful for provenance, but the product state machine has outgrown it.
6. Existing compatibility tests prove that patch strings and parameter bounds
   are present. They do not exercise controller entry/exit semantics.

The field incident for `map-8ddcf3f8c078` demonstrates all four runtime
defects. MPPI followed the route for 48.6 seconds, then aborted on a 0.509 ms
future-TF lookup. With no obstacle event and a clear forward costmap, the
runtime accepted a detour at route index 116 with rejoin index 131. RPP then
continued to route index 214 at 0.24 m/s instead of returning ownership to
MPPI. The final 2.5-second net displacement was 0.051 m despite a nonzero final
command, so Nav2's progress checker aborted and the runtime mislabeled the
actuation/progress failure as a blocked path.

## Failure classes

| Failure class | Required evidence | Product response |
|---|---|---|
| `TRANSIENT_CONTROL` | controller abort without route obstruction while localization and sensor time remain usable | hold zero output briefly, restamp and retry the unfinished MPPI route |
| `LOCALIZATION_LOST` | fixed-map localization gate is stale, unusable or not tracking | `HOLDING`; cancel after grace period; resume MPPI suffix after stable recovery |
| `PATH_OBSTRUCTED` | the footprint-inflated local costmap blocks consecutive samples of the recorded route ahead | compute one bounded local detour and give only that segment to RPP |
| `CONTROLLER_FAILED` | controller retry budget exhausted without obstruction or actuation-stall evidence | stop with the controller failure class; do not fabricate an obstacle |
| `ACTUATION_STALL` | a fresh nonzero final command exists but measured pose displacement stays below the commissioned movement requirement | stop with an actuation diagnosis; retain command and pose evidence |
| `SYSTEM_FAULT` | binding, process, transport or non-recoverable runtime gate failure | revoke motion authorization and require runtime recovery |

Humble's `FollowPath` action result contains only an empty result plus the
action status. The runtime therefore cannot recover a detailed failure reason
from the action result alone. It must classify using independent runtime gates,
route obstruction evidence, final-command evidence and measured motion. An
unclassified first abort receives a bounded MPPI retry, never an automatic
detour.

## State model

```text
BOOTING -> LOCALIZING -> READY
READY -> FOLLOWING
FOLLOWING -> HOLDING -> FOLLOWING               localization recovers
FOLLOWING -> RETRYING -> FOLLOWING              transient/unclassified abort
FOLLOWING -> DETOURING                          verified obstruction
DETOURING -> REJOINING -> FOLLOWING             local segment completes
FOLLOWING -> COMPLETED                          route completes
any moving state -> STOPPING -> READY            operator stop
any moving state -> BLOCKED                      verified path obstruction only
any moving state -> FAULT                        controller/actuation/system failure
```

`DETOURING` owns only the current pose-to-anchor path. `REJOINING` is the
explicit handoff that restamps and submits the unfinished recorded-route suffix
to MPPI. Reaching a route-progress index is not itself controller ownership;
ownership changes only after the RPP action succeeds and the MPPI suffix is
accepted.

## Parameter ownership

- Route speed and MPPI controller parameters shape normal tracking.
- Detour speed shapes only the bounded RPP segment and must remain above the
  commissioned Go2 gait floor after Collision Monitor slowdown.
- Collision polygons and slowdown remain owned by Collision Monitor.
- Progress timing detects lack of motion. It is not an obstacle classifier.
- Humble `PoseProgressChecker` treats either 0.15 m translation or 0.15 rad
  rotation as progress, so RPP heading alignment is not timed out as a stall.
- Obstruction confirmation is derived from the already footprint-inflated
  local costmap along the recorded route, not from patrol age.

## Acceptance scenarios

The navigation slice is not field-accepted until all scenarios pass with a
runtime trace:

1. A clear route stays on MPPI and uses the released route speed.
2. A sub-frame TF timing abort retries MPPI without entering detour mode.
3. A clear local costmap never authorizes a detour.
4. A real route obstruction authorizes one bounded A* detour.
5. RPP reaches the rejoin anchor, then MPPI accepts the unfinished route.
6. A nonzero final command without measured movement reports
   `ACTUATION_STALL`, not `PATH_OBSTRUCTED`.
7. Localization loss holds, cancels and resumes the route suffix without
   resetting route progress.
8. Operator stop is idempotent and leaves no controller or motion bridge
   orphan.

Historical incident replays are regression fixtures. Offline success never
replaces the final clear-route and obstructed-route robot receipts.

## Deployment status

The V2 branch now owns `go2_nav2_runtime` under `modules/navigation/ros`
instead of reconstructing product behavior with build-time overlays. The
Unitree receiver delivery limits are likewise an owned `device_io` source.
The failure classifier, route-obstruction proof and controller handoff have
offline replay tests, including the `map-8ddcf3f8c078` evidence. This code has
replaced `evidence-detour-r3` on the robot as
`v2-edge-20260809-orchestration-r2`. Static sensor, service and motion-bridge
startup receipts passed. A wrong-environment localization attempt correctly
remained unauthorized and sent no patrol goal. The navigation slice is not
field accepted until the operator selects a matching map and the acceptance
scenarios above produce robot receipts.
