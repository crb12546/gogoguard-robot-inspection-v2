# Navigation orchestration model

This document describes the current product behavior around Nav2. It does not
invent a new localization, planner or controller algorithm.

## One normal path through the system

```text
blue patrol route -> FollowPath (MPPI) -> Unitree velocity receiver
                         |
             route ahead is truly occupied
                         v
              SmacPlanner2D finds a bypass
                         |
                         v
                  FollowPath (MPPI)
                         |
                         v
               rejoin the blue route
```

There is one path-following controller: MPPI. The normal blue route and a
planned bypass both go through it. The old custom local A* to RPP handoff is no
longer part of runtime behavior.

## What each layer owns

- The blue route says where the patrol should normally go.
- The green allowed-area mask says where planning is permitted. Nav2's local
  and global costmaps consume the same KeepoutFilter mask.
- MPPI continuously follows the current path and handles ordinary nearby
  obstacle avoidance.
- SmacPlanner2D is used only when the recorded route ahead remains occupied
  and a wider bypass is needed. Its planning window is the global rolling
  costmap, not MPPI's shorter controller horizon.
- Collision Monitor has one visible 0.48 m robot safety circle. It is the final
  emergency stop boundary, not another route planner.
- The Unitree receiver validates authorization, freshness, finite values and
  hardware limits. It does not decide whether the world is blocked.

## Obstruction and retry behavior

One occupied reading does not immediately become a blocked route. The runtime
requires consecutive occupied samples from a fresh, healthy local costmap.
Only then does it ask SmacPlanner2D for a path from the current pose to a future
point on the blue route, normally about eight metres ahead.

If a path exists, the same MPPI controller follows it and then resumes the
unfinished blue route. If no path exists yet, the dog holds zero velocity and
retries at the configured cadence. It does not give up after a small fixed
number of attempts; retry ends only when a path appears, the runtime becomes
unhealthy, or the operator stops the patrol.

## Localization behavior

FAST-LIO supplies continuous odometry. Fixed-map VGICP corrects accumulated
drift. A transient map match is never allowed to replace the last trusted
anchor immediately: recovery requires three mutually consistent candidates
before committing a new `map -> odom` transform.

If fixed-map localization becomes briefly unavailable, navigation enters
`HOLDING` and sends zero output. Once localization is stable again, the runtime
clears stale costmap data and resumes only the unfinished route suffix. A map
match failure is not fabricated into an obstacle.

## Failure classes

| Class | Evidence | Response |
|---|---|---|
| `TRANSIENT_CONTROL` | controller abort, clear route, healthy localization | briefly hold and retry the unfinished MPPI route |
| `LOCALIZATION_LOST` | fixed-map pose is stale or unusable | hold, recover a trusted anchor, refresh the costmap, resume |
| `PATH_OBSTRUCTED` | consecutive occupied route samples | request a Smac bypass and keep retrying while blocked |
| `COSTMAP_UNHEALTHY` | stale/wrong-frame costmap or occupied robot cell | refuse motion and report the exact gate |
| `ACTUATION_STALL` | fresh nonzero command but no measured movement | stop with an actuation diagnosis |
| `SYSTEM_FAULT` | binding, process or transport failure | revoke motion authorization |

## Stop ownership

The workstation's stop action cancels both FollowPath and any outstanding
Smac planning request, revokes motion authorization, stops Nav2/localization
and the Unitree velocity receiver, issues a final `StopMove`, and releases SDK
remote-control ownership. Repeating stop is safe.

## Acceptance scenarios

The implementation is only field-accepted after these receipts exist:

1. A clear full patrol completes on one MPPI controller.
2. A person crossing causes smooth avoidance or a temporary hold, then patrol
   continues after the person leaves.
3. A passable parked obstacle causes Smac to find a legal path inside the green
   area, MPPI follows it, and the dog rejoins the blue route.
4. A genuine dead end holds zero velocity and keeps replanning until the space
   clears or the operator stops.
5. A false VGICP jump is rejected while the last trusted odometry/map anchor
   remains active.
6. Operator stop leaves no controller, planner or motion bridge running.

Historical A*/RPP incident tests remain as regression evidence only; they are
not imported by the active orchestration path.
