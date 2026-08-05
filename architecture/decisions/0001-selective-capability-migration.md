# ADR 0001: retain the established capability stack

Status: accepted, 2026-08-05

## Decision

V2 reorganizes the existing Go2 inspection capability stack; it does not
replace it. Livox Driver2, the modified FAST-LIO runtime, cloud GLIM,
small_gicp, Nav2 Humble/MPPI and Unitree SDK2 remain the technical baseline.

Algorithm and hardware integration sources are migrated from an immutable old
repository commit with provenance. Product orchestration, Site Console, SaaS
composition and deployment ownership are rebuilt around explicit V2 module
contracts. Large mixed old application files are not copied wholesale.

## Why

Real robot debugging requires the hardware and algorithm work already present.
Starting those capabilities again would discard evidence and make field tests
meaningless. Copying the entire old application would preserve the coupling
that V2 exists to remove. Selective migration keeps both continuity and a clear
place to diagnose each failure.

## Enforcement

- `dependencies/capability_migration.lock.json` identifies source trees.
- `third_party/locked_stack` preserves the exact first-slice source snapshot.
- `make knowledge-check` rejects snapshot drift and stale module knowledge.
- Adapted code lives in its owning V2 module and has parity tests against the
  locked source contracts.
- Only real build and robot receipts can advance a capability beyond offline
  status in `PROJECT_STATE.md`.
