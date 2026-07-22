# Contact Pipeline modules

The contact pipeline has one authoritative data flow:

`Persistent topology -> persistent classification -> timestep views -> solvers`.

## Core

Owns orchestration only. It schedules the phases and the XPBD iteration loop. It must not implement broad-phase caching, lifecycle classification, or editor diagnostics.

## BroadPhase

Produces frame-local candidate interaction pairs. It never owns cross-frame contact state.

## Persistent

Owns stable entity-pair topology, contact lifecycle classification, classification versions, and derived stable keys. This is the only cross-frame contact authority.

## Prediction

Builds timestep/substep envelopes and frame-local interaction/contact views from the persistent authority or the full-sweep reference path.

## SoftAvoidance

Consumes only the compact frame-local soft-avoidance view. It must not read legacy Fat AABB caches or the full persistent neighbor set directly.

## Legacy/FatAabb

Quarantined historical implementation. Files remain temporarily for compatibility and diagnostics while the next refactor phase removes runtime ownership and dead execution paths. New production code must not add dependencies on this folder.
