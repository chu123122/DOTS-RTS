# Incremental contact-pipeline architecture

## Authoritative data flow

```text
Guarded swept proxies
        ↓
Persistent neighbor topology
        ↓ versioned classification
Persistent predictive contact lifecycle
        ↓ derived stable keys
Timestep interaction / soft / active / dormant views
        ↓
Motion integration + Soft Avoidance + XPBD / wall projection
```

## Ownership

- **BroadPhase** produces frame-local interaction candidates. It owns no cross-frame state.
- **Persistent** owns stable entity-pair topology, classification versions, lifecycle,
  stable normals, and stable keys used to derive views.
- **Prediction** owns timestep envelopes, view construction, safety validation, and the
  single incremental-repair/full-rebuild fallback boundary.
- **SoftAvoidance** consumes only the compact soft view.
- **Solver** consumes only frame-local active constraints and may not mutate topology. Gauss–Seidel remains the serial reference; Jacobi evaluates pairs in parallel and gathers deterministic body corrections through a frame-local CSR incident index.
- **Motion** owns substep velocity preparation, position prediction, and reconstruction.
- **Diagnostics** observes completed telemetry snapshots; it is not a state authority.

### Runtime state versus telemetry

`IncrementalContactCacheState` and the persistent containers form the
authoritative gameplay state. Cache validity, topology epochs, classification
epochs, lifecycle state, dirty worksets and fallback decisions may only depend
on those runtime structures and current solver inputs.

`IncrementalContactPipelineStatistics` is diagnostics-owned telemetry. It may
record counters, timings, ratios and oracle results after work is performed, but
its fields are not valid control inputs. An oracle mismatch invalidates the
runtime state through an explicit write to `IncrementalContactCacheState`; the
telemetry counter itself never becomes the authority.

## Lifetimes

| Data | Lifetime | Stable identity |
|---|---|---|
| Persistent proxy / neighbor / contact | Cross timestep | Entity pair |
| Incremental cache state / dirty certificate | Cross timestep or current repair | Entity / topology epoch |
| Pipeline telemetry | One completed timestep | Snapshot timestep |
| Timestep interaction / soft / active view | One timestep or rebuild interval | Current BodyIndex mapping |
| Lambda / activation flags | One substep or timestep | Frame-local pair |
| Iteration correction | One XPBD iteration | Pair index / BodyIndex |

## Correctness invariants

1. Every possible contact lies in the current interaction view or touches a pending dirty body.
2. Persistent normal orientation is converted to current BodyIndex order once per timestep.
3. Soft/RVO output cannot leave the proven interaction envelope without being clamped.
4. Predicted positions and solver corrections are validated after their respective mutations.
5. Failed incremental proof converges on one full-sweep fallback path.
6. Jacobi pair evaluation reads one immutable position snapshot; body gather applies contributions in deterministic incident-pair order without float atomics.
7. Oracle missing-pair count must remain zero when diagnostics are enabled.
8. Telemetry fields are write-only observations for the simulation pipeline and cannot drive correctness decisions.

## Explicit exclusions

Contact-island or body sleeping is not implemented. Stable active contacts continue to be
solved; this refactor changes ownership and work-set selection, not sleep/wake semantics.
