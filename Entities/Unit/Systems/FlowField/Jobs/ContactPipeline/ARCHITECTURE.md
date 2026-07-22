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
- **Solver** consumes only frame-local active constraints and may not mutate topology.
- **Motion** owns substep velocity preparation, position prediction, and reconstruction.
- **Diagnostics** observes completed snapshots; it is not a state authority.

## Lifetimes

| Data | Lifetime | Stable identity |
|---|---|---|
| Persistent proxy / neighbor / contact | Cross timestep | Entity pair |
| Timestep interaction / soft / active view | One timestep or rebuild interval | Current BodyIndex mapping |
| Lambda / activation flags | One substep or timestep | Frame-local pair |
| Iteration correction | One XPBD iteration | None |

## Correctness invariants

1. Every possible contact lies in the current interaction view or touches a pending dirty body.
2. Persistent normal orientation is converted to current BodyIndex order once per timestep.
3. Soft/RVO output cannot leave the proven interaction envelope without being clamped.
4. Predicted positions and solver corrections are validated after their respective mutations.
5. Failed incremental proof converges on one full-sweep fallback path.
6. Oracle missing-pair count must remain zero when diagnostics are enabled.

## Explicit exclusions

Contact-island or body sleeping is not implemented. Stable active contacts continue to be
solved; this refactor changes ownership and work-set selection, not sleep/wake semantics.
