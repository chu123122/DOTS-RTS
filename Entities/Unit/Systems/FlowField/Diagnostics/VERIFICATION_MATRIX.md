# Contact Pipeline Verification Matrix

This matrix is the runtime acceptance contract for the diagnostics refactor.
The repository CI executes source-level contracts for completed-step identity,
per-World routing and gameplay-only telemetry elimination. Rows marked
**Unity required** have not been executed and must be run in Editor or a
licensed batchmode runner before PR #16 is mergeable.

## Build matrix

| Diagnostics | Burst | Safety | Solver | Status |
|---|---:|---:|---|---|
| Off | Off | On | Gauss-Seidel | Unity required |
| Off | Off | On | Jacobi | Unity required |
| Off | On | On | Jacobi | Unity required |
| Off | On | Off | Jacobi | Unity required |
| On | Off | On | Jacobi | Unity required |
| On | On | On | Jacobi | Unity required |
| On | On | Off | Jacobi | Unity required |

Run each configuration with 0, 1, 2, 100 and the project stress-scale unit count.

## Required invariants

1. Diagnostics Off/On produces the same pair keys, lifecycle, cache validity,
   repair, fallback, positions and velocities within the documented tolerance.
2. Gameplay-only creates no diagnostic entity, buffer, TempJob allocation,
   publisher job, profiler clock read, Oracle pass or capture loop.
3. A published snapshot contains one SimulationStepId; frame and pipeline
   data are immutable after publication.
4. Old snapshot references remain unchanged across later publishes and Reset.
5. World A and World B may use different timestep-cache settings.
6. All TempJob containers are disposed after their final reader and within the
   Unity lifetime limit.

## Static contract status

The following are enforced by repository CI:

- frame and pipeline publication use one captured completed-step identity;
- duplicate or backwards timestep publication is rejected per World;
- debugger, experiment overrides and latest snapshots are keyed by World id;
- gameplay preprocessing contains no Jacobi iteration/block telemetry types,
  allocations or pure telemetry reduction/finalize jobs;
- persistent classification timestamps do not occupy gameplay struct storage;
- this document has a committed Unity `.meta` file;
- persistent and frame-local contact resources have separate lifetime owners;
- dirty-body block offsets are not reused as soft incident cursors;
- contact-view source resolution, classification and commit are explicit stages;
- persistent classification phase state is separate from diagnostics telemetry;
- normal and diagnostics Jacobi jobs share one pair-evaluation implementation;
- legacy Fat-AABB configuration names survive only as serialization migration attributes;
- solver heat samples are aggregated into published cell heatmaps and guarded proxies;
- contact-set diagnostics preserve full-rebuild and fallback-added-pair counts separately;
- each GUI pass consumes Frame and Pipeline from one published snapshot generation;
- effective settings and recorder output include `TimestepContactMargin`;
- Oracle warnings describe observation-only behavior.

These checks are not substitutes for the Unity-required matrix above.
