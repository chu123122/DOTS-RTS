# Contact Pipeline Verification Matrix

This matrix is the runtime acceptance contract for the diagnostics refactor.
The repository CI currently executes source-level contracts; rows marked
**Unity required** must be run in Editor or a licensed batchmode runner.

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
