# Known contact-pipeline debt

## Completed architecture migration

- Runtime code moved from the historical `Jobs/ContactPipeline` tree to explicit
  Contracts, State, Kernels, Scheduling, Stages and Observability owners.
- Flat persistent/frame resource owners moved under `State/Persistent` and
  `State/Frame`; Unity `.meta` GUIDs were preserved for moved assets.
- Body and persistent interaction contracts were split from aggregate source files.
- The four stage ABIs were split into independent job files.
- Serial and parallel lifecycle ABIs are separate, and every direct stage
  NativeContainer capability is constructed before scheduling; inactive enum
  branches no longer rely on invalid `default` containers.
- Every `InteractionCertificationJob`, `MotionIntegrationJob`, `SoftAvoidanceJob`
  and `ConstraintSolverJob` fragment now lives below its owning stage.
- `ActiveConstraintIncidentIndex` was split into a shared kernel plus certification
  and solver adapters.
- Parallel executable jobs were extracted from the scheduler partial into
  `Scheduling/Parallel/Jobs`; scheduler files now contain construction/order only.
- Runtime observation contracts are data-only and isolated from presentation,
  recording and control. Solver capture fragments are compile-gated.
- Spatial debugger readback is a standalone observer with explicit EntityManager,
  diagnostics entity and proxy-view inputs; it no longer reaches through a system
  partial to candidate storage.
- Certificate flags are derived from committed structure, mapping, configuration,
  topology and classification evidence. Serial and parallel consumers fail closed
  before SoftAvoidance and Solver when the certificate scope/view counts mismatch.
- Diagnostics-on and diagnostics-off script compilation are both permanent
  validation configurations.
- InteractionCertificationJob gameplay-critical helper logic extracted into
  pure static kernel classes under `Kernels/` (PersistentProxyBuilder,
  PersistentContactMath, PersistentStoreLookup, IncrementalDirtyBodyStore,
  PersistentCacheReusability, PredictiveContactScheduler,
  DirtyIncidentPairMapper). The certifier job keeps its field layout (Burst/
  Collections Safety requires NativeContainers as direct struct fields); each
  partial method is now a one-line forwarder and the kernel signatures expose
  only the containers each algorithm consumes.
- Diagnostics field declarations and telemetry load/store helpers for
  ConstraintSolverJob, InteractionCertificationJob and SoftAvoidanceJob moved
  into sibling `*.Diagnostics.cs` partials so the struct bodies stay free of
  `#if RTS_CONTACT_DIAGNOSTICS` field-block noise.

## Remaining runtime evidence, not structural migration

The following require Unity runtime execution and are not proven by source layout or
batch script compilation:

- GS/Jacobi trajectory and pair-set differential tests;
- full-sweep versus persistent incremental oracle runs;
- Burst Inspector confirmation for all newly relocated jobs;
- Collections Safety under early return, repair and fallback paths;
- TempJob disposal coverage for exceptions/domain reload;
- deterministic replay hashes and multi-World isolation;
- performance/allocation benchmarks before and after the migration.

## Deliberate exclusions

- Contact islands and sleeping require a separate island/wake design.
- `ReciprocalVelocityObstacle` remains a compatibility enum name; the current
  implementation is not ORCA/RVO2 linear programming.
- Public namespace renaming is excluded to avoid mixing an API migration with the
  physical ownership migration.
- The certifier's orchestration methods (`BuildTimestepPredictiveSchedule` shell,
  `ClassifyOrReusePersistentNeighborPairs`, `ClassifyPersistentNeighborPair`,
  `FullRebuildPersistentNeighborTopology`, `ClassifyAndPatchDirtyIncidentContacts`,
  `IncrementallyRepairPersistentNeighborTopology`,
  `TryIncrementallyRepairEscapedContactSet`, `TryReusePersistentContactViews`)
  stay in the `InteractionCertificationJob` partial. Each coordinates 8-15
  NativeContainers plus sibling instance methods; moving them into a kernel
  would require passing that field set as parameters, making the data flow
  harder to read than the current `this.`-scoped access. The extracted kernels
  cover the pure-algorithm and lookup surface; these orchestrators are the
  remaining body by design.
