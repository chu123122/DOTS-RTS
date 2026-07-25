# Known contact-pipeline debt

## Completed hard-cutover architecture work

The phase 1-5 migrations are terminal cutovers: no forwarding frame/pair wrapper,
all-capability solver type, aggregate resource bag or retired compatibility directory
remains in the production tree.

- Scheduled `SimulationStepId` is independent from persistent `CacheGeneration`.
- Persistent containers are documented and guarded as certifier-owned candidate state.
- `InteractionCertificate`, explicit certification evidence, violation evidence and
  compact-view commit signing exist.
- Serial base-motion, predicted-position and solver-correction escapes revoke and
  reissue certificates through the certifier path.
- The staged P1-P6 path shares the same compact-view commit/signing boundary and
  routes solver-correction escapes through the common certificate guard.
- `BaseFlowMovementSystem` is the World composition root. Stage construction is
  distributed across focused lifetime owners rather than an ABI adapter.
- The shared frame-state compatibility record has been deleted. Body snapshot,
  navigation, intent, motion evidence, substep state and result are independent arrays.
- Pure `BodyPair` interaction views are physically separated from direct-storage `ContactConstraint` solver records.
- `ContactConstraint` is the only public solver constraint contract; evaluation records
  and their math helper are solver-internal implementation details.
- Serial and P1-P6 navigation/soft-wall/hard-wall code use separate navigation and
  obstacle semantics over the current shared grid storage.
- The all-capability `SolveXpbdUnitContactsJob` and `CrowdEnvironmentAccess` partial
  have been deleted. Certification, motion, soft avoidance and constraint solving are
  scheduled as focused jobs; the scheduler itself is never scheduled.
- The aggregate `BaseFlowMovementComposition` and `ContactPipelineResources` files
  have been deleted. Persistent candidates, body products, certification products,
  soft scratch, solver scratch and execution state have independent owners.
- The `Jobs/Compatibility` type graveyard has been deleted. Its Adaptive Fat-AABB
  records had no production or serialization references and were not migrated.
- Architecture contracts are protected by a dedicated static CI workflow.

## Remaining capability-boundary debt
- P1-P6 jobs still carry raw grid storage fields because Unity job safety requires
  Native containers to remain direct job fields. Their interpretation is now routed
  through `GridObstacleView`; a future backend split may change storage without
  changing stage semantics.
- Parallel escape jobs preserve correctness through the existing repair scheduler,
  but explicit per-body `InteractionCertificateViolation` records are currently
  richest on the serial path. Parallel compaction should eventually emit the same
  reason-coded evidence without concurrent list writes.
- `CertifiedInteractionProductDescriptor` is currently a container-free scope
  descriptor. Consumer-specific read-only certified views should replace the final
  direct `NativeList` fields passed to focused stages.

## Performance debt

- Gauss–Seidel contact projection remains serial. The Jacobi mode has a parallel
  pair-evaluate/body-gather path; parallel Gauss–Seidel would require graph coloring
  or conflict-free batches.
- Soft Avoidance uses pair-evaluate/body-gather. Its frame-local CSR remains
  separate from the active-contact CSR.
- Active BodyIndex→constraint CSR and cross-timestep Entity→persistent-pair indexes
  are versioned and reused, but deterministic compaction, sorting and persistent-
  view publication remain serial coordination points.
- Local topology repair queries a versioned persistent cell→proxy membership view.
  Capacity failure safely falls back to the authoritative O(D×N) scan; production
  sizing still needs runtime evidence.
- Persistent pair classification uses serial prepare, parallel pair evaluation and
  deterministic serial commit. Commit cost can dominate when most pairs change.
- Clean persistent paths still remap and sort compact stable-key views. Do not force
  materialization of one universal InteractionSet merely for architectural symmetry.

## Terminology debt

- Serialized `UnitMovementSettings.MaxForce` is still an authoring-schema name.
  Runtime products use `MaxAcceleration` and `SteeringVelocityError`; asset-schema
  renaming should use `FormerlySerializedAs` in a separate authoring migration.
- `ReciprocalVelocityObstacle` is a compatibility enum name. The implemented math
  is pairwise reciprocal closest-approach velocity correction, not ORCA/RVO2 linear
  programming and not acceleration-velocity-obstacle set construction.
- `ClassificationEpoch` currently behaves partly as a configuration fingerprint.
  A later schema migration should separate exact fingerprint from monotonic revision.

## Diagnostics and control-plane debt

- Observation publication is immutable and per World, and Oracle is observation-
  only. Debugger/experiment commands still live under the historical Diagnostics
  directory; naming should eventually separate Control and Observation folders.
- Solver signatures still thread telemetry references through production stages.
  Diagnostics-off preprocessing removes storage/work, but narrower stage ABIs should
  remove those parameters entirely.

## Required external verification

The lightweight workflows do not execute:

- Unity Editor compilation;
- Burst compilation;
- Collections Safety;
- GS/Jacobi runtime differential tests;
- full-sweep versus incremental pair-set oracle tests;
- replay hashes;
- multi-World runtime isolation;
- TempJob lifetime under all early-return/fallback paths;
- performance and allocation benchmarks.

The PR must remain Draft until those Unity-required checks are executed.

## Scope decision

Sleeping is deliberately excluded. It requires a separate island/wake design and
evidence.
