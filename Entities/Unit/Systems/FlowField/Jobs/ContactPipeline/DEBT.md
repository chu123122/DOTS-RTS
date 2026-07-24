# Known contact-pipeline debt

## Completed architecture boundary work

- Scheduled `SimulationStepId` is independent from persistent `CacheGeneration`.
- Persistent containers are documented and guarded as certifier-owned candidate state.
- `InteractionCertificate`, violation evidence and compact-view commit signing exist.
- Serial base-motion, predicted-position and solver-correction escapes revoke and
  reissue certificates through the certifier path.
- `BaseFlowMovementSystem` is a small composition root; historical solver ABI
  expansion is isolated in `BaseFlowMovementComposition`.
- Body, navigation and motion-intent data are physically composed from explicit
  contracts inside the compatibility frame state.
- Contact definition, runtime state and timestep history are physically separated.
- Navigation and obstacle semantics use different views over the current shared grid.
- Architecture contracts are protected by a dedicated static CI workflow.

## Remaining capability-boundary debt

- `SolveXpbdUnitContactsJob` remains a historical compatibility super-struct. Its
  partial methods can still see candidate, certified, motion, solver and diagnostics
  fields even though lower-stage source files no longer use candidate state.
- `BaseFlowMovementComposition` still expands individual Native containers into
  that compatibility ABI. It should shrink into certifier, soft, motion, assembly
  and solver-specific contexts without adding synchronization points.
- Timestep/substep vectors remain flat in `FlowMovementFrameState` because several
  existing Burst jobs pass them through `ref`/`out`. They should migrate into
  `CrowdMotionEvidence` and `CrowdBodyStepState` in a dedicated differential-tested
  batch.
- The staged P1-P6 body jobs still carry raw grid fields internally. Their wall and
  base-motion access should be migrated to the same container-free obstacle helpers
  used by the serial reference path.
- Parallel escape jobs preserve correctness through the existing repair scheduler,
  but explicit per-body `InteractionCertificateViolation` records are currently
  richest on the serial path. Parallel compaction should eventually emit the same
  reason-coded evidence without concurrent list writes.
- `CertifiedInteractionProductDescriptor` is currently a container-free scope
  descriptor. Consumer-specific read-only context types should replace direct
  NativeList fields in the solver ABI.

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

- `IndependentForce` and serialized `MaxForce` remain compatibility names. Runtime
  semantics are preferred velocity plus steering velocity error integrated under a
  velocity-change-rate cap; authoring migration should use `FormerlySerializedAs`
  before final renaming.
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
