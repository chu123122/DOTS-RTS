# Crowd contact-pipeline architecture

## Domain boundary

This subsystem is not a general rigid-body `PhysicsWorld`. It is a data-oriented
crowd-motion pipeline for flow-field navigation, pairwise soft avoidance,
predictive disc contacts and XPBD position projection.

```text
ECS authoritative body state
        ↓
Navigation intent
        ↓
Preliminary motion evidence
        ↓
Candidate persistent interaction state
        ↓
Interaction correctness certifier
        ↓
Certified compact consumer views
        ↓
Soft velocity correction + motion integration
        ↓
Wall / XPBD constraint projection
        ↓
Velocity reconstruction and ECS writeback
```

## Candidate → certifier → certified product

Persistent containers are reusable **candidate source data**, not direct solver
inputs:

```text
PersistentSweptProxies
PersistentNeighborPairs
PersistentPredictiveContacts
PersistentActiveContactKeys
PersistentSoftAvoidancePairKeys
PersistentDormantContactSchedule
IncrementalContactCacheState
```

The certifier combines those candidates with current authoritative evidence:

```text
current body/entity set
current trajectory and guard bounds
current radius and mapping
same-step configuration fingerprint
base-motion / predicted-position / solver-correction escape evidence
```

It then chooses exactly one path:

```text
accept existing candidate
incrementally repair candidate
authoritative full rebuild
```

The path-selection threshold, including the dirty-body ratio, does not prove
correctness. A certificate is issued only after structural checks, mapping,
topology coverage, classification and compact-view commit have completed.

The consumer-visible product is logically one bundle but remains physically
compact:

```text
InteractionCertificate
Certified SoftAvoidancePairs
Certified TimestepContactPairs
Certified PredictiveContactSchedule
optional materialized TimestepInteractionPairs
```

A clean persistent path is not required to materialize one universal pair array.
Lower stages must not branch on `SourceMode`; provenance exists for audit and
telemetry only.

## Certificate scope

`InteractionCertificate` binds the product to:

- `WorldId`;
- scheduled `SimulationStepId`;
- body-set fingerprint;
- exact configuration fingerprint;
- topology and classification revisions;
- substep interval and horizon duration.

"Unconditionally trusted" means trusted only inside that explicit scope. A lower
stage may report an `InteractionCertificateViolation`, but it cannot edit
persistent candidate state. The certifier remains the sole accept/repair/rebuild
owner and reissues a certificate after recovery.

The shared `CommitTimestepContactViews` function is the certification commit
boundary for both the serial reference implementation and the staged P1-P6
Jacobi implementation.

## Layer ownership

- **ECS adapter / composition root** captures same-step configuration and identity,
  schedules stages, owns World-scoped resources and wires `JobHandle` dependencies.
  Detailed solver ABI expansion lives in `BaseFlowMovementComposition`, not in
  `BaseFlowMovementSystem.OnUpdate`.
- **Navigation** reads `FlowNavigationView` and produces preferred velocity and
  steering intent. It does not interpret contact or wall policy.
- **Motion prediction** produces trajectory/envelope evidence and substep positions.
- **Persistent candidate store** owns stable entity-pair topology, proxy guard
  bounds, predictive lifecycle and reusable indexes. Only the certifier may mutate it.
- **Prediction / certifier** owns structural validation, dirty classification,
  incremental repair, authoritative rebuild, compact-view commit and certificate
  issue/revocation.
- **SoftAvoidance** consumes only the certified soft-pair view and produces a
  velocity correction. It does not own predicted positions or candidate cache.
- **Motion integration** combines steering and soft correction and produces the
  unconstrained predicted position.
- **Constraint assembly / Solver** consumes frame-local certified definitions.
  Gauss–Seidel remains the serial reference; Jacobi evaluates pairs in parallel and
  deterministically gathers body corrections through a frame-local CSR index.
- **Environment** is exposed through `GridObstacleView`. Navigation and collision
  may currently share the same cell storage, but they no longer share semantics.
- **Diagnostics** observes completed immutable publication. Oracle results cannot
  invalidate or rebuild gameplay state.
- **Control plane** supplies next-step debugger/experiment commands at the
  composition boundary. It is separate from observation even where legacy files
  remain under the Diagnostics directory.

## Data contracts

`retired shared frame state` is being migrated without changing the solver math. It
already physically composes:

```text
CrowdBodySnapshot
CrowdNavigationState
CrowdMotionIntent
```

Timestep/substep vector fields remain temporarily flat because existing Burst jobs
pass several fields by `ref`/`out`. New code must use the explicit contracts and
must not add another unrelated field to the compatibility blackboard.

`UnitCollisionPair` storage is split by lifetime:

```text
ContactConstraintDefinition   immutable-by-convention solver definition
ContactConstraintRuntime      substep/iteration lambda and activation
ContactConstraintHistory      timestep utilization/provenance
```

Compatibility properties preserve current call sites while narrower solver ABI
migration proceeds.

## Environment views

The backing `NativeArray<FlowFieldCell>` may be shared, but callers use semantic
accessors:

```text
FlowNavigationView
    IsReachable / direction / integration semantics

GridObstacleView
    IsBlocked / cell geometry semantics
```

Soft wall avoidance, motion escape and hard wall projection must not directly
interpret `FlowFieldCell.Cost`. This permits later separation of high-cost
traversable terrain, physical-only regions, clearance fields, arbitrary static
geometry and dynamic obstacles without redesigning navigation intent.

## Runtime state versus telemetry

`IncrementalContactCacheState` and persistent containers are candidate gameplay
state used by the certifier. `IncrementalContactPipelineStatistics` is write-only
observation telemetry and cannot drive cache validity, repair, activation or
fallback.

Scheduled simulation identity and persistent-cache age are distinct:

```text
SimulationStepId    assigned by the World composition root
CacheGeneration     derived from persistent candidate maintenance age
```

Diagnostics publication must never reconstruct `SimulationStepId` from cache age.
This also keeps A0/non-persistent configurations publishable.

Oracle mismatch is observation-only. It records missing/extra pairs for validation
but never writes `IncrementalCacheState`, `IsValid`, topology or certified views.

## Lifetimes

| Data | Lifetime | Owner | Identity |
|---|---|---|---|
| Body snapshot / navigation / intent | One scheduled timestep | ECS adapter / navigation | World + step + BodyIndex mapping |
| Persistent proxy / neighbor / predictive lifecycle | Cross timestep candidate | Certifier candidate store | Stable Entity pair + topology revision |
| Interaction certificate and compact views | Certificate scope | Certifier, immutable after issue | World + step + body/config/topology/classification scope |
| Predicted position / velocity correction | One substep | Motion / constraint step | BodyIndex |
| Lambda / activation runtime | One substep or iteration | Solver | Frame-local constraint index |
| Constraint history | One timestep | Assembly / solver | Frame-local pair |
| Telemetry | One completed timestep | Diagnostics capture | World + scheduled step |
| Published snapshot | Cross-frame immutable observation | Publisher | World + generation + scheduled step |

## Correctness invariants

1. Every possible interaction lies in a certified compact view or touches pending
   violation evidence that is repaired before the affected view is consumed.
2. Persistent candidates are never direct SoftAvoidance, Motion or Solver inputs.
3. Entity mapping, exact configuration and guard containment are hard certification
   evidence; dirty ratio only selects repair versus rebuild.
4. Predictive normal orientation is converted to current BodyIndex order once for
   the timestep definition.
5. Soft output cannot exceed its certified interaction envelope; the preserved
   correction is clamped to the proven scope.
6. Predicted positions and solver corrections report distinct violation reasons.
7. Failed incremental proof converges on the authoritative full-sweep fallback.
8. Jacobi pair evaluation reads one immutable position snapshot and gathers in
   deterministic incident-pair order without floating-point atomics.
9. `SourceMode` cannot change gameplay consumer behavior.
10. Diagnostics/Oracle can observe but cannot mutate candidate or certified state.
11. Diagnostics-off builds must not add gameplay Native containers, jobs or
    profiler reads.
12. Position corrections are converted back to velocity only by the reconstruction
    stage.

## Composition root

`BaseFlowMovementSystem` is intentionally retained as the World composition root.
It knows stage order, resource lifetime, configuration snapshots and JobHandle
edges. It must not know pair classification, guard proof, repair policy, XPBD
lambda math, CSR construction or heatmap aggregation.

`BaseFlowMovementComposition` is a temporary ABI adapter. New stage contracts
should shrink that adapter until candidate state is visible only to the certifier.

## CI boundary

`.github/scripts/validate_contact_architecture.py` statically protects the current
migration:

- explicit body/certificate/pair-lifetime contracts must exist;
- scheduled step identity cannot be derived from cache generation;
- compact views must be signed at their common commit boundary;
- the composition root cannot expand the solver ABI directly;
- SoftAvoidance, Motion and Wall stages cannot reach persistent candidate fields;
- serial environment stages cannot interpret navigation cost directly;
- Oracle cannot control gameplay cache.

This CI does not replace Unity Editor compilation, Burst compilation, Collections
Safety, replay-hash, differential pair-set or runtime performance verification.

## Explicit exclusions

Contact islands and sleeping are not implemented. They require a separate
island/wake design and evidence. This architecture refactor changes ownership and
capabilities, not sleep/wake semantics.
