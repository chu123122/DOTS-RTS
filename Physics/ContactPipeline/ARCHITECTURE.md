# Crowd contact-pipeline architecture

## Domain boundary

This subsystem is not a general rigid-body `PhysicsWorld`. It is a data-oriented
crowd-motion pipeline for flow-field navigation, pairwise soft avoidance,
predictive disc contacts and XPBD position projection.

```text
ECS authoritative body state
        ↓
CrowdPhysicsStepInput
        ↓
BroadPhaseCandidateBatch
        ↓
NarrowPhaseConstraintBatch
        ↓
CrowdMotionSolver
  SoftAvoidance → Integrate → XPBD → Velocity reconstruction
        ↓
CrowdPhysicsStepOutput → ECS writeback
```

The runtime remains in one ECS World. `CrowdPhysicsSystemGroup` provides a
separate scheduling phase; no second ECS World or second Unity `PhysicsWorld`
is created.

## Deletion-first convergence contract

The current migration is completed by removing structures that violate the
target data flow before introducing replacement abstractions. Compatibility
facades, forwarding properties and duplicate writable representations are not
retained merely to keep old call sites compiling.

The target dependency graph is:

```text
Gameplay adapter
    ↓ immutable CrowdPhysicsStepInput
Physics composition API
    ↓
BroadPhase stage
    ↓ immutable candidate view
NarrowPhase stages
    ├─ InitialContact OR PersistentClassification
    ├─ SubstepRepair (only when requested by explicit violation evidence)
    └─ Certificate
    ↓ immutable certified constraint views
Solver stages
    ├─ shared pre-solve stages
    ├─ XPBD backend branch: GS IJob OR Jacobi IJobParallelFor
    └─ shared IterationFinalize / reconstruction
    ↓ immutable CrowdPhysicsStepOutput
Gameplay writeback
```

`IterationFinalize` may publish a repair request back to the scheduler. That is
a new scheduling decision, not permission for Solver code to mutate
NarrowPhase or persistent storage. Stage implementation namespaces do not call
one another: each stage may depend only on Contracts and neutral stateless
Kernels. The scheduler owns all stage ordering.

The following structures are forbidden:

1. two writable containers holding complete copies of the same authoritative
   record;
2. a resource or context struct that grants a Job unrelated stage capabilities;
3. a shared body-state blackboard written by Navigation, NarrowPhase, Solver and
   reconstruction;
4. calls from one concrete Stage data-flow type into another concrete Stage;
5. a cross-stage helper file that accumulates unrelated classification, repair,
   scheduling, certificate and diagnostics algorithms;
6. assembly-wide access to Physics internals when a narrow runtime lease or
   observation contract is sufficient.

Persistent predictive contacts therefore use exactly one authoritative value
store:

```text
NativeList<PersistentPredictiveContact>     authoritative records
NativeParallelHashMap<StableEntityPairKey, int> derived key → list-index lookup
```

The index never stores `PersistentPredictiveContact`. Any list replacement or
compaction rebuilds the derived index before publication; every point update
writes only the list entry addressed by the index.

Specific Jobs carry their exact `NativeArray`/`NativeList` fields. Managed
composition code may use short-lived construction helpers, but
`Certification*Resources`, cache owners or other aggregate capability bags
must not be embedded in scheduled Jobs.

Deletion order and completion checks:

| Order | Delete first | Minimal replacement | Completion check |
|---|---|---|---|
| 1 | duplicate persistent contact values | authoritative list + key/index map | no hashmap stores `PersistentPredictiveContact`; all mutations target the list |
| 2 | aggregate Certification bags in Jobs | exact Job fields | static scan finds no resource-bag Job field |
| 3 | Stage-to-Stage calls and giant helper | neutral single-purpose Kernels | Stage dependency graph is acyclic |
| 4 | shared `StepState/Evidence` write blackboard | NarrowPhase evidence + Solver runtime products | each field has one owning stage |
| 5 | residual serial repair/classification algorithms | Defer Count/Prefix/Scatter plus explicit publication/finalize jobs | no aggregate commit re-evaluates, copies or sorts whole candidate sets |
| 6 | Gameplay friend access and exposed Physics resource owners | `CrowdPhysicsRuntime` + timestep/diagnostics leases | no Gameplay IVT; Gameplay cannot name cache/stage owners |

No row is considered complete from source layout alone. It requires ordinary
and Diagnostics compilation plus the relevant static contract; Unity
Editor/Burst/Collections Safety/Play Mode and Profiler evidence remain separate
runtime acceptance levels.

## Candidate → certifier → certified product

Persistent containers are reusable **candidate source data**, not direct solver
inputs:

```text
PersistentSweptProxies
PersistentNeighborPairs
PersistentPredictiveContacts
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
telemetry only. The scheduler may only branch on certificate validity and fails
closed before a consumer stage when scope or committed-view counts mismatch.

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

`PrepareClassificationPublication → Materialize → Count → Prefix → Scatter`
is the shared compact-view publication boundary. Initial classification and
substep repair then use separate, bounded state/oracle/certificate jobs.
`ValidateConsumerViews` is the single scheduling gate for both solver backends.

## Layer ownership

- **Gameplay adapter** translates FlowField/arrival/collider authoring data into
  `CrowdPhysicsBodyInput`, `CrowdObstacleSnapshot` and frozen
  `CrowdPhysicsSettings`. It invokes only `ScheduleStep`.
- **BroadPhase** owns proxy bounds, candidate generation and its `CrossFrameCache`
  partition. It does not classify contacts or own lambda.
- **NarrowPhase** owns exact interaction classification, cache proof/repair and
  production of soft/hard constraint views. Certification is an internal proof
  mechanism, not a fourth public layer.
- **CrowdMotionSolver** consumes the narrow-phase product in fixed order:
  SoftAvoidance, integration, wall/XPBD projection and velocity reconstruction.
  XPBD mutates only `ContactConstraintRuntime`.
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
  Both backends reuse the same parallel prediction, certification, soft-avoidance,
  wall, repair and velocity-reconstruction stages. Only XPBD contact projection
  branches: Gauss–Seidel uses one ordered `IJob`; Jacobi evaluates pairs and
  gathers body corrections in parallel through a frame-local CSR index.
- **Environment** is exposed through a versioned `CrowdObstacleSnapshot`.
  Navigation `FlowFieldCell` storage is translated at the adapter boundary and
  is never read by the contact pipeline.
- **Diagnostics** observes completed immutable publication. Oracle results cannot
  invalidate or rebuild gameplay state.
- **Control plane** supplies next-step debugger/experiment commands at the
  composition boundary. It is separate from observation even where legacy files
  remain under the Diagnostics directory. Per-World `SimulationDebuggerRuntime`
  owns unit selection; `ContactDiagnosticSelection` is an optional ECS control
  input and is never attached to a per-system publication entity.

## Data contracts

Body data is stored as independent timestep products rather than a shared frame
blackboard:

```text
CrowdPhysicsStepInput       Unit/Navigation -> Physics
BroadPhaseCandidateBatch   BroadPhase -> NarrowPhase
NarrowPhaseConstraintBatch NarrowPhase -> CrowdMotionSolver
CrowdPhysicsStepOutput      Solver -> Unit writeback
```

The implementation may use SoA arrays internally. Those arrays are hidden inside
one of two lifetime owners:

```text
CrossFrameCache  World lifetime; guarded Broad/Narrow reusable state
TimestepCache   one physics timestep; may span substeps
```

`ContactConstraintDefinition` contains immutable endpoints/mode/normal.
`ContactConstraintRuntime` contains lambda, normal orientation and activation
history. The combined record lives only in `TimestepCache`.

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

`BaseFlowMovementSystem` is the Gameplay adapter, not the physics composition
root. It collects navigation input and an immutable obstacle snapshot, then
uses the public `CrowdPhysicsRuntime`. Gameplay can create a
`CrowdPhysicsStep`, write only `InputBodies`, schedule it, read only
`OutputBodies`, publish through a `CrowdPhysicsDiagnosticsStep`, and release
both leases. It cannot name or access `CrossFrameCache`, `TimestepCache`,
frame-resource owners, certification Jobs or Solver arrays.

`CrowdContactPipelineScheduler` is managed scheduling composition only; it is not
a scheduled job and carries no algorithm implementation. `ScheduleParallelStages`
is the only runtime pipeline entry. Executable parallel jobs live under
`Scheduling/Parallel/Jobs`. Certification coordination is split into explicit
stage Jobs with exact container fields. A concrete Stage may call its own
single-purpose DataFlow, but may not call another concrete Stage DataFlow;
shared operations live in neutral Kernels. No aggregate certification kernel or
Certification resource bag is constructed. `SoftAvoidanceJob` and
`ConstraintSolverJob` retain focused stage boundaries, so Collections Safety
sees the NativeContainer capabilities of each scheduled unit.
`CrowdPhysicsPipelineComposition` is the single managed composition adapter.
It is internal to `RTS.Physics`; only `CrowdPhysicsRuntime.ScheduleStep` reaches
it. The runtime owns the World-lifetime `CrossFrameCache`; each
`CrowdPhysicsStep` owns one `TimestepCache`. Specific Jobs carry direct
`NativeArray`/`NativeList` fields so Unity Collections Safety sees the real
access graph; product/cache wrappers are never embedded as Job fields.

The one-timestep owner is split by write responsibility rather than by historical
Certification naming:

```text
BroadPhaseFrameResources          candidate discovery worksets
ContactProductFrameResources      certified soft/hard consumer products
ContactClassificationFrameResources classification results/publication blocks
ContactRepairFrameResources       dirty/repair/incident rebuild worksets
ContactCertificateFrameResources  certificate, schedule and contact scratch
```

Initial classification finalization is
`PublishPersistentClassificationState → ValidatePersistentClassificationOracles
→ FinalizePersistentClassificationCertificate`. Repair finalization is
`PublishSubstepRepairClassification → MergeRepairedContactView
→ ClearRepairedEnvelopeEscape → Prepare/Scatter/FinalizePersistentIncidentLookup
→ FinalizeSubstepRepairCertificate`. The two former aggregate commit jobs are
retired symbols.

## Execution topology

The scheduler does not maintain separate GS and Jacobi pipelines:

```text
parallel prediction / certification / soft avoidance / motion / wall
                              ↓
                    XPBD backend branch
                    ├─ GS: ordered IJob
                    └─ Jacobi: pair IJobParallelFor
                               + block reduction
                               + body IJobParallelFor
                              ↓
shared validation / repair / velocity reconstruction / publication
```

Recovery follows the same backend rule. GS recovery remains ordered; Jacobi
recovery reuses parallel pair evaluation and body gather. The retired serial
lifecycle, certification, motion, soft-avoidance and scheduler path must not
return.

## CI boundary

`.github/scripts/validate_contact_architecture.py` statically protects the current
migration:

- explicit body/certificate/pair-lifetime contracts must exist;
- Gameplay friend access and all internal resource-owner leaks must remain deleted;
- `CrowdPhysicsRuntime` must remain the only Gameplay scheduling boundary;
- scheduled step identity cannot be derived from cache generation;
- compact views must be signed at their common commit boundary;
- the retired all-capability solver type and environment-access partial cannot return;
- aggregate composition/resource bags, flat resource owners and the historical
  `Jobs/ContactPipeline` root cannot return;
- the scheduling composition cannot become another `IJob`;
- staged broad-phase sorting may not return to `SortJobDefer`;
- substep-repair bulk copies must remain deferred parallel jobs;
- SoftAvoidance, Motion and Wall stages cannot reach persistent candidate fields;
- environment stages cannot interpret navigation cost directly;
- Oracle cannot control gameplay cache.

This CI does not replace Unity Editor compilation, Burst compilation, Collections
Safety, replay-hash, differential pair-set or runtime performance verification.

## Explicit exclusions

Contact islands and sleeping are not implemented. They require a separate
island/wake design and evidence. This architecture refactor changes ownership and
capabilities, not sleep/wake semantics.
