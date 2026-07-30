# Known contact-pipeline debt

## Completed structural migration

- `InteractionCertificationAlgorithms`, `CertificationStageKernel`, all
  `Certification*Resources`, forwarding properties and the old Algorithms files
  are deleted.
- Scheduler composition uses the real owners directly:
  obstacle snapshot, body state, broad-phase candidates, narrow-phase constraints,
  persistent cache, solver state, execution state and diagnostics.
- Persistent predictive contacts have one authoritative
  `NativeList<PersistentPredictiveContact>` plus a derived
  `StableEntityPairKey -> list index` lookup.
- Dirty-body refresh is `IJobParallelForDefer -> Reduce`.
- Dirty contact/schedule is `Count -> Prefix -> Scatter`.
- Full sweep is body-cell `Count/Prefix/Scatter`, cell-pair
  `Count/Prefix/Scatter`, staged parallel block sort/merge, then deduplicate.
  No `SortJobDefer` remains.
- Substep repair preparation only sizes buffers. Interaction-pair and previous
  contact-pair copies are separate `IJobParallelForDefer` jobs.
- Classification publication is `Prepare -> Materialize -> Count -> Prefix ->
  Scatter`; persistent contact index construction is parallel.
- The aggregate `CommitPersistentClassificationJob` and
  `CommitSubstepRepairJob` are deleted. Initial and repair paths publish state,
  rebuild compact views, rebuild incident lookup and issue certificates in named
  Stage jobs. Repair and activation views use candidate materialization, staged
  block sort/merge, then Count/Prefix/Scatter publication.
- InitialContact, SubstepRepair, PersistentClassification, Certificate and
  IterationFinalize no longer call another concrete Stage DataFlow. Shared
  operations live in neutral, single-purpose Kernels.
- The old `CrowdBodyStepState` name and avoidance blackboard are deleted.
  `CrowdSolverBodyState`, `CrowdAvoidanceState` and
  `CrowdMotionEvidence` have distinct responsibilities.
- GS and Jacobi share prediction, certification, repair, soft avoidance, wall,
  iteration-finalize and reconstruction stages. Only XPBD contact projection
  branches into ordered GS and parallel Jacobi.
- `InternalsVisibleTo("RTS.Gameplay")` is deleted. Gameplay uses
  `CrowdPhysicsRuntime`, `CrowdPhysicsStep` and
  `CrowdPhysicsDiagnosticsStep`; it cannot own `CrossFrameCache`,
  `TimestepCache`, frame resources or executable solver Jobs.
- Runtime code is compiled in `RTS.Physics`; game rules and ECS adapters in
  `RTS.Gameplay`; NetCode in `RTS.Network`; UI/Input remain the terminal default
  assembly because they depend on default-assembly `QFramework` and generated
  `PlayerAction` sources.

## Remaining runtime evidence

The latest source has completed a fresh Unity Editor import, Entities/Jobs/Burst
IL post-processing and the built-in local gameplay validation. The following
still require stronger runtime evidence:

- focused Collections Safety coverage for staged block merge, deferred repair
  copies and exception/disposal paths;
- representative GS/Jacobi trajectory and pair-set differential captures beyond
  the small built-in equivalence scenario;
- representative full-sweep versus persistent reuse/repair oracle captures;
- deterministic replay hashes and multi-World isolation;
- representative 5k-unit Profiler captures. In particular, source-level removal
  of `SortJobDefer` and serial `PrepareSubstepRepair` does not by itself prove a
  frame-time improvement.

## Deliberate boundaries

- `RTS.Physics.Editor` retains friend access for editor-only validators. Runtime
  Gameplay has no friend access.
- UI/Input/scene composition remains in default `Assembly-CSharp`. Adding
  `RTS.UI.asmdef` before moving or wrapping `Assets/Qframework/QFramework.cs`
  and `Assets/Resources/PlayerAction.cs` would not compile.
- Certification may read completed solver state when processing an explicit
  repair request, but does not mutate solver state. This is a scheduler-ordered
  feedback transition, not a shared multi-writer blackboard.
- Contact islands and sleeping require a separate island/wake design.
- `ReciprocalVelocityObstacle` remains a compatibility enum name; the current
  implementation is not ORCA/RVO2 linear programming.
- Public namespace renaming is excluded from this migration.

## Completed: repair / activation serial P1（2026-07-30）

- 删除 `MergeRepairedContactViewJob` 和
  `TimestepContactRepairViewKernel.MergeEscapedTimestepContactView`。
- 删除 `FinalizePreparedSubstepJob` 及宽字段转发。
- repair publication 与 predictive activation 均改为显式并行阶段。
- 仍需在 Unity Editor 中复验 Burst、Collections Safety、PlayMode 和代表性
  Profiler 数据；静态结构完成不等同于运行性能结论。
