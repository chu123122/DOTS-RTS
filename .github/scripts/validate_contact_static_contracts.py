from pathlib import Path
import re

FLOW = Path("Gameplay/Entities/Unit/Systems/FlowField")
GAMEPLAY_ADAPTER = FLOW / "Runtime/BuildCrowdMotionIntentJob.cs"
PHYSICS = Path("Physics")
PIPE = PHYSICS / "ContactPipeline"
WORKFLOWS = Path(".github/workflows")
QUERY_PROTOCOL = Path("Gameplay/Entities/_Common/Systems/Physics")


def fail(message: str) -> None:
    raise SystemExit(message)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")

sources = {
    path: read(path)
    for root in (FLOW, PHYSICS)
    for path in root.rglob("*.cs")
}
ownership = {
    r"\bpublic\s+struct\s+BodyPair\b": PIPE / "Contracts/Interaction/BodyPair.cs",
    r"\bpublic\s+struct\s+ContactConstraint\b": PIPE / "Contracts/Interaction/ContactConstraint.cs",
    r"\bpublic\s+struct\s+SweptDiscCellEntry\b": PIPE / "Contracts/Interaction/SweptDiscTypes.cs",
    r"\bpublic\s+struct\s+IncrementalContactCacheState\b": PIPE / "Contracts/Interaction/IncrementalContactCacheState.cs",
    r"\bpublic\s+struct\s+IncrementalContactPipelineStatistics\b": PIPE / "Observability/Contracts/ContactPipelineTelemetry.cs",
}
for pattern, expected in ownership.items():
    found = [path for path, text in sources.items() if re.search(pattern, text)]
    if found != [expected]:
        fail(f"Type ownership mismatch {pattern}: {found}")

constraint = read(PIPE / "Contracts/Interaction/ContactConstraint.cs")
for token in ("public struct ContactConstraintDefinition",
              "public struct ContactConstraintRuntime",
              "public ContactConstraintDefinition Definition",
              "public ContactConstraintRuntime Runtime"):
    if token not in constraint:
        fail(f"Contact definition/runtime split missing: {token}")

for path, text in sources.items():
    if text.count("{") != text.count("}"):
        fail(f"Brace mismatch: {path}")
    meta = Path(str(path) + ".meta")
    if not meta.exists() or "guid:" not in read(meta):
        fail(f"Missing Unity source metadata: {path}")

if (PHYSICS / "Jobs/ContactPipeline").exists():
    fail("Legacy ContactPipeline physical root returned")
if not GAMEPLAY_ADAPTER.exists():
    fail("Gameplay FlowField adapter is not owned by Gameplay")
if (PHYSICS / "Jobs/BuildCrowdMotionIntentJob.cs").exists():
    fail("Gameplay FlowField adapter returned to Physics")
if "RTS.Gameplay.Core" in read(PHYSICS / "RTS.Physics.asmdef"):
    fail("Physics assembly has a reverse dependency on Gameplay.Core")
gameplay_grid_contract = read(
    Path("Gameplay/Core/Unit/Components/FlowField/GridComponent.cs"))
for physics_owned in (
    "UnitContactSolverSettings",
    "PredictiveDiscContactStatistics",
    "ContactPositionSolverMode",
    "SoftAvoidanceVelocitySolverMode",
):
    if physics_owned in gameplay_grid_contract:
        fail(f"Physics contract returned to Gameplay.Core: {physics_owned}")
for path, text in sources.items():
    if PHYSICS not in path.parents or "Editor" in path.parts:
        continue
    if "using RTS.Unit.Components;" in text:
        fail(f"Physics runtime imports Gameplay component ownership: {path}")
    if "FlowFieldUtils." in text:
        fail(f"Physics runtime imports Gameplay grid math: {path}")
grid_geometry = read(
    PIPE / "Contracts/Interaction/CrowdEnvironmentViews.cs")
for grid_operation in (
    "public static int FlatIndex(",
    "public static int2 WorldToCell(",
):
    if grid_operation not in grid_geometry:
        fail(f"Physics grid geometry operation missing: {grid_operation}")
for path in FLOW.glob("*Resources.cs"):
    if path.name in {"ContactPipelineResources.cs", "ContactPipelineExecutionResources.cs",
                     "CrowdStepBodyResources.cs", "ConstraintSolverFrameResources.cs",
                     "InteractionCertificationFrameResources.cs", "SoftAvoidanceFrameResources.cs"}:
        fail(f"Contact resource owner remains flat: {path}")

parallel_schedule = read(PIPE / "Scheduling/SharedContactPipelineScheduler.cs")
parallel_jobs = read(PIPE / "Scheduling/Parallel/Jobs/ParallelContactStageJobs.cs")
classification_publication_jobs = read(
    PIPE /
    "Scheduling/Parallel/Jobs/ClassificationPublicationStageJobs.cs")
dirty_refresh_jobs = read(
    PIPE / "Stages/Certification/SubstepRepair/DirtyBodyRefreshStageJobs.cs")
dirty_contact_jobs = read(
    PIPE / "Stages/Certification/SubstepRepair/DirtyContactScheduleStageJobs.cs")
dirty_incident_jobs = read(
    PIPE / "Stages/Certification/SubstepRepair/SubstepRepairStageJobs.cs")
full_sweep_jobs = read(
    PIPE / "Stages/Certification/InitialContact/FullSweepBroadPhaseStageJobs.cs")
persistent_topology_jobs = read(
    PIPE /
    "Stages/Certification/PersistentClassification/"
    "PersistentTopologyStageJobs.cs")
repair_kernel = read(
    PIPE / "Stages/Certification/SubstepRepair/PersistentRepairKernel.cs")
persistent_classification = read(
    PIPE /
    "Stages/Certification/PersistentClassification/"
    "PersistentClassificationKernel.cs")
repair_view = read(
    PIPE /
    "Stages/Certification/SubstepRepair/"
    "TimestepContactRepairViewKernel.cs")
prepare_repair_buffers = repair_kernel.split(
    "internal static void PrepareSubstepRepairBuffers(", 1)[1]
for token in (
    "previousTimestepContactPairs.Clear();",
    "classificationBodyPairs.Clear();",
    "pairs.Clear();",
    "classificationResults.Clear();",
):
    if token not in prepare_repair_buffers:
        fail(f"Substep repair early-return workset is not cleared: {token}")
wall_finalize = read(
    PIPE /
    "Stages/Certification/IterationFinalize/"
    "WallIterationFinalizeDataFlow.cs")
contact_finalize = read(
    PIPE /
    "Stages/Certification/IterationFinalize/"
    "ContactIterationFinalizeKernel.cs")
active_incident_jobs = read(
    PIPE /
    "Stages/Certification/Certificate/"
    "ActiveConstraintIncidentStageJobs.cs")
if "enum StagedContactPipelinePhase" not in read(
        PIPE / "Contracts/Execution/ContactPipelineStageContracts.cs"):
    fail("Named staged-pipeline phase contract missing")
for token in ("PrepareTimestepPredictionBodiesJob", "EvaluateSoftAvoidancePairsJob"):
    if token not in parallel_jobs or token not in parallel_schedule:
        fail(f"Parallel schedule/job linkage missing: {token}")

# Repair copy iteration counts belong to destination worksets that Prepare
# clears on every early return and resizes only for valid work.
for token in (
    "}.Schedule(\n                Classification.BodyPairs,",
    "}.Schedule(\n                PreviousTimestepContactPairs,",
):
    if token not in parallel_schedule:
        fail(f"Substep repair copy has the wrong deferred workset: {token}")
substep_repair_schedule = parallel_schedule.split(
    "private void ScheduleSubstepRepairPreparation(", 1)[1].split(
    "private void SchedulePersistentClassificationFinalization(", 1)[0]
for earlier, later in (
    ("handle = prepare;", "JobHandle copyInteractions ="),
    ("handle = copyInteractions;", "JobHandle copyPrevious ="),
):
    if earlier not in substep_repair_schedule or \
            substep_repair_schedule.index(earlier) > \
            substep_repair_schedule.index(later):
        fail(
            "Substep repair loses the last successfully scheduled handle: "
            f"{earlier}")

uncached_prediction_schedule = parallel_schedule.split(
    "if (!Configuration.EnableTimestepContactSetCache)", 1)[1].split(
        "handle = new ValidateBaseMotionBodiesJob", 1)[0]
for field_name in (
    "PersistentProxies",
    "PersistentProxyIndexByBody",
    "PersistentCacheState",
    "DirtyFlagsByBody",
):
    if f"{field_name} =" not in uncached_prediction_schedule:
        fail(
            "Uncached timestep prediction leaves a NativeContainer unbound: "
            f"{field_name}")

for source, job_name in (
    (dirty_contact_jobs, "ScatterDirtyContactScheduleJob"),
):
    scatter_job = source.split(
        f"internal struct {job_name}", 1)[1].split(
            "internal struct ", 1)[0]
    if "BlockCounts" not in scatter_job:
        fail(
            "Deferred scatter job does not retain its iteration-count "
            f"container: {job_name}")

classification_resources = read(
    PIPE / "State/Frame/ContactClassificationFrameResources.cs")
for token in (
    "public NativeList<byte> BlockWorkset;",
    "NativeArray<byte> Workset;",
    "PublicationBlockWorkset",
):
    if token not in (
            classification_publication_jobs +
            parallel_schedule +
            classification_resources):
        fail(f"Classification publication workset split missing: {token}")
classification_prepare_schedule = parallel_schedule.split(
    "new PrepareClassificationPublicationJob", 1)[1].split(
        "}.Schedule(handle);", 1)[0]
for field in (
    "BlockWorkset =",
    "ContactIndex =",
    "PersistentContacts =",
    "InitialTimestepContacts =",
):
    if field not in classification_prepare_schedule:
        fail(f"Classification publication prepare field unbound: {field}")
contact_view_jobs = read(
    PIPE /
    "Stages/Certification/SubstepRepair/"
    "ContactViewPublicationStageJobs.cs")
activation_jobs = read(
    PIPE /
    "Stages/Certification/SubstepRepair/"
    "PredictiveContactActivationStageJobs.cs")
activation_kernel = read(
    PIPE /
    "Stages/Certification/SubstepRepair/"
    "PredictiveContactActivationKernel.cs")
contact_view_scheduler = read(
    PIPE / "Scheduling/ContactViewPublicationScheduler.cs")
parallel_repair_source = (
    dirty_incident_jobs + repair_view + contact_view_jobs +
    activation_jobs + activation_kernel + contact_view_scheduler +
    parallel_schedule + repair_kernel)
for retired in (
    "MergeRepairedContactViewJob",
    "FinalizePreparedSubstepJob",
    "MergeEscapedTimestepContactView(",
    "ActivateScheduledPredictiveContactsForSubstep(",
    "InsertConstraintSorted(",
):
    if retired in parallel_repair_source:
        fail(f"Serial repair/activation path returned: {retired}")
for required in (
    "MaterializeRepairContactCandidatesJob :",
    "SortContactViewCandidateBlocksJob :",
    "MergeContactViewCandidateBlocksJob :",
    "CountRepairContactPublicationBlocksJob",
    "PrefixRepairContactPublicationJob",
    "ScatterRepairContactPublicationBlocksJob",
    "EvaluateScheduledContactsJob : IJobParallelForDefer",
    "CountPredictiveContactActivationBlocksJob",
    "PrefixPredictiveContactActivationJob",
    "ScatterPredictiveContactActivationBlocksJob",
    "ScheduleRepairContactViewPublication(",
    "SchedulePredictiveContactActivation(",
):
    if required not in parallel_repair_source:
        fail(f"Parallel repair/activation stage missing: {required}")


evaluate_activation = activation_jobs.split(
    "internal struct EvaluateScheduledContactsJob", 1)[1].split(
        "internal struct ", 1)[0]
for token in (
    "[ReadOnly]",
    "public NativeArray<PersistentPredictiveContact> PersistentContacts;",
):
    if token not in evaluate_activation:
        fail(f"Activation evaluation persistent input is not readonly: {token}")
if re.search(r"PersistentContacts\[[^]]+\]\s*=", evaluate_activation):
    fail("Activation evaluation writes authoritative persistent contacts")
scatter_activation = activation_jobs.split(
    "internal struct ScatterPredictiveContactActivationBlocksJob", 1)[1].split(
        "internal struct ", 1)[0]
for token in (
    "record.HasPersistentUpdate != 0",
    "PersistentContacts[record.PersistentContactIndex] =",
):
    if token not in scatter_activation:
        fail(f"Activation persistent update scatter missing: {token}")

persistent_classification_schedules = parallel_schedule.split(
    "new EvaluatePersistentPairClassificationsJob")[1:]
if len(persistent_classification_schedules) < 3:
    fail("Persistent classification staged repair schedules are missing")
for schedule_index, schedule_source in enumerate(
        persistent_classification_schedules):
    initializer = schedule_source.split("}.Schedule(", 1)[0]
    if "MotionEvidence =" not in initializer:
        fail(
            "Persistent classification leaves MotionEvidence unbound at "
            f"schedule {schedule_index}")

repair_prediction_job = parallel_jobs.split(
    "internal struct PrepareRepairPredictionBodiesJob", 1)[1].split(
        "internal struct InitializeSoftAvoidanceBodiesJob", 1)[0]
if "int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;" not in repair_prediction_job:
    fail("Incremental repair prediction no longer maps compact dirty indices to bodies")
for element_type, field_name in (
    ("CrowdBodySnapshot", "Bodies"),
    ("CrowdNavigationState", "NavigationStates"),
    ("CrowdMotionIntent", "MotionIntents"),
):
    readonly_input = re.compile(
        r"\[ReadOnly\]\s*"
        rf"public NativeArray<{element_type}>\s+{field_name};")
    if not readonly_input.search(repair_prediction_job):
        fail(
            "Incremental repair prediction immutable input lacks ReadOnly: "
            f"{field_name}")
for element_type, field_name in (
    ("CrowdMotionEvidence", "MotionEvidence"),
    ("CrowdSolverBodyState", "StepStates"),
):
    unrestricted_write = re.compile(
        r"\[NativeDisableParallelForRestriction\]\s*"
        rf"public NativeArray<{element_type}>\s+{field_name};")
    if not unrestricted_write.search(repair_prediction_job):
        fail(
            "Incremental repair prediction indirect body write lacks an explicit "
            f"parallel-for restriction override: {field_name}")

all_source = "\n".join(sources.values())
for retired in (
    "InteractionCertificationAlgorithms",
    "CertificationStageKernel",
    "CertificationKernelResources",
    "CertificationKernelResources.Compose(",
    "CreateAlgorithms(",
    "InteractionCertificationJob.cs",
    "InteractionCertificationStageJobs.cs",
    "BuildCrowdObstacleSnapshotJob",
):
    if retired in all_source:
        fail(f"Retired certification facade returned: {retired}")

immutable_input_write = re.compile(
    r"(?:bodyResources\.)?\b"
    r"(?:Bodies|NavigationStates|MotionIntents)\[[^\]]+\]\s*=")
for path, text in sources.items():
    if PIPE not in path.parents:
        continue
    if path == PIPE / "Scheduling/CrowdPhysicsRuntime.cs":
        # The runtime boundary owns the one allowed AoS -> Physics SoA expansion.
        continue
    if immutable_input_write.search(text):
        fail(f"Contact pipeline writes immutable step input: {path}")

for path in (
    PIPE / "Scheduling/Parallel/Jobs/ParallelContactStageJobs.cs",
    PIPE / "Scheduling/Parallel/Jobs/ParallelJacobiJobs.cs",
    PIPE / "Stages/Solver/ConstraintSolverJob.cs",
    PIPE / "Stages/SoftAvoidance/SoftAvoidanceJob.cs",
):
    for line in read(path).splitlines():
        if re.search(
                r"public NativeArray<(?:CrowdBodySnapshot|"
                r"CrowdNavigationState|CrowdMotionIntent)>", line) and \
                "[ReadOnly]" not in line:
            fail(f"Immutable step input lacks ReadOnly capability: {path}")

motion_evidence_contract = read(
    PIPE / "Contracts/Body/CrowdMotionEvidence.cs")
solver_runtime_contract = read(
    PIPE / "Contracts/Body/CrowdSolverBodyState.cs")
for solver_field in ("ContactCorrection", "WallCorrection"):
    if f"public float3 {solver_field};" in motion_evidence_contract:
        fail(f"Solver runtime leaked into narrow-phase evidence: {solver_field}")
for runtime_field in (
    "TimestepContactCorrection",
    "TimestepWallCorrection",
):
    if f"public float3 {runtime_field};" not in solver_runtime_contract:
        fail(f"Solver cumulative runtime field missing: {runtime_field}")

solver_job = read(PIPE / "Stages/Solver/ConstraintSolverJob.cs")
if "[ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;" \
        not in solver_job:
    fail("Solver can write narrow-phase evidence")
for path in (
    PIPE / "Stages/Solver/XpbdContactSolver.cs",
    PIPE / "Stages/Solver/WallConstraintSolver.cs",
):
    if re.search(r"MotionEvidence\[[^\]]+\]\s*=", read(path)):
        fail(f"Solver writes narrow-phase evidence: {path}")
for job_name in (
    "PrepareBaseVelocityBodiesJob",
    "InitializeSoftAvoidanceBodiesJob",
    "GatherSoftAvoidanceBodiesJob",
    "PredictUnconstrainedBodiesJob",
    "SolveWallConstraintBodiesJob",
    "ReconstructVelocityBodiesJob",
):
    job_source = parallel_jobs.split(
        f"internal struct {job_name}", 1)[1].split(
            "internal struct ", 1)[0]
    if re.search(r"MotionEvidence\[[^\]]+\]\s*=", job_source):
        fail(f"Solver stage writes narrow-phase evidence: {job_name}")
    for line in job_source.splitlines():
        if "public NativeArray<CrowdMotionEvidence> MotionEvidence;" in line \
                and "[ReadOnly]" not in line:
            fail(f"Solver stage has writable narrow-phase evidence: {job_name}")
parallel_jacobi_jobs = read(
    PIPE / "Scheduling/Parallel/Jobs/ParallelJacobiJobs.cs")
gather_jacobi = parallel_jacobi_jobs.split(
    "internal struct GatherAndApplyParallelJacobiBodiesJob", 1)[1]
if "MotionEvidence" in gather_jacobi:
    fail("Jacobi gather still owns narrow-phase evidence")

environment_publisher = read(FLOW / "FlowFieldBakeSystem.cs")
environment_job = read(FLOW / "GenerateCostFieldJob.cs")
composition_root = read(FLOW / "BaseFlowMovementSystem.cs")
timestep_cache = read(
    PIPE / "State/Frame/ContactPipelineExecutionResources.cs")
for token in (
    "GenerateCrowdObstacleFieldJob",
    "public NativeArray<CrowdObstacleCell> ObstacleCells",
    "CollisionWorld.CalculateDistance(",
):
    if token not in environment_job:
        fail(f"Unity Physics obstacle publication is incomplete: {token}")
for token in (
    "TryGetPublishedObstacleSnapshot(",
    "_obstacleVersion",
    "_activeObstacleCells",
    "_pendingObstacleCells",
):
    if token not in environment_publisher:
        fail(f"Published obstacle snapshot state missing: {token}")
if "timestepCache.ObstacleCells" in composition_root or \
        "NativeArray<CrowdObstacleCell> ObstacleCells" in timestep_cache:
    fail("Per-timestep FlowField obstacle translation returned")

shape_contract = read(
    Path("Gameplay/Core/Unit/Components/BasicUnitComponents.cs"))
shape_sync = read(
    Path("Gameplay/Entities/Unit/Systems/Initialization/CrowdShapeSyncSystem.cs"))
body_contract = read(
    PIPE / "Contracts/Body/CrowdBodySnapshot.cs")
proxy_contract = read(
    PIPE / "Contracts/Interaction/PersistentSweptProxy.cs")
dirty_contract = read(
    PIPE / "Contracts/Interaction/IncrementalContactCacheState.cs")
proxy_builder = read(PIPE / "Kernels/PersistentProxyBuilder.cs")
for source, token in (
    (shape_contract, "public uint Version;"),
    (shape_sync, "class CrowdShapeSyncSystem"),
    (shape_sync, "CrowdShapeAdapter.NextVersion("),
    (body_contract, "public uint ShapeVersion;"),
    (proxy_contract, "public uint ShapeVersion;"),
    (dirty_contract, "Shape = 1 << 4"),
    (proxy_builder, "previous.ShapeVersion != current.ShapeVersion"),
):
    if token not in source:
        fail(f"Crowd shape version chain missing: {token}")
cache_reusability = read(PIPE / "Kernels/PersistentCacheReusability.cs")
if "state.ObstacleVersion == config.ObstacleVersion" not in cache_reusability:
    fail("Obstacle version does not invalidate the persistent cache")

query_proxy_contract = read(
    Path("Gameplay/Core/Unit/Components/BasicUnitComponents.cs"))
query_proxy_publication = read(
    QUERY_PROTOCOL / "CrowdQueryProxyPublicationSystem.cs")
query_group = read(QUERY_PROTOCOL / "CrowdQuerySystemGroup.cs")
query_filters = read(QUERY_PROTOCOL / "CrowdQueryCollisionFilters.cs")
apply_movement = read(FLOW / "Runtime/ApplyFlowMovementJob.cs")
attack_query = read(
    Path("Gameplay/Entities/_Common/Systems/Attack/UnitAttackTriggerSystem.cs"))
track_query = read(
    Path("Gameplay/Entities/_Common/Systems/Track/TrackTriggerSystem.cs"))
attack_contract = read(
    Path("Gameplay/Entities/_Common/Systems/Attack/AttackComponents.cs"))
track_contract = read(
    Path("Gameplay/Entities/_Common/Systems/Track/TrackComponents.cs"))
damage_contract = read(
    Path("Gameplay/Entities/_Common/Systems/HealPoint/DamageComponents.cs"))
damage_trigger = read(
    Path(
        "Gameplay/Entities/_Common/Systems/DamageOnTrigger/"
        "DamageOnTriggerSystem.cs"))
damage_consumer = read(
    Path(
        "Gameplay/Entities/_Common/Systems/HealPoint/"
        "CalculateFrameDamageSystem.cs"))

for source, token in (
    (query_proxy_contract, "public uint CrowdStepVersion;"),
    (query_proxy_contract, "public uint ProxyVersion;"),
    (apply_movement, "queryProxy.CrowdStepVersion = CrowdStepVersion;"),
    (query_proxy_publication, "class CrowdQueryProxyPublicationSystem"),
    (query_proxy_publication, "[UpdateAfter(typeof(PhysicsInitializeGroup))]"),
    (query_proxy_publication, "[UpdateBefore(typeof(PhysicsSimulationGroup))]"),
    (query_proxy_publication, "proxy.ProxyVersion = proxy.CrowdStepVersion;"),
    (query_group, "[UpdateAfter(typeof(FixedStepSimulationSystemGroup))]"),
    (query_group, "[UpdateBefore(typeof(CrowdPhysicsSystemGroup))]"),
    (attack_contract, "public uint QueryProxyVersion;"),
    (track_contract, "public uint QueryProxyVersion;"),
    (damage_contract, "public uint QueryProxyVersion;"),
    (damage_trigger, "QueryProxyVersion = queryProxyVersion"),
    (damage_consumer, "IsDamageVersionCurrent("),
):
    if token not in source:
        fail(f"Crowd query proxy version protocol missing: {token}")

for source, name in (
    (attack_query, "attack"),
    (track_query, "track"),
):
    if "UpdateInGroup(typeof(CrowdQuerySystemGroup))" not in source:
        fail(f"{name} query does not run against the published PhysicsWorld")
    if "UpdateInGroup(typeof(PredictedSimulationSystemGroup))" in source:
        fail(f"{name} query returned to the pre-Physics prediction group")
    for token in (
        "sourceQueryProxy.CrowdStepVersion ==",
        "sourceQueryProxy.ProxyVersion",
        "physicsWorld.GetRigidBodyIndex(entity)",
        "physicsWorld.Bodies[sourceBodyIndex].WorldFromBody.pos",
        "targetProxy.ProxyVersion != queryProxyVersion",
        "QueryProxyVersion = queryProxyVersion",
    ):
        if token not in source:
            fail(f"{name} query version declaration missing: {token}")

for source, token in (
    (query_filters, "public const uint Unit = 1u << 1;"),
    (query_filters, "public const uint Obstacle = 1u << 2;"),
    (query_filters, "CrowdQueryCollisionFilters.UnitOverlap"),
    (query_filters, "CrowdQueryCollisionFilters.ObstacleOverlap"),
):
    if token not in source and token.split(".")[-1] not in source:
        fail(f"Crowd query collision filter contract missing: {token}")

for prefab_name in ("Unit.prefab", "Unit 1.prefab", "Unit 2.prefab"):
    prefab = read(Path("../Resources/Prefabs") / prefab_name)
    collides_with = re.search(
        r"m_CollidesWithCategories:\s*"
        r".*?Category01:\s*(\d)",
        prefab,
        re.DOTALL)
    if collides_with is None or collides_with.group(1) != "0":
        fail(f"Unity Physics Unit-Unit response filter is enabled: {prefab_name}")

for token in (
    "RefreshDirtyBodiesJob : IJobParallelForDefer",
    "ReduceDirtyBodyRefreshJob : IJob",
):
    if token not in dirty_refresh_jobs or token.split(" :")[0] not in parallel_schedule:
        fail(f"Dirty body refresh chain missing: {token}")
for token in (
    "CountDirtyContactScheduleJob : IJobParallelForDefer",
    "PrefixDirtyContactScheduleJob : IJob",
    "ScatterDirtyContactScheduleJob : IJobParallelForDefer",
):
    if token not in dirty_contact_jobs or token.split(" :")[0] not in parallel_schedule:
        fail(f"Dirty contact/schedule chain missing: {token}")
for token in (
    "CountBodyCellsJob : IJobParallelForDefer",
    "PrefixBodyCellsJob : IJob",
    "ScatterBodyCellsJob : IJobParallelForDefer",
    "CountCellPairsJob : IJobParallelForDefer",
    "PrefixCellPairsJob : IJob",
    "ScatterCellPairsJob : IJobParallelForDefer",
    "DeduplicateAndPublishBroadPhasePairsJob : IJob",
):
    if token not in full_sweep_jobs or token.split(" :")[0] not in parallel_schedule:
        fail(f"Full sweep broad-phase chain missing: {token}")

for chain in (
    ("RefreshDirtyBodiesJob", "ReduceDirtyBodyRefreshJob"),
    ("CountDirtyContactScheduleJob", "PrefixDirtyContactScheduleJob",
     "ScatterDirtyContactScheduleJob"),
    ("CountBodyCellsJob", "PrefixBodyCellsJob", "ScatterBodyCellsJob",
     "CountCellPairsJob", "PrefixCellPairsJob", "ScatterCellPairsJob",
     "PrepareBroadPhasePairSortJob", "SortBroadPhasePairBlocksJob",
     "MergeBroadPhasePairBlocksJob",
     "DeduplicateAndPublishBroadPhasePairsJob"),
    ("PreparePersistentTopologyPublicationJob",
     "BuildPersistentProxiesJob",
     "BuildPersistentProxyIndexJob",
     "PublishPersistentNeighborPairsJob",
     "FinalizePersistentTopologyPublicationJob"),
    ("ScheduleFullSweepBroadPhase(",
     "SchedulePersistentTopologyPublication(ref handle);",
     "SchedulePersistentReusePublication(ref handle, runtimeState);",
     "new PreparePersistentClassificationJob"),
):
    positions = [parallel_schedule.find(token) for token in chain]
    if positions != sorted(positions) or any(position < 0 for position in positions):
        fail(f"Parallel stage order mismatch: {' -> '.join(chain)}")
substep_schedule = parallel_schedule.split(
    "for (int substepIndex = 0;", 1)[1].split(
        "for (int iterationIndex = 0;", 1)[0]
for token in (
    "SchedulePersistentTopologyPublication(ref handle);",
    "ScheduleSubstepRepairPreparation(",
):
    if token not in substep_schedule:
        fail(f"Persistent substep stage missing: {token}")
if substep_schedule.find(
        "SchedulePersistentTopologyPublication(ref handle);") > \
        substep_schedule.find("ScheduleSubstepRepairPreparation("):
    fail("Persistent topology publication is scheduled after classification")
if "SortJobDefer" in parallel_schedule:
    fail("Broad-phase pair sort returned to SortJobDefer")
for token in (
    "new PrepareBroadPhasePairSortJob",
    "new SortBroadPhasePairBlocksJob",
    "new MergeBroadPhasePairBlocksJob",
    "new CopyBroadPhasePairSortResultJob",
):
    if token not in parallel_schedule:
        fail(f"Staged broad-phase pair sort is incomplete: {token}")
if full_sweep_jobs.count(
        "FullSweepBroadPhaseMath.IsCanonicalSharedCell(") != 2 or \
        "int2 canonicalCell = math.max(minA, minB);" not in full_sweep_jobs:
    fail("Full sweep does not emit each shared-cell pair from one canonical cell")

prepare_repair = repair_kernel.split(
    "internal static void PrepareSubstepRepairBuffers(", 1)[1]
for retired_call in (
    "CertificateDataFlow.PrepareCurrentBodyLookup(",
    "MapDirtyIncidentNeighborPairsToCurrentBodies(",
    "FullRebuildPersistentNeighborTopology(",
    "IncrementallyRepairPersistentNeighborTopology(",
    "BuildOrRefreshTimestepContactViews(",
):
    if retired_call in prepare_repair:
        fail(f"Serial substep repair work returned: {retired_call}")
for retired_call in (
    "RepairSubstepContactView(",
    "RepairOrRebuildPreparedContactViewForRemainingTime(",
    "BuildSubstepInteractionAndSoftViews(",
    "PrepareSubstepContactPrediction(",
    "BuildSubstepContactView(",
    "BuildOrRefreshTimestepContactViews(",
):
    if retired_call in repair_kernel:
        fail(
            "Serial substep repair entry returned anywhere in "
            f"PersistentRepairKernel: {retired_call}")

prepare_classification = persistent_classification.split(
    "internal static void PreparePersistentClassification(", 1)[1].split(
        "internal static bool TryFindPersistentProxy", 1)[0]
for retired_call in (
    "RefreshPersistentPairSourceForClassification(",
    "FullRebuildPersistentNeighborTopology(",
    "IncrementallyRepairPersistentNeighborTopology(",
):
    if retired_call in prepare_classification:
        fail(f"Serial initial classification work returned: {retired_call}")

for source, stage in (
    (activation_jobs, "PredictiveContactActivationStageJobs"),
    (wall_finalize, "FinalizeWallIteration"),
    (contact_finalize, "FinalizeContactIteration"),
):
    for retired_call in (
        "RepairOrRebuildContactViewForRemainingTime(",
        "BuildOrRefreshTimestepContactViews(",
        "EnsureActiveConstraintIncidentIndex(",
    ):
        if retired_call in source:
            fail(
                f"Serial finalize repair returned in {stage}: "
                f"{retired_call}")

for token in (
    "PrepareActiveIncidentIndexJob : IJob",
    "ClearActiveIncidentCountsJob : IJobParallelForDefer",
    "CountActiveIncidentPairsJob : IJobParallelForDefer",
    "PrefixActiveIncidentPairsJob : IJob",
    "ScatterActiveIncidentPairsJob : IJobParallelForDefer",
    "SortActiveIncidentRangesJob : IJobParallelForDefer",
):
    if token not in active_incident_jobs or \
            token.split(" :")[0] not in parallel_schedule:
        fail(f"Parallel active-incident chain missing: {token}")
for token in (
    "BodyWorkset.ResizeUninitialized(prepared != 0 ? BodyCount : 0)",
    "BroadPhase.FullSweepBodyWorkset",
    "Solver.ActiveIncidentBodyWorkset",
    "Solver.ActiveIncidentPairWorkset",
    "state.Fingerprint == fingerprint",
):
    if token not in full_sweep_jobs + parallel_schedule + active_incident_jobs:
        fail(f"Deferred rebuild early-out missing: {token}")
for source, job_name in (
    (parallel_jobs, "PrepareSubstepContactPredictionBodiesJob"),
    (full_sweep_jobs, "CountBodyCellsJob"),
    (full_sweep_jobs, "ScatterBodyCellsJob"),
    (persistent_topology_jobs, "BuildPersistentProxiesJob"),
    (persistent_topology_jobs, "BuildPersistentProxyIndexJob"),
    (persistent_topology_jobs, "PublishPersistentNeighborPairsJob"),
    (persistent_topology_jobs, "MapPersistentReusePairsJob"),
    (active_incident_jobs, "ClearActiveIncidentCountsJob"),
    (active_incident_jobs, "CountActiveIncidentPairsJob"),
    (active_incident_jobs, "ScatterActiveIncidentPairsJob"),
    (active_incident_jobs, "SortActiveIncidentRangesJob"),
):
    job_start = source.find(job_name)
    if job_start < 0:
        fail(f"Deferred workset job is missing: {job_name}")
    job_end = source.find("[BurstCompile]", job_start)
    job_source = source[
        job_start:job_end if job_end >= 0 else len(source)]
    if " Workset;" not in job_source:
        fail(
            "Deferred iteration-count container is not retained by "
            f"{job_name}")
    for schedule_source in parallel_schedule.split(f"new {job_name}")[1:]:
        initializer = schedule_source.split("}.Schedule(", 1)[0]
        if "Workset =" not in initializer:
            fail(f"Deferred workset is unbound at {job_name} schedule")
if "DirtyBodies.Length == 0" not in read(
        PIPE / "Stages/Certification/Certificate/CertificateStageJobs.cs"):
    fail("No-dirty consumer validation early-out is missing")
for token in (
    "PreparePersistentReusePublicationJob : IJob",
    "MapPersistentReusePairsJob : IJobParallelForDefer",
    "FinalizePersistentReusePublicationJob : IJob",
    "PersistentReusePairWorkset",
    "RequireValidPersistentCache",
):
    if token not in persistent_topology_jobs + parallel_schedule + \
            full_sweep_jobs:
        fail(f"Persistent no-dirty reuse stage missing: {token}")
if parallel_schedule.count(
        "RequireDirtyBodies = (byte)(\n"
        "                    Configuration.EnableTimestepContactSetCache "
        "? 1 : 0)") < 2:
    fail("Substep consumer validation dirty gates are missing")
if (full_sweep_jobs + persistent_topology_jobs).count(
        "PersistentCacheReusability.IsStructurallyReusable(") < 2:
    fail("Persistent reuse does not validate the full cache fingerprint")
for token in (
    "PersistentProxies =",
    "PersistentProxyIndexByBody =",
    "Configuration = Configuration",
):
    if parallel_schedule.count(token) < 2:
        fail(f"Persistent reuse fingerprint input is unbound: {token}")
for token in (
    "SchedulePersistentRepairStages(",
    "ScheduleActiveConstraintIncidentIndex(",
    "ref JobHandle handle",
):
    if token not in parallel_schedule:
        fail(f"Finalize staged repair schedule missing: {token}")

for token in (
    "JobHandle handle = dependency;",
    "catch",
    "handle.Complete();",
):
    if token not in parallel_schedule:
        fail(f"Scheduler exception cleanup contract missing: {token}")

# Every pair whose cell rectangles overlap has exactly one canonical shared cell:
# component-wise max of the two minimum cells.
for min_ax in range(3):
    for max_ax in range(min_ax, 3):
        for min_ay in range(3):
            for max_ay in range(min_ay, 3):
                cells_a = {
                    (x, y)
                    for x in range(min_ax, max_ax + 1)
                    for y in range(min_ay, max_ay + 1)
                }
                for min_bx in range(3):
                    for max_bx in range(min_bx, 3):
                        for min_by in range(3):
                            for max_by in range(min_by, 3):
                                shared = cells_a & {
                                    (x, y)
                                    for x in range(min_bx, max_bx + 1)
                                    for y in range(min_by, max_by + 1)
                                }
                                if not shared:
                                    continue
                                canonical = (
                                    max(min_ax, min_bx),
                                    max(min_ay, min_by),
                                )
                                if canonical not in shared:
                                    fail("Canonical shared-cell property failed")

certification_source = "\n".join(
    text for path, text in sources.items()
    if PIPE / "Stages/Certification" in path.parents)
if re.search(
        r"=>\s*(?:Environment|Body|Views|Persistent|Solver|Diagnostics|Configuration)\.",
        certification_source):
    fail("Certification resource forwarding property returned")

scheduler_owner = read(
    PIPE / "Scheduling/CrowdContactPipelineScheduler.cs")
for owner in (
    "CrowdObstacleSnapshot Obstacles",
    "CrowdStepBodyResources Body",
    "BroadPhaseCandidateBatch BroadPhaseCandidates",
    "NarrowPhaseConstraintBatch NarrowPhaseConstraints",
    "BroadPhaseFrameResources BroadPhase",
    "NativeList<ContactConstraint> PreviousTimestepContactPairs",
    "ContactClassificationFrameResources Classification",
    "ContactRepairFrameResources Repair",
    "ContactCertificateFrameResources Certificate",
    "CrossFrameCache Persistent",
    "ConstraintSolverFrameResources Solver",
    "ContactPipelineExecutionResources Execution",
    "ContactDiagnosticsFrameResources Diagnostics",
):
    if owner not in scheduler_owner:
        fail(f"Scheduler direct resource owner is missing: {owner}")

for pattern in ("refactor-contact-pipeline-phase*.yml", "audit-*.yml",
                "apply-hard-cutover*.yml", "diagnose-*.yml"):
    if list(WORKFLOWS.glob(pattern)):
        fail(f"Temporary workflow remains: {pattern}")
print("Contact pipeline terminal static contracts passed.")
