from pathlib import Path
import re

ROOT = Path(".")
PHYSICS = ROOT / "Physics"
PIPE = PHYSICS / "ContactPipeline"
GAMEPLAY = ROOT / "Gameplay"
FLOW = GAMEPLAY / "Entities/Unit/Systems/FlowField"


def fail(message: str) -> None:
    raise SystemExit(message)


def read(path: Path) -> str:
    if not path.exists():
        fail(f"Missing architecture target: {path}")
    return path.read_text(encoding="utf-8")


def require(source: str, token: str, owner: str) -> None:
    if token not in source:
        fail(f"Missing {owner}: {token}")


required_directories = (
    PIPE / "Contracts/Body",
    PIPE / "Contracts/Certification",
    PIPE / "Contracts/Execution",
    PIPE / "Contracts/Interaction",
    PIPE / "State/Frame",
    PIPE / "State/Persistent",
    PIPE / "Kernels",
    PIPE / "Scheduling/Parallel/Jobs",
    PIPE / "Stages/Lifecycle",
    PIPE / "Stages/Certification/InitialContact",
    PIPE / "Stages/Certification/SubstepRepair",
    PIPE / "Stages/Certification/PersistentClassification",
    PIPE / "Stages/Certification/Certificate",
    PIPE / "Stages/Certification/IterationFinalize",
    PIPE / "Stages/SoftAvoidance",
    PIPE / "Stages/Solver",
    PIPE / "Observability/Contracts",
)
for directory in required_directories:
    if not directory.is_dir():
        fail(f"Missing ownership directory: {directory}")

physics_sources = {
    path: read(path) for path in PHYSICS.rglob("*.cs")
}
gameplay_sources = {
    path: read(path) for path in GAMEPLAY.rglob("*.cs")
}
all_physics = "\n".join(physics_sources.values())
all_gameplay = "\n".join(gameplay_sources.values())

# Assembly direction: Gameplay may depend on Physics; Physics may not depend on
# Gameplay, and the runtime boundary may not be bypassed through a friend assembly.
physics_asmdef = read(PHYSICS / "RTS.Physics.asmdef")
gameplay_asmdef = read(GAMEPLAY / "RTS.Gameplay.asmdef")
network_asmdef = read(ROOT / "Network/RTS.Network.asmdef")
assembly_info = read(PHYSICS / "AssemblyInfo.cs")
if "RTS.Gameplay" in physics_asmdef:
    fail("RTS.Physics has a reverse Gameplay assembly dependency")
require(gameplay_asmdef, '"RTS.Physics"', "Gameplay -> Physics reference")
require(network_asmdef, '"RTS.Gameplay"', "Network -> Gameplay reference")
if 'InternalsVisibleTo("RTS.Gameplay")' in assembly_info:
    fail("Gameplay friend access to all Physics internals returned")

# Gameplay receives a narrow runtime lease. It must not own stage caches, resource
# owners, executable solver jobs or the internal composition function.
runtime = read(PIPE / "Scheduling/CrowdPhysicsRuntime.cs")
for token in (
    "public sealed class CrowdPhysicsRuntime",
    "public sealed class CrowdPhysicsStep",
    "public NativeArray<CrowdPhysicsBodyInput> InputBodies",
    "public NativeArray<CrowdBodyResult>.ReadOnly OutputBodies",
    "CrowdPhysicsPipelineComposition.ScheduleStep(",
):
    require(runtime, token, "Crowd Physics runtime facade")
for leaked in (
    "CrossFrameCache",
    "TimestepCache",
    "CrowdStepBodyResources",
    "BroadPhaseFrameResources",
    "ContactProductFrameResources",
    "ContactClassificationFrameResources",
    "ContactRepairFrameResources",
    "ContactCertificateFrameResources",
    "InteractionCertificationFrameResources",
    "ConstraintSolverFrameResources",
    "ContactDiagnosticsFrameResources",
    "CrowdPhysicsPipelineComposition",
    "AdaptCrowdPhysicsStepInputJob",
    "InitializeCrowdStepStateJob",
    "BuildCrowdBodyResultsJob",
):
    if leaked in all_gameplay:
        fail(f"Gameplay reaches Physics implementation detail: {leaked}")
diagnostic_resources = read(
    PIPE / "State/Frame/ContactDiagnosticsFrameResources.cs")
if "public struct ContactDiagnosticsFrameResources" in diagnostic_resources:
    fail("Mutable diagnostics resources became a public Physics API")
composition_root = read(FLOW / "BaseFlowMovementSystem.cs")
for token in (
    "CrowdPhysicsRuntime.Create()",
    "_physicsRuntime.CreateStep(unitCount)",
    "_physicsRuntime.ScheduleStep(",
    "physicsStep.InputBodies",
    "physicsStep.OutputBodies",
):
    require(composition_root, token, "Gameplay Physics facade use")
if re.search(r"new Native(?:Array|List|Reference|Parallel)", composition_root):
    fail("Gameplay composition root allocates Physics Native resources directly")

# Deleted god objects, compatibility bags, forwarding APIs and profiler-visible
# serial implementations may not return.
retired_symbols = (
    "InteractionCertificationAlgorithms",
    "CertificationStageKernel",
    "CertificationKernelResources",
    "CertificationEnvironmentResources",
    "CertificationBodyResources",
    "CertificationViewResources",
    "PersistentCertificationResources",
    "CertificationSolverResources",
    "CertificationDiagnosticsResources",
    "InteractionCertificationFrameResources",
    "CreateAlgorithms(",
    "CreateNarrowPhaseResources(",
    "CrowdBodyStepState",
    "PrepareSubstepRepairTopology",
    "SortJobDefer",
    "MergeRepairedContactViewJob",
    "FinalizePreparedSubstepJob",
    "MergeEscapedTimestepContactView(",
    "ActivateScheduledPredictiveContactsForSubstep(",
)
for symbol in retired_symbols:
    if symbol in all_physics + all_gameplay:
        fail(f"Retired architecture symbol returned: {symbol}")
retired_files = (
    PIPE / "Stages/Certification/CertificationStageKernel.cs",
    PIPE / "Stages/Certification/CertificationKernelResources.cs",
    PIPE / "Stages/Certification/CertificationResources.cs",
    PIPE / "Stages/Certification/SubstepRepair/IncrementalPredictiveContactKernel.cs",
    PIPE / "Stages/Certification/SubstepRepair/ContactEnvelopeGuardKernel.cs",
    PIPE / "Stages/Certification/InitialContact/SweptDiscBroadPhaseKernel.cs",
)
for path in retired_files:
    if path.exists():
        fail(f"Retired architecture file returned: {path}")

# Persistent predictive contacts have exactly one value authority. The map is a
# derived pair-key -> authoritative-list-index lookup.
candidate_store = read(
    PIPE / "State/Persistent/InteractionCandidateStore.cs")
require(
    candidate_store,
    "NativeList<PersistentPredictiveContact> PredictiveContacts",
    "persistent contact authority")
require(
    candidate_store,
    "NativeParallelHashMap<StableEntityPairKey, int> PredictiveContactIndex",
    "persistent contact derived index")
if "NativeHashMap<StableEntityPairKey, PersistentPredictiveContact>" in \
        all_physics:
    fail("Persistent contact values are duplicated in a hashmap")

# Stage jobs carry concrete containers. Aggregate resource/cache owners cannot be
# fields of scheduled jobs.
owner_types = (
    "CrossFrameCache",
    "TimestepCache",
    "CrowdStepBodyResources",
    "BroadPhaseFrameResources",
    "ContactProductFrameResources",
    "ContactClassificationFrameResources",
    "ContactRepairFrameResources",
    "ContactCertificateFrameResources",
    "ConstraintSolverFrameResources",
    "SoftAvoidanceFrameResources",
    "ContactPipelineExecutionResources",
    "ContactDiagnosticsFrameResources",
)
for path, source in physics_sources.items():
    for job in re.finditer(
            r"\bstruct\s+(\w+Job)\b[^{}]*:\s*IJob[^{]*\{", source):
        start = job.end()
        next_job = re.search(
            r"\bstruct\s+\w+Job\b[^{}]*:\s*IJob", source[start:])
        body = source[
            start:start + next_job.start()
            if next_job is not None else len(source)]
        for owner in owner_types:
            if re.search(rf"\b{owner}\b", body):
                fail(
                    f"Scheduled job {job.group(1)} embeds owner {owner}: {path}")

# Concrete certification stages may call their own DataFlow, never another
# stage's DataFlow. Shared behavior belongs in neutral Kernels.
stage_names = (
    "InitialContact",
    "SubstepRepair",
    "PersistentClassification",
    "Certificate",
    "IterationFinalize",
)
for stage in stage_names:
    stage_root = PIPE / f"Stages/Certification/{stage}"
    source = "\n".join(read(path) for path in stage_root.glob("*.cs"))
    for other in stage_names:
        if other != stage and f"{other}DataFlow" in source:
            fail(f"{stage} calls concrete {other}DataFlow")

# Body products are separated by writer responsibility.
solver_state = read(PIPE / "Contracts/Body/CrowdSolverBodyState.cs")
avoidance_state = read(PIPE / "Contracts/Body/CrowdAvoidanceState.cs")
for token in ("SoftVelocity", "WallVelocity", "NeighborCount"):
    require(avoidance_state, token, "avoidance state")
    if token in solver_state:
        fail(f"Avoidance field returned to solver state: {token}")
for token in (
    "IntegratedVelocity",
    "SolvedPosition",
    "ContactCorrection",
    "WallCorrection",
):
    require(solver_state, token, "solver state")

# The expensive serial profiler entries are replaced by explicit staged chains.
scheduler = (
    read(PIPE / "Scheduling/SharedContactPipelineScheduler.cs") +
    read(PIPE / "Scheduling/ContactViewPublicationScheduler.cs"))
full_sweep = read(
    PIPE / "Stages/Certification/InitialContact/FullSweepBroadPhaseStageJobs.cs")
repair = (
    read(PIPE / "Stages/Certification/SubstepRepair/SubstepRepairStageJobs.cs") +
    read(PIPE / "Stages/Certification/SubstepRepair/ContactViewPublicationStageJobs.cs") +
    read(PIPE / "Stages/Certification/SubstepRepair/PredictiveContactActivationStageJobs.cs"))
for token in (
    "CountBodyCellsJob : IJobParallelForDefer",
    "PrefixBodyCellsJob : IJob",
    "ScatterBodyCellsJob : IJobParallelForDefer",
    "CountCellPairsJob : IJobParallelForDefer",
    "PrefixCellPairsJob : IJob",
    "ScatterCellPairsJob : IJobParallelForDefer",
    "PrepareBroadPhasePairSortJob : IJob",
    "SortBroadPhasePairBlocksJob : IJobParallelForDefer",
    "MergeBroadPhasePairBlocksJob : IJobParallelForDefer",
    "CopyBroadPhasePairSortResultJob : IJobParallelForDefer",
    "DeduplicateAndPublishBroadPhasePairsJob : IJob",
):
    require(full_sweep, token, "full-sweep staged broad phase")
    require(scheduler, token.split(" :")[0], "full-sweep schedule")
for token in (
    "PrepareSubstepRepairBuffersJob : IJob",
    "CopySubstepRepairInteractionPairsJob",
    "IJobParallelForDefer",
    "CopyPreviousTimestepContactPairsJob",
):
    require(repair, token, "substep-repair staged copy")
for token in (
    "ScheduleSubstepRepairPreparation(",
    "new PrepareSubstepRepairBuffersJob",
    "new CopySubstepRepairInteractionPairsJob",
    "new CopyPreviousTimestepContactPairsJob",
):
    require(scheduler, token, "substep-repair schedule")
for retired in (
    "CommitPersistentClassificationJob",
    "CommitSubstepRepairJob",
):
    if retired in all_physics:
        fail(f"Serial classification god job returned: {retired}")
for token in (
    "PublishPersistentClassificationStateJob : IJob",
    "FinalizePersistentClassificationCertificateJob : IJob",
    "PublishSubstepRepairClassificationJob : IJob",
    "MaterializeRepairContactCandidatesJob",
    "SortContactViewCandidateBlocksJob",
    "MergeContactViewCandidateBlocksJob",
    "CountRepairContactPublicationBlocksJob",
    "PrefixRepairContactPublicationJob",
    "ScatterRepairContactPublicationBlocksJob",
    "EvaluateScheduledContactsJob",
    "CountPredictiveContactActivationBlocksJob",
    "PrefixPredictiveContactActivationJob",
    "ScatterPredictiveContactActivationBlocksJob",
    "ClearRepairedEnvelopeEscapeJob : IJobParallelForDefer",
    "PreparePersistentIncidentLookupJob : IJob",
    "ScatterPersistentIncidentLookupJob : IJobParallelForDefer",
    "FinalizeSubstepRepairCertificateJob : IJob",
):
    require(all_physics, token, "explicit classification/repair stage")
    require(scheduler, token.split(" :")[0], "classification/repair schedule")

# GS and Jacobi reuse the same pre/post stages; only the XPBD projection branch
# is backend-specific.
for token in (
    "ConstraintSolverOperation.SolveGaussSeidelContact",
    "EvaluateParallelJacobiPairsJob",
    "GatherAndApplyParallelJacobiBodiesJob",
):
    require(scheduler, token, "XPBD backend branch")
if "ScheduleSerial" in scheduler:
    fail("Retired serial scheduler entry returned")

print("Contact architecture contracts passed.")
