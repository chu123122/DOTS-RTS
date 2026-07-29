from pathlib import Path
import re

FLOW = Path("Gameplay/Entities/Unit/Systems/FlowField")
PHYSICS = Path("Physics")
PIPE = PHYSICS / "ContactPipeline"


def fail(message: str) -> None:
    raise SystemExit(message)


def read(path: Path) -> str:
    if not path.exists():
        fail(f"Missing architecture target: {path}")
    return path.read_text(encoding="utf-8")

required_dirs = (
    "Contracts/Body", "Contracts/Certification", "Contracts/Execution",
    "Contracts/Interaction", "State/Frame", "State/Persistent", "Kernels",
    "Scheduling/Parallel/Jobs", "Stages/Lifecycle", "Stages/Certification",
    "Stages/Certification/InitialContact",
    "Stages/Certification/SubstepRepair",
    "Stages/Certification/PersistentClassification",
    "Stages/Certification/Certificate",
    "Stages/Certification/IterationFinalize",
    "Stages/SoftAvoidance", "Stages/Solver",
    "Observability/Contracts")
for relative in required_dirs:
    if not (PIPE / relative).is_dir():
        fail(f"Missing contact ownership directory: {relative}")

# One public body product per file keeps lifetime and mutation semantics visible.
for name in ("CrowdBodySnapshot", "CrowdNavigationState", "CrowdMotionIntent",
             "CrowdMotionEvidence", "CrowdBodyStepState", "CrowdBodyResult"):
    text = read(PIPE / f"Contracts/Body/{name}.cs")
    if f"public struct {name}" not in text:
        fail(f"Body contract ownership mismatch: {name}")

stage_files = {
    "SoftAvoidanceJob": PIPE / "Stages/SoftAvoidance/SoftAvoidanceJob.cs",
    "ConstraintSolverJob": PIPE / "Stages/Solver/ConstraintSolverJob.cs",
}
for name, path in stage_files.items():
    if f"public partial struct {name} : IJob" not in read(path):
        fail(f"Focused stage ABI missing: {name}")
certification_algorithms = read(
    PIPE / "Stages/Certification/CertificationStageKernel.cs")
certification_jobs = "\n".join(read(path) for path in (
    PIPE / "Stages/Certification/InitialContact/InitialContactStageJobs.cs",
    PIPE / "Stages/Certification/SubstepRepair/SubstepRepairStageJobs.cs",
    PIPE / "Stages/Certification/PersistentClassification/PersistentClassificationStageJobs.cs",
    PIPE / "Stages/Certification/Certificate/CertificateStageJobs.cs",
    PIPE / "Stages/Certification/IterationFinalize/IterationFinalizeStageJobs.cs",
))
if "internal partial struct CertificationStageKernel" not in certification_algorithms:
    fail("Certification stage kernel missing")
if ": IJob" in certification_algorithms or "InteractionCertificationOperation" in certification_algorithms:
    fail("Retired certification god job returned")
for name in (
    "PreparePersistentClassificationJob",
    "CommitPersistentClassificationJob",
    "BuildInitialContactSetJob",
    "FinalizeEnvelopeEscapesJob",
    "PrepareSubstepRepairJob",
    "CommitSubstepRepairJob",
    "FinalizePreparedSubstepJob",
    "ValidateConsumerViewsJob",
    "FinalizeWallIterationJob",
    "FinalizeContactIterationJob",
):
    if f"struct {name} : IJob" not in certification_jobs:
        fail(f"Certification stage job missing: {name}")
if (PIPE / "Core/ContactPipelineStageJobs.cs").exists():
    fail("Four-stage ABI aggregate returned")

scheduler = read(PIPE / "Scheduling/CrowdContactPipelineScheduler.cs")
parallel_scheduler = read(PIPE / "Scheduling/SharedContactPipelineScheduler.cs")
algorithm_jobs = read(PIPE / "Scheduling/Parallel/Jobs/ParallelContactStageJobs.cs") + read(
    PIPE / "Scheduling/Parallel/Jobs/ParallelJacobiJobs.cs")
if ": IJob" in scheduler or re.search(r"private struct .*Job", scheduler + parallel_scheduler):
    fail("Scheduler owns executable job algorithms")
for token in ("ScheduleParallelStages", "ValidateConsumerViews"):
    if token not in parallel_scheduler:
        fail(f"Parallel scheduling boundary missing: {token}")
for token in ("EvaluateParallelJacobiPairsJob", "GatherAndApplyParallelJacobiBodiesJob"):
    if token not in algorithm_jobs:
        fail(f"Parallel algorithm container missing: {token}")

# All partial fragments must live under their owning stage/scheduling root.
allowed = {
    "CertificationStageKernel": PIPE / "Stages/Certification",
    "SoftAvoidanceJob": PIPE / "Stages/SoftAvoidance",
    "ConstraintSolverJob": PIPE / "Stages/Solver",
    "ContactPipelineLifecycleJob": PIPE / "Stages/Lifecycle",
    "CrowdContactPipelineScheduler": PIPE / "Scheduling",
}
for path in list(PHYSICS.rglob("*.cs")) + list(FLOW.rglob("*.cs")):
    text = read(path)
    for name in re.findall(r"partial (?:struct|class)\s+(\w+)", text):
        owner = allowed.get(name)
        if owner is not None and owner not in path.parents:
            fail(f"{name} fragment escaped owner: {path}")

resources = PIPE / "State"
resource_contracts = {
    "Persistent/InteractionCandidateStore.cs": "struct InteractionCandidateStore",
    "Frame/CrowdStepBodyResources.cs": "struct CrowdStepBodyResources",
    "Frame/InteractionCertificationFrameResources.cs": "struct InteractionCertificationFrameResources",
    "Frame/SoftAvoidanceFrameResources.cs": "struct SoftAvoidanceFrameResources",
    "Frame/ConstraintSolverFrameResources.cs": "struct ConstraintSolverFrameResources",
    "Frame/ContactPipelineExecutionResources.cs": "struct ContactPipelineExecutionResources",
}
for relative, token in resource_contracts.items():
    source = read(resources / relative)
    if token not in source:
        fail(f"Resource owner mismatch: {relative}")
    if relative.startswith("Frame/") and re.search(r"\?\s*new Native[\s\S]{0,200}:\s*default", source):
        fail(f"Scheduled stage capability is conditionally unconstructed: {relative}")

for relative in ("Stages/Lifecycle/ContactPipelineLifecycleJob.cs",):
    if not (PIPE / relative).is_file():
        fail(f"Lifecycle execution-mode ABI missing: {relative}")

for relative, required_bindings in {
    "Frame/SoftAvoidanceFrameResources.cs": (
        "RuntimeState = execution.PipelineRuntimeState",
    ),
    "Frame/ConstraintSolverFrameResources.cs": (
        "RuntimeState = execution.PipelineRuntimeState",
        "IterationState = execution.SolverIterationState",
        "BlockStatistics = execution.JacobiBlockStatistics",
    ),
}.items():
    source = read(resources / relative)
    for binding in required_bindings:
        if binding not in source:
            fail(f"Stage capability binding missing in {relative}: {binding}")

composition = read(FLOW / "BaseFlowMovementSystem.cs")
for binding in (
    "CertificationEnvironment = certificationEnvironment",
    "CertificationBody = certificationBody",
    "CertificationViews = certificationViews",
    "CertificationPersistent = certificationPersistent",
    "CertificationSolver = certificationSolver",
    "CertificationDiagnostics = certificationDiagnostics",
):
    if binding not in composition:
        fail(f"Scheduler resource composition missing: {binding}")

certifier = read(PIPE / "Stages/Certification/Certificate/InteractionCorrectnessKernel.cs")
certificate = read(PIPE / "Contracts/Certification/InteractionCertificationContracts.cs")
for token in ("BuildCertificationFlags(", "GetConsumerCertificateFailure(",
              "ValidateConsumerViews("):
    if token not in certifier:
        fail(f"Certificate gate capability missing: {token}")
if "CertificateScopeMismatch" not in certificate or "CommittedViewMismatch" not in certificate:
    fail("Certificate gate violations are not explicit")

for retired in (PHYSICS / "Jobs/ContactPipeline", FLOW / "ContactPipelineResources.cs",
                FLOW / "BaseFlowMovementComposition.cs"):
    if retired.exists():
        fail(f"Retired contact architecture returned: {retired}")
for retired in (
    PIPE / "Scheduling/Parallel/ParallelContactPipelineScheduler.cs",
    PIPE / "Stages/Lifecycle/SerialContactPipelineLifecycleJob.cs",
    PIPE / "Stages/Certification/Prediction/SerialInteractionCertificationOperations.cs",
    PIPE / "Stages/Motion",
):
    if retired.exists():
        fail(f"Retired serial pipeline implementation returned: {retired}")
if "ScheduleSerial" in scheduler + parallel_scheduler:
    fail("Retired serial scheduler entry returned")
for token in (
    "ConstraintSolverOperation.SolveGaussSeidelContact",
    "EvaluateParallelJacobiPairsJob",
):
    if token not in parallel_scheduler:
        fail(f"XPBD backend branch missing: {token}")
print("Contact architecture contracts passed.")
