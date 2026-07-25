from pathlib import Path
import re

FLOW = Path("Entities/Unit/Systems/FlowField")
PIPE = FLOW / "Runtime/ContactPipeline"


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
    "Stages/Motion", "Stages/SoftAvoidance", "Stages/Solver",
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
    "InteractionCertificationJob": PIPE / "Stages/Certification/InteractionCertificationJob.cs",
    "MotionIntegrationJob": PIPE / "Stages/Motion/MotionIntegrationJob.cs",
    "SoftAvoidanceJob": PIPE / "Stages/SoftAvoidance/SoftAvoidanceJob.cs",
    "ConstraintSolverJob": PIPE / "Stages/Solver/ConstraintSolverJob.cs",
}
for name, path in stage_files.items():
    if f"public partial struct {name} : IJob" not in read(path):
        fail(f"Focused stage ABI missing: {name}")
if (PIPE / "Core/ContactPipelineStageJobs.cs").exists():
    fail("Four-stage ABI aggregate returned")

scheduler = read(PIPE / "Scheduling/CrowdContactPipelineScheduler.cs")
parallel_scheduler = read(PIPE / "Scheduling/Parallel/ParallelContactPipelineScheduler.cs")
algorithm_jobs = read(PIPE / "Scheduling/Parallel/Jobs/ParallelContactPipelineJobs.cs") + read(
    PIPE / "Scheduling/Parallel/Jobs/ParallelJacobiJobs.cs")
if ": IJob" in scheduler or re.search(r"private struct .*Job", scheduler + parallel_scheduler):
    fail("Scheduler owns executable job algorithms")
for token in ("ScheduleParallelJacobiP1P6", "ValidateConsumerViewsP1P6"):
    if token not in parallel_scheduler:
        fail(f"Parallel scheduling boundary missing: {token}")
for token in ("EvaluateParallelJacobiPairsJob", "GatherAndApplyParallelJacobiBodiesJob"):
    if token not in algorithm_jobs:
        fail(f"Parallel algorithm container missing: {token}")

# All partial fragments must live under their owning stage/scheduling root.
allowed = {
    "InteractionCertificationJob": PIPE / "Stages/Certification",
    "MotionIntegrationJob": PIPE / "Stages/Motion",
    "SoftAvoidanceJob": PIPE / "Stages/SoftAvoidance",
    "ConstraintSolverJob": PIPE / "Stages/Solver",
    "SerialContactPipelineLifecycleJob": PIPE / "Stages/Lifecycle",
    "ParallelContactPipelineLifecycleJob": PIPE / "Stages/Lifecycle",
    "CrowdContactPipelineScheduler": PIPE / "Scheduling",
}
for path in FLOW.rglob("*.cs"):
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

for relative in (
    "Stages/Lifecycle/SerialContactPipelineLifecycleJob.cs",
    "Stages/Lifecycle/ParallelContactPipelineLifecycleJob.cs",
):
    if not (PIPE / relative).is_file():
        fail(f"Lifecycle execution-mode ABI missing: {relative}")

for relative, required_bindings in {
    "Frame/InteractionCertificationFrameResources.cs": (
        "RuntimeState = execution.ParallelJacobiRuntimeState",
        "IterationState = execution.ParallelJacobiIterationState",
        "BlockStatistics = execution.ParallelJacobiBlockTelemetry",
    ),
    "Frame/SoftAvoidanceFrameResources.cs": (
        "RuntimeState = execution.ParallelJacobiRuntimeState",
    ),
    "Frame/ConstraintSolverFrameResources.cs": (
        "RuntimeState = execution.ParallelJacobiRuntimeState",
        "IterationState = execution.ParallelJacobiIterationState",
        "BlockStatistics = execution.ParallelJacobiBlockTelemetry",
    ),
}.items():
    source = read(resources / relative)
    for binding in required_bindings:
        if binding not in source:
            fail(f"Stage capability binding missing in {relative}: {binding}")

certifier = read(PIPE / "Stages/Certification/Prediction/InteractionCorrectnessCertifier.cs")
certificate = read(PIPE / "Contracts/Certification/InteractionCertificationContracts.cs")
for token in ("BuildCertificationFlags(", "IsConsumerCertificateValid(",
              "ValidateConsumerViewsSerial(", "ValidateConsumerViewsP1P6("):
    if token not in certifier:
        fail(f"Certificate gate capability missing: {token}")
if "CertificateScopeMismatch" not in certificate or "CommittedViewMismatch" not in certificate:
    fail("Certificate gate violations are not explicit")

for retired in (FLOW / "Jobs/ContactPipeline", FLOW / "ContactPipelineResources.cs",
                FLOW / "BaseFlowMovementComposition.cs"):
    if retired.exists():
        fail(f"Retired contact architecture returned: {retired}")
print("Contact architecture contracts passed.")
