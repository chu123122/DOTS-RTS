from pathlib import Path
import re

FLOW = Path("Gameplay/Entities/Unit/Systems/FlowField")
PHYSICS = Path("Physics")
PIPE = PHYSICS / "ContactPipeline"


def fail(message: str) -> None:
    raise SystemExit(message)


def read(path: Path) -> str:
    if not path.exists():
        fail(f"Missing audit target: {path}")
    return path.read_text(encoding="utf-8")

sources = {
    path: read(path)
    for root in (FLOW, PHYSICS)
    for path in root.rglob("*.cs")
}
all_source = "\n".join(sources.values())
for token in ("FlowMovementFrameState", "UnitCollisionPair", "SolveXpbdUnitContactsJob",
              "ContactFrameResources", "ContactPersistentState", "ContactPipelineRuntimeOptions"):
    if re.search(rf"\b{re.escape(token)}\b", all_source):
        fail(f"Retired contact symbol returned: {token}")

configuration = read(PIPE / "Contracts/Execution/ContactPipelineConfiguration.cs")
for token in ("WorldId", "SimulationStepId", "GuardEnvelopeMargin",
              "CalculateCertificationFingerprint"):
    if token not in configuration:
        fail(f"Configuration contract missing: {token}")

solver = read(PIPE / "Stages/Solver/XpbdContactSolver.cs")
jacobi = read(PIPE / "Scheduling/Parallel/Jobs/ParallelJacobiJobs.cs")
for token in ("internal struct ContactConstraintEvaluation",
              "internal static class XpbdContactConstraintMath"):
    if token not in solver:
        fail(f"Solver math boundary missing: {token}")
if jacobi.count("XpbdContactConstraintMath.Evaluate(") != 1:
    fail("Parallel Jacobi pair math is duplicated")
if "Interlocked" in jacobi or "Atomic" in jacobi:
    fail("Parallel Jacobi uses floating-point atomics")

base = read(FLOW / "BaseFlowMovementSystem.cs")
if re.search(r"new Native(?:Array|List|Reference|Parallel)", base):
    fail("Composition root allocates contact Native resources directly")
for token in (
    "CrowdPhysicsRuntime.Create()",
    "_physicsRuntime.CreateStep(unitCount)",
    "_physicsRuntime.ScheduleStep(",
):
    if token not in base:
        fail(f"Composition root wiring missing: {token}")
for retired in ("InteractionCertificationFrameResources.Create(",
                "SoftAvoidanceFrameResources.Create(",
                "ConstraintSolverFrameResources.Create(",
                "ContactPipelineExecutionResources.Create("):
    if retired in base:
        fail(f"Gameplay still allocates an internal stage resource: {retired}")

certifier = read(PIPE / "Kernels/InteractionCertificateKernel.cs")
classification_publication = read(
    PIPE / "Scheduling/Parallel/Jobs/ClassificationPublicationStageJobs.cs")
classification_stage = read(
    PIPE /
    "Stages/Certification/PersistentClassification/"
    "PersistentClassificationStageJobs.cs")
for token in (
    "IssueCertificateForCommittedViews(",
    "GetConsumerCertificateFailure(",
):
    if token not in certifier:
        fail(f"Certification boundary missing: {token}")
for token in (
    "ScatterClassificationPublicationBlocksJob",
    "InitialTimestepContacts[constraintWrite]",
):
    if token not in classification_publication:
        fail(f"Interaction view publication missing: {token}")
if "FinalizePersistentClassificationCertificateJob" not in \
        classification_stage:
    fail("Initial classification certificate stage missing")

oracle = read(PIPE / "Kernels/ContactOracleKernel.cs")
if "IncrementalCacheState" in oracle or ".IsValid = 0" in oracle:
    fail("Oracle controls authoritative cache state")

verification = read(PHYSICS / "Diagnostics/VERIFICATION_MATRIX.md")
if "Unity required" not in verification:
    fail("Verification matrix overstates non-Unity checks")
print("Contact pipeline audit passed.")
