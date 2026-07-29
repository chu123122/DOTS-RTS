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
for token in ("InteractionCandidateStore.Create(", "CrowdStepBodyResources.Create(",
              "InteractionCertificationFrameResources.Create(",
              "SoftAvoidanceFrameResources.Create(", "ConstraintSolverFrameResources.Create("):
    if token not in base:
        fail(f"Composition root wiring missing: {token}")

certifier = read(PIPE / "Stages/Certification/Prediction/InteractionCorrectnessCertifier.cs")
timestep = read(PIPE / "Stages/Certification/Prediction/TimestepContactSet.cs")
for token in ("IssueCertificateForCommittedViews(", "RevokeInteractionCertificate("):
    if token not in certifier:
        fail(f"Certification boundary missing: {token}")
for token in ("ResolveInteractionSource(", "CommitTimestepContactViews("):
    if token not in timestep:
        fail(f"Interaction view phase missing: {token}")

oracle = read(PIPE / "Stages/Certification/Validation/IncrementalContactOracle.cs")
if "IncrementalCacheState" in oracle or ".IsValid = 0" in oracle:
    fail("Oracle controls authoritative cache state")

verification = read(PHYSICS / "Diagnostics/VERIFICATION_MATRIX.md")
if "Unity required" not in verification:
    fail("Verification matrix overstates non-Unity checks")
print("Contact pipeline audit passed.")
