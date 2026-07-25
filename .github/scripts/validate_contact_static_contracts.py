from pathlib import Path
import re

FLOW = Path("Entities/Unit/Systems/FlowField")
PIPE = FLOW / "Runtime/ContactPipeline"
WORKFLOWS = Path(".github/workflows")


def fail(message: str) -> None:
    raise SystemExit(message)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")

sources = {path: read(path) for path in FLOW.rglob("*.cs")}
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

for path, text in sources.items():
    if text.count("{") != text.count("}"):
        fail(f"Brace mismatch: {path}")
    meta = Path(str(path) + ".meta")
    if not meta.exists() or "guid:" not in read(meta):
        fail(f"Missing Unity source metadata: {path}")

if (FLOW / "Jobs/ContactPipeline").exists():
    fail("Legacy ContactPipeline physical root returned")
for path in FLOW.glob("*Resources.cs"):
    if path.name in {"ContactPipelineResources.cs", "ContactPipelineExecutionResources.cs",
                     "CrowdStepBodyResources.cs", "ConstraintSolverFrameResources.cs",
                     "InteractionCertificationFrameResources.cs", "SoftAvoidanceFrameResources.cs"}:
        fail(f"Contact resource owner remains flat: {path}")

parallel_schedule = read(PIPE / "Scheduling/Parallel/ParallelContactPipelineScheduler.cs")
parallel_jobs = read(PIPE / "Scheduling/Parallel/Jobs/ParallelContactPipelineJobs.cs")
if "enum StagedContactPipelinePhase" not in read(
        PIPE / "Contracts/Execution/ParallelPipelineExecutionContracts.cs"):
    fail("Named staged-pipeline phase contract missing")
for token in ("PrepareTimestepPredictionBodiesJob", "EvaluateSoftAvoidancePairsJob"):
    if token not in parallel_jobs or token not in parallel_schedule:
        fail(f"Parallel schedule/job linkage missing: {token}")

for pattern in ("refactor-contact-pipeline-phase*.yml", "audit-*.yml",
                "apply-hard-cutover*.yml", "diagnose-*.yml"):
    if list(WORKFLOWS.glob(pattern)):
        fail(f"Temporary workflow remains: {pattern}")
print("Contact pipeline terminal static contracts passed.")
