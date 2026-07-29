from pathlib import Path
import re

FLOW = Path("Gameplay/Entities/Unit/Systems/FlowField")
PHYSICS = Path("Physics")
PIPE = PHYSICS / "ContactPipeline"
WORKFLOWS = Path(".github/workflows")


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

for path, text in sources.items():
    if text.count("{") != text.count("}"):
        fail(f"Brace mismatch: {path}")
    meta = Path(str(path) + ".meta")
    if not meta.exists() or "guid:" not in read(meta):
        fail(f"Missing Unity source metadata: {path}")

if (PHYSICS / "Jobs/ContactPipeline").exists():
    fail("Legacy ContactPipeline physical root returned")
for path in FLOW.glob("*Resources.cs"):
    if path.name in {"ContactPipelineResources.cs", "ContactPipelineExecutionResources.cs",
                     "CrowdStepBodyResources.cs", "ConstraintSolverFrameResources.cs",
                     "InteractionCertificationFrameResources.cs", "SoftAvoidanceFrameResources.cs"}:
        fail(f"Contact resource owner remains flat: {path}")

parallel_schedule = read(PIPE / "Scheduling/SharedContactPipelineScheduler.cs")
parallel_jobs = read(PIPE / "Scheduling/Parallel/Jobs/ParallelContactStageJobs.cs")
dirty_refresh_jobs = read(
    PIPE / "Stages/Certification/SubstepRepair/DirtyBodyRefreshStageJobs.cs")
dirty_contact_jobs = read(
    PIPE / "Stages/Certification/SubstepRepair/DirtyContactScheduleStageJobs.cs")
full_sweep_jobs = read(
    PIPE / "Stages/Certification/InitialContact/FullSweepBroadPhaseStageJobs.cs")
if "enum StagedContactPipelinePhase" not in read(
        PIPE / "Contracts/Execution/ContactPipelineStageContracts.cs"):
    fail("Named staged-pipeline phase contract missing")
for token in ("PrepareTimestepPredictionBodiesJob", "EvaluateSoftAvoidancePairsJob"):
    if token not in parallel_jobs or token not in parallel_schedule:
        fail(f"Parallel schedule/job linkage missing: {token}")

repair_prediction_job = parallel_jobs.split(
    "internal struct PrepareRepairPredictionBodiesJob", 1)[1].split(
        "internal struct InitializeSoftAvoidanceBodiesJob", 1)[0]
if "int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;" not in repair_prediction_job:
    fail("Incremental repair prediction no longer maps compact dirty indices to bodies")
for element_type, field_name in (
    ("CrowdBodySnapshot", "Bodies"),
    ("CrowdNavigationState", "NavigationStates"),
    ("CrowdMotionIntent", "MotionIntents"),
    ("CrowdMotionEvidence", "MotionEvidence"),
    ("CrowdBodyStepState", "StepStates"),
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
    "CreateAlgorithms(",
    "InteractionCertificationJob.cs",
    "InteractionCertificationStageJobs.cs",
):
    if retired in all_source:
        fail(f"Retired certification facade returned: {retired}")

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
    "CountBodyCellsJob : IJobParallelFor",
    "PrefixBodyCellsJob : IJob",
    "ScatterBodyCellsJob : IJobParallelFor",
    "CountCellPairsJob : IJobParallelForDefer",
    "PrefixCellPairsJob : IJob",
    "ScatterCellPairsJob : IJobParallelForDefer",
    "SortAndDeduplicateBroadPhasePairsJob : IJob",
):
    if token not in full_sweep_jobs or token.split(" :")[0] not in parallel_schedule:
        fail(f"Full sweep broad-phase chain missing: {token}")

for chain in (
    ("RefreshDirtyBodiesJob", "ReduceDirtyBodyRefreshJob"),
    ("CountDirtyContactScheduleJob", "PrefixDirtyContactScheduleJob",
     "ScatterDirtyContactScheduleJob"),
    ("CountBodyCellsJob", "PrefixBodyCellsJob", "ScatterBodyCellsJob",
     "CountCellPairsJob", "PrefixCellPairsJob", "ScatterCellPairsJob",
     "SortAndDeduplicateBroadPhasePairsJob"),
):
    positions = [parallel_schedule.find(token) for token in chain]
    if positions != sorted(positions) or any(position < 0 for position in positions):
        fail(f"Parallel stage order mismatch: {' -> '.join(chain)}")

certification_source = "\n".join(
    text for path, text in sources.items()
    if PIPE / "Stages/Certification" in path.parents)
if re.search(
        r"=>\s*(?:Environment|Body|Views|Persistent|Solver|Diagnostics|Configuration)\.",
        certification_source):
    fail("Certification resource forwarding property returned")

for group in (
    "CertificationEnvironment", "CertificationBody", "CertificationViews",
    "CertificationPersistent", "CertificationSolver",
    "CertificationDiagnostics",
):
    if f"public " not in read(
            PIPE / "Scheduling/CrowdContactPipelineScheduler.cs") or \
            group not in read(PIPE / "Scheduling/CrowdContactPipelineScheduler.cs"):
        fail(f"Scheduler does not own resource group directly: {group}")

for pattern in ("refactor-contact-pipeline-phase*.yml", "audit-*.yml",
                "apply-hard-cutover*.yml", "diagnose-*.yml"):
    if list(WORKFLOWS.glob(pattern)):
        fail(f"Temporary workflow remains: {pattern}")
print("Contact pipeline terminal static contracts passed.")
