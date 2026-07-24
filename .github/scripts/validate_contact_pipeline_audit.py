from pathlib import Path
import re

flow = Path("Entities/Unit/Systems/FlowField")
solver = flow / "Jobs/ContactPipeline/Solver/ParallelJacobiSolver.cs"
p1p6 = flow / "Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs"
persistent = flow / "Jobs/ContactPipeline/Persistent/PersistentParallelClassificationP1P6.cs"
base = flow / "BaseFlowMovementSystem.cs"
resources = flow / "ContactPipelineResources.cs"
reset = flow / "Diagnostics/Capture/Jobs/ContactDiagnosticsCaptureLifecycle.cs"
snapshot = flow / "Diagnostics/Capture/PublishedSimulationDiagnosticsSnapshot.cs"
publishing = flow / "Diagnostics/Capture/SimulationDebuggerSnapshotPublishing.cs"
pipeline_snapshot = flow / "Diagnostics/Capture/IncrementalContactPipelineDiagnostics.cs"
runtime = flow / "Diagnostics/Runtime/SimulationDebuggerRuntime.cs"
experiment = flow / "Diagnostics/Experiments/IncrementalContactPipelineExperimentRuntime.cs"
contracts = flow / "Diagnostics/Runtime/SimulationDebuggerContracts.cs"
oracle = flow / "Diagnostics/Validation/IncrementalContactOracle.cs"
authoring = Path("Entities/Unit/Authoring/FlowField/FlowFieldManagerAuthoring.cs")
grid = Path("Entities/Unit/Components/FlowField/GridComponent.cs")
timestep = flow / "Jobs/ContactPipeline/Prediction/TimestepContactSet.cs"
verification = flow / "Diagnostics/VERIFICATION_MATRIX.md"
verification_meta = flow / "Diagnostics/VERIFICATION_MATRIX.md.meta"

required_paths = (solver, p1p6, persistent, base, resources, reset, snapshot, publishing,
                  pipeline_snapshot, runtime, experiment, contracts, oracle,
                  authoring, grid, timestep, verification, verification_meta)
for path in required_paths:
    if not path.exists():
        raise SystemExit(f"Missing audit target: {path}")


def preprocess_diagnostics(source: str, enabled: bool) -> str:
    output = []
    stack = []
    active = True
    for line in source.splitlines():
        directive = line.strip()
        if directive.startswith("#if "):
            symbol = directive[4:].strip()
            condition = enabled if symbol == "RTS_CONTACT_DIAGNOSTICS" else True
            stack.append((active, condition))
            active = active and condition
        elif directive == "#else":
            parent, condition = stack[-1]
            stack[-1] = (parent, not condition)
            active = parent and not condition
        elif directive == "#endif":
            active = stack.pop()[0]
        elif active:
            output.append(line)
    if stack:
        raise SystemExit("Unbalanced diagnostics preprocessor directives")
    return "\n".join(output)

solver_text = solver.read_text(encoding="utf-8")
p1p6_text = p1p6.read_text(encoding="utf-8")
reset_text = reset.read_text(encoding="utf-8")
for text, name in ((solver_text, "ParallelJacobi"), (p1p6_text, "P1P6")):
    init = re.search(r"private void Initialize.*?Pipeline\(.*?\n    \}", text, re.S)
    if init and any(token in init.group(0) for token in (
            "IterationDiagnostics.Clear()", "PairDiagnostics.Clear()",
            "SelectedBodyDiagnostic.Value")):
        raise SystemExit(f"{name} directly touches diagnostic Native containers")
if "#if RTS_CONTACT_DIAGNOSTICS" not in reset_text:
    raise SystemExit("Diagnostic reset lost compile-time boundary")

snapshot_text = snapshot.read_text(encoding="utf-8")
publishing_text = publishing.read_text(encoding="utf-8")
pipeline_snapshot_text = pipeline_snapshot.read_text(encoding="utf-8")
runtime_text = runtime.read_text(encoding="utf-8")
experiment_text = experiment.read_text(encoding="utf-8")
contracts_text = contracts.read_text(encoding="utf-8")

for required in (
        "CompletedSimulationStepMetadata", "CompletedStep",
        "frame.SimulationStepId!=stepId", "metadata.SimulationStepId!=stepId",
        "stepId<=state.LastPublishedStepId", "Frame => _frame.DeepCopy()"):
    combined = snapshot_text + pipeline_snapshot_text
    if required not in combined:
        raise SystemExit(f"Completed-step publication contract missing: {required}")
for forbidden in ("SystemAPI.Time", "_movementQuery.CalculateEntityCount()",
                  "BuildEffectiveSettings("):
    if forbidden in publishing_text:
        raise SystemExit(f"Completed frame rebuilt from next update state: {forbidden}")
for forbidden in ("SlotA", "SlotB", "AcquireWriteSlot", "PublishFrame(",
                  "PublishPipeline(", "class SimulationDiagnosticsRingBuffer"):
    if forbidden in snapshot_text:
        raise SystemExit(f"Mutable/partial/history publication returned: {forbidden}")

for required in (
        "Dictionary<ulong, WorldState>", "CaptureMaskFor(ulong worldId)",
        "TryConsumeSettingsRequest(ulong worldId", "TryConsumeContactCacheReset(ulong worldId)",
        "Dictionary<ulong,WorldPublicationState>", "TryGetLatest(ulong worldId",
        "Dictionary<ulong,OverrideState>", "Apply(ulong worldId"):
    if required not in runtime_text + snapshot_text + experiment_text:
        raise SystemExit(f"Per-World diagnostics contract missing: {required}")
if "public static bool TimestepContactSetCacheEnabled { get; set; }" in runtime_text:
    raise SystemExit("Obsolete gameplay setting retained a global backing field")

release_source = "\n".join(preprocess_diagnostics(path.read_text(encoding="utf-8"), False)
                           for path in (base, solver, p1p6, persistent))
for forbidden in (
        "ParallelJacobiIterationTelemetry", "JacobiBlockTelemetry",
        "new NativeReference<ParallelJacobiIterationTelemetry>",
        "new NativeList<JacobiBlockTelemetry>", "ReduceParallelJacobiBlocksJob",
        "ReduceSoftAvoidanceBlocksJob", "FinalizeP1P6SoftAvoidanceJob",
        "BeginP1P6FinalizeSubstepJob", "ReduceP1P6VelocityBodyBlocksJob",
        "FinalizeP1P6VelocityStatisticsJob"):
    if forbidden in release_source:
        raise SystemExit(f"Gameplay Jacobi telemetry call graph returned: {forbidden}")
if re.search(r"public long (?:Build|Classification)StartTimestamp;", release_source):
    raise SystemExit("Persistent classification timestamps occupy gameplay state")

oracle_text = oracle.read_text(encoding="utf-8")
if "IncrementalCacheState" in oracle_text or ".IsValid = 0" in oracle_text:
    raise SystemExit("Diagnostics oracle still controls gameplay cache")

all_source = "\n".join(
    path.read_text(encoding="utf-8")
    for root in (flow, Path("Entities/Unit/Components/FlowField"),
                 Path("Entities/Unit/Authoring/FlowField"))
    if root.exists()
    for path in root.rglob("*.cs"))
if "ContactPipelineRuntimeOptions" in all_source:
    raise SystemExit("Static gameplay configuration authority returned")
if "EnableTimestepContactSetCache" not in grid.read_text(encoding="utf-8"):
    raise SystemExit("Per-World timestep-cache setting missing")

authoring_text = authoring.read_text(encoding="utf-8")
for forbidden in (
        "AddComponent(entity, new PredictiveDiscContactStatistics",
        "AddComponent(entity, new Stage3SelectedBodyDiagnostic",
        "AddBuffer<Stage3ContactIterationDiagnostic>",
        "AddBuffer<Stage3ContactPairDiagnostic>",
        "AddBuffer<Stage3ContactHeatSample>"):
    if forbidden in authoring_text:
        raise SystemExit(f"Gameplay Baker still owns diagnostics ECS state: {forbidden}")

timestep_text = timestep.read_text(encoding="utf-8")
for old in ("BuildTimestepContactSet", "FinalizeTimestepContactView",
            "CommitFinalizedTimestepContactView"):
    if old in timestep_text:
        raise SystemExit(f"Legacy contact-view stage returned: {old}")
for required in ("BuildOrRefreshTimestepContactViews", "ClassifyTimestepContacts",
                 "CommitTimestepContactViews"):
    if required not in timestep_text:
        raise SystemExit(f"Contact-view stage missing: {required}")

if "Unity required" not in verification.read_text(encoding="utf-8"):
    raise SystemExit("Verification matrix no longer distinguishes unexecuted Unity tests")
if "guid:" not in verification_meta.read_text(encoding="utf-8"):
    raise SystemExit("Verification matrix Unity metadata is invalid")


# P2 maintenance boundaries: resource ownership, explicit stages and terminology.
base_text = base.read_text(encoding="utf-8")
resources_text = resources.read_text(encoding="utf-8")
for required in ("struct ContactPersistentState", "struct ContactFrameResources",
                 "ContactFrameResources Create(", "JobHandle Dispose("):
    if required not in resources_text:
        raise SystemExit(f"Contact resource lifetime owner missing: {required}")
if re.search(r"new Native(?:Array|List|Reference|Parallel)", base_text):
    raise SystemExit("BaseFlowMovementSystem again allocates contact Native resources directly")
if "DirtyBodyBlockOffsets" not in resources_text or "DirtyBodyBlockOffsets" not in p1p6_text:
    raise SystemExit("Dirty-body block scratch is not independently owned")
for forbidden in ("BlockOffsetsAndCounts = SoftIncidentWriteCursors",
                  "BlockOffsets = SoftIncidentWriteCursors",
                  "EscapeCountsByBlock = SoftIncidentWriteCursors"):
    if forbidden in p1p6_text:
        raise SystemExit(f"Soft incident cursor reused across phase semantics: {forbidden}")

persistent_text = persistent.read_text(encoding="utf-8")
release_maintenance = preprocess_diagnostics(resources_text + "\n" + persistent_text, False)
for required in ("struct PersistentClassificationPhaseState",
                 "struct PersistentClassificationTelemetryState",
                 "RefreshPersistentPairSourceForClassification"):
    if required not in persistent_text:
        raise SystemExit(f"Persistent classification boundary missing: {required}")
if "PersistentClassificationTelemetryState" in release_maintenance:
    raise SystemExit("Persistent classification telemetry exists in gameplay preprocessing")
if "ParallelPersistentClassificationState" in persistent_text + resources_text + p1p6_text + base_text:
    raise SystemExit("Mixed persistent classification state returned")

if solver_text.count("XpbdContactConstraintMath.Evaluate(") != 1:
    raise SystemExit("Parallel Jacobi pair math is duplicated outside EvaluateJacobiPair")
if solver_text.count("EvaluateJacobiPair(") < 3:
    raise SystemExit("Both parallel Jacobi jobs do not share EvaluateJacobiPair")

for required in ("ContactViewBuildResult", "ResolveInteractionSource(",
                 "ObserveContactViewBuildResult(", "ClassifyTimestepContacts(",
                 "CommitTimestepContactViews("):
    if required not in timestep_text:
        raise SystemExit(f"Explicit contact-view phase/result missing: {required}")
commit_match = re.search(
    r"private void CommitTimestepContactViews\(.*?\n    \}", timestep_text, re.S)
if not commit_match:
    raise SystemExit("Cannot locate CommitTimestepContactViews")
if "ValidateIncrementalContactSetAgainstQuadraticOracle" in commit_match.group(0):
    raise SystemExit("Oracle validation is hidden inside the commit phase")

if "enum StagedContactPipelinePhase" not in p1p6_text:
    raise SystemExit("Historical P1-P6 scheduling lacks named phase documentation")
source_without_migration_attributes = all_source.replace(
    '[FormerlySerializedAs("enableFatAabbCache")]', '').replace(
    '[FormerlySerializedAs("fatAabbCacheMargin")]', '')
for forbidden in ("EnableFatAabbCache", "FatAabbCacheMargin",
                  "enableFatAabbCache", "fatAabbCacheMargin"):
    if forbidden in source_without_migration_attributes:
        raise SystemExit(f"Legacy persistent-cache terminology returned: {forbidden}")
for required in ("EnablePersistentContactCache", "PersistentGuardEnvelopeMargin"):
    if required not in grid.read_text(encoding="utf-8"):
        raise SystemExit(f"Persistent contact setting missing: {required}")
