from pathlib import Path
import re

FLOW = Path("Entities/Unit/Systems/FlowField")
PIPELINE = FLOW / "Jobs/ContactPipeline"
WORKFLOWS = Path(".github/workflows")


def fail(message: str) -> None:
    raise SystemExit(message)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


sources = {path: read(path) for path in FLOW.rglob("*.cs")}
production = "\n".join(sources.values())

legacy_symbols = (
    "BuildAdaptiveFatAabbHotspots",
    "BuildAdaptiveHybridContactPairs",
    "BuildContactPairsFromFatAabbCache",
    "EnsureFatAabbRawCandidates",
    "UpdateAdaptiveFatAabbHistoryAfterSolve",
    "AreCorrectedDiscsInsideFatCache",
    "ShadowStatistics",
    "LegacyBroadPhaseStatistics",
    "LegacyBroadPhaseSource",
    "MappedFatCachePairs",
    "FlowMovementFrameState",
    "UnitCollisionPair",
    "SolveXpbdUnitContactsJob",
    "BaseFlowMovementComposition",
    "ContactFrameResources",
    "ContactPersistentState",
)
returned = [
    token for token in legacy_symbols
    if re.search(rf"\b{re.escape(token)}\b", production)
]
if returned:
    fail("Retired contact symbols returned: " + repr(returned))

forbidden_paths = (
    FLOW / "Jobs/Legacy",
    FLOW / "Jobs/Compatibility",
    FLOW / "Jobs/Compatibility.meta",
    FLOW / "Jobs/FlowMovementFrameState.cs",
    FLOW / "Jobs/CalculateIndependentFlowForceJob.cs",
    FLOW / "BaseFlowMovementComposition.cs",
    FLOW / "ContactPipelineResources.cs",
    PIPELINE / "Core/SolveXpbdUnitContactsJob.cs",
    PIPELINE / "Core/ContactPairTypes.cs",
    PIPELINE / "Core/CrowdEnvironmentAccess.cs",
)
present = [str(path) for path in forbidden_paths if path.exists()]
if present:
    fail("Retired files returned: " + repr(present))

configuration_path = PIPELINE / "Core/ContactPipelineConfiguration.cs"
configuration_text = read(configuration_path)
required_margin_contract = (
    "public float GuardEnvelopeMargin;",
    "GuardEnvelopeMargin = solverSettings.PersistentGuardEnvelopeMargin",
)
missing = [
    token for token in required_margin_contract
    if token not in configuration_text
]
if missing:
    fail("Normalized guard margin contract missing: " + repr(missing))

for runtime_root in (
        PIPELINE / "Motion",
        PIPELINE / "SoftAvoidance",
        PIPELINE / "Solver",
        PIPELINE / "Prediction"):
    escaped = [
        path for path in runtime_root.rglob("*.cs")
        if "PersistentGuardEnvelopeMargin" in read(path)
    ]
    if escaped:
        fail(
            "Serialized guard margin escaped normalized runtime config: " +
            repr(escaped))

if re.search(
        r"\b(?:SleepingBody|SleepingIsland|ContactIslandSleeping)\b",
        production):
    fail("Sleeping policy introduced without a dedicated design stage")

ownership = {
    r"\bpublic\s+struct\s+BodyPair\b": PIPELINE / "Core/BodyPair.cs",
    r"\bpublic\s+struct\s+ContactConstraint\b":
        PIPELINE / "Core/ContactConstraint.cs",
    r"\bpublic\s+struct\s+SweptDiscCellEntry\b":
        PIPELINE / "BroadPhase/SweptDiscTypes.cs",
    r"\bpublic\s+struct\s+IncrementalContactCacheState\b":
        PIPELINE / "Persistent/IncrementalPredictiveContactTypes.cs",
    r"\bpublic\s+struct\s+IncrementalContactPipelineStatistics\b":
        FLOW / "Diagnostics/Capture/ContactPipelineTelemetry.cs",
}
for pattern, expected_path in ownership.items():
    paths = [
        path for path, text in sources.items()
        if re.search(pattern, text)
    ]
    if paths != [expected_path]:
        fail(f"{pattern} ownership mismatch: {paths}")

solver_math = read(PIPELINE / "Solver/XpbdContactSolver.cs")
internal_tokens = (
    "internal struct ContactConstraintEvaluation",
    "internal static class XpbdContactConstraintMath",
    "internal static ContactConstraintEvaluation Evaluate(",
)
missing = [token for token in internal_tokens if token not in solver_math]
if missing:
    fail("Solver-local evaluation accessibility mismatch: " + repr(missing))

runtime_types = read(
    PIPELINE / "Persistent/IncrementalPredictiveContactTypes.cs")
telemetry = read(FLOW / "Diagnostics/Capture/ContactPipelineTelemetry.cs")
if "Nanoseconds" in runtime_types or "OraclePairCount" in runtime_types:
    fail("Telemetry leaked into authoritative runtime state")
if "IncrementalContactCacheState" in telemetry:
    fail("Runtime cache state leaked into telemetry schema")

temporary_patterns = (
    "refactor-contact-pipeline-phase*.yml",
    "audit-*.yml",
    "apply-hard-cutover*.yml",
    "diagnose-*.yml",
    "export-cutover-source.yml",
    "fix-contact-*.yml",
    "update-parallel-jacobi-contract.yml",
)
temporary = []
for pattern in temporary_patterns:
    temporary.extend(WORKFLOWS.glob(pattern))
if temporary:
    fail("Temporary workflows remain: " + repr(sorted(map(str, temporary))))

expected_diagnostics = (
    FLOW / "Diagnostics/Capture/ContactPipelineTelemetry.cs",
    FLOW / "Diagnostics/Capture/IncrementalContactPipelineDiagnostics.cs",
    FLOW / "Diagnostics/Capture/SimulationDebuggerSnapshotPublishing.cs",
    FLOW / "Diagnostics/Capture/Jobs/PublishPredictiveDiscContactStatisticsJob.cs",
    FLOW / "Diagnostics/Capture/Jobs/Stage3ContactDiagnosticRecorder.cs",
    FLOW / "Diagnostics/Instrumentation/ContactPipelineProfilerClock.cs",
    FLOW / "Diagnostics/Validation/IncrementalContactOracle.cs",
    FLOW / "Diagnostics/README.md",
)
missing = [str(path) for path in expected_diagnostics if not path.exists()]
if missing:
    fail("Diagnostics ownership files missing: " + repr(missing))

legacy_diagnostics = (
    FLOW / "Diagnostics/ContactPipelineTelemetry.cs",
    FLOW / "Diagnostics/IncrementalContactPipelineDiagnostics.cs",
    FLOW / "Diagnostics/SimulationDebuggerSnapshotPublishing.cs",
    FLOW / "Jobs/PublishPredictiveDiscContactStatisticsJob.cs",
    FLOW / "Jobs/Stage3ContactDiagnosticRecorder.cs",
    FLOW / "Jobs/IncrementalContactOracle.cs",
    PIPELINE / "Core/ContactPipelineProfilerClock.cs",
)
returned = [str(path) for path in legacy_diagnostics if path.exists()]
if returned:
    fail("Diagnostics files returned to runtime folders: " + repr(returned))

csv_text = read(
    FLOW / "Diagnostics/Recording/IncrementalContactPipelineCsvRecorder.cs")
if "CsvSchemaVersion = 7" not in csv_text:
    fail("Unexpected contact CSV schema")
if "LegacyCacheUseCount" in csv_text or "LegacyBroadPhaseStatistics" in csv_text:
    fail("Legacy CSV columns returned")

solver = read(PIPELINE / "Solver/ParallelJacobiSolver.cs")
p1p6 = read(PIPELINE / "Solver/ParallelContactPipelineP1P6.cs")
resources = read(FLOW / "ConstraintSolverFrameResources.cs")
stage_jobs = read(PIPELINE / "Core/ContactPipelineStageJobs.cs")
base = read(FLOW / "BaseFlowMovementSystem.cs")
contracts = {
    "parallel solver": (
        solver,
        (
            "IJobParallelForDefer",
            "EvaluateParallelJacobiPairsJob",
            "GatherAndApplyParallelJacobiBodiesJob",
        )),
    "P1-P6 scheduler": (
        p1p6,
        (
            "ScheduleParallelJacobiP1P6",
            "EvaluateParallelJacobiPairsWithDiagnosticsJob",
            "ParallelSimulationDebuggerPairCandidates",
            "CountParallelSimulationDebuggerPairBlocksJob",
            "PrefixParallelSimulationDebuggerPairsJob",
            "ScatterParallelSimulationDebuggerPairsJob",
            "ConstraintSolverOperation.MergeParallelDebuggerPairs",
        )),
    "solver resources": (
        resources,
        (
            "ActiveIncidentOffsets",
            "ActiveIncidentPairIndices",
            "JacobiPairCorrections",
        )),
    "constraint stage": (
        stage_jobs,
        ("MergeParallelDebuggerPairs", "ConstraintSolverOperation")),
}
missing = []
for owner, (text, tokens) in contracts.items():
    missing.extend(
        f"{owner}: {token}" for token in tokens if token not in text)
if missing:
    fail("Parallel Jacobi contract missing: " + repr(missing))
if "Interlocked" in solver or "Atomic" in solver:
    fail("Parallel Jacobi must not use floating-point atomics")
if "requiresSerialJacobiCapture" in base:
    fail("Selected-pair capture must not change the Jacobi backend")
if ("bool useParallelJacobi = usesJacobiScratch;" not in base or
        "captureParallelSelectedPairs" not in base):
    fail("Selected-pair capture must use the parallel pair-slot path")

editor_settings = FLOW / "Editor/SimulationDiagnosticsBuildSettings.cs"
compile_status = FLOW / "Diagnostics/SimulationDiagnosticsCompileStatus.cs"
asset = FLOW / "Editor/SimulationDiagnosticsBuildSettings.asset"
for path in (editor_settings, compile_status, asset):
    if not path.exists():
        fail(f"Missing diagnostics build artifact: {path}")
editor_text = read(editor_settings)
required = (
    "RTS_CONTACT_DIAGNOSTICS",
    "PlayerSettings.GetScriptingDefineSymbols",
    "PlayerSettings.SetScriptingDefineSymbols",
    "Editor Diagnostics",
    "Development Diagnostics",
    "Release Gameplay Only",
)
missing = [token for token in required if token not in editor_text]
if missing:
    fail("Diagnostics build settings contract missing: " + repr(missing))
if "UnityEditor" in read(compile_status):
    fail("Runtime compile status must not reference UnityEditor")

structure_roots = (
    PIPELINE,
    FLOW / "Diagnostics/Capture",
    FLOW / "Diagnostics/Instrumentation",
    FLOW / "Diagnostics/Validation",
)
for source_root in structure_roots:
    for path in source_root.rglob("*.cs"):
        text = read(path)
        if text.count("{") != text.count("}"):
            fail(f"Brace mismatch: {path}")

print("Contact pipeline terminal static contracts passed.")
