from pathlib import Path
import re

flow = Path("Entities/Unit/Systems/FlowField")
solver = flow / "Jobs/ContactPipeline/Solver/ParallelJacobiSolver.cs"
p1p6 = flow / "Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs"
reset = flow / "Diagnostics/Capture/Jobs/ContactDiagnosticsCaptureLifecycle.cs"
snapshot = flow / "Diagnostics/Capture/PublishedSimulationDiagnosticsSnapshot.cs"
contracts = flow / "Diagnostics/Runtime/SimulationDebuggerContracts.cs"
oracle = flow / "Diagnostics/Validation/IncrementalContactOracle.cs"
authoring = Path("Entities/Unit/Authoring/FlowField/FlowFieldManagerAuthoring.cs")
grid = Path("Entities/Unit/Components/FlowField/GridComponent.cs")
timestep = flow / "Jobs/ContactPipeline/Prediction/TimestepContactSet.cs"

for path in (solver, p1p6, reset, snapshot, contracts, oracle, authoring, grid, timestep):
    if not path.exists():
        raise SystemExit(f"Missing audit target: {path}")

solver_text = solver.read_text(encoding="utf-8")
p1p6_text = p1p6.read_text(encoding="utf-8")
reset_text = reset.read_text(encoding="utf-8")
for text, name in ((solver_text, "ParallelJacobi"), (p1p6_text, "P1P6")):
    init = re.search(r"private void Initialize.*?Pipeline\(.*?\n    \}", text, re.S)
    if init and any(token in init.group(0) for token in (
            "IterationDiagnostics.Clear()",
            "PairDiagnostics.Clear()",
            "SelectedBodyDiagnostic.Value")):
        raise SystemExit(f"{name} directly touches diagnostic Native containers")
if "#if RTS_CONTACT_DIAGNOSTICS" not in reset_text:
    raise SystemExit("Diagnostic reset lost compile-time boundary")

snapshot_text = snapshot.read_text(encoding="utf-8")
if "public static class PublishedSimulationDiagnosticsRuntime" not in snapshot_text:
    raise SystemExit("Published diagnostics runtime has an invalid declaration")
if "PublishedPublishedSimulationDiagnosticsRuntime" in snapshot_text:
    raise SystemExit("Duplicated Published prefix returned")
for forbidden in ("SlotA", "SlotB", "AcquireWriteSlot", "PublishFrame(", "PublishPipeline("):
    if forbidden in snapshot_text:
        raise SystemExit(f"Mutable/partial snapshot publication returned: {forbidden}")
for required in ("PublishComplete(", "new PublishedSimulationDiagnosticsSnapshot(", "DeepCopy()"):
    if required not in snapshot_text:
        raise SystemExit(f"Immutable snapshot contract missing: {required}")
if "class SimulationDiagnosticsRingBuffer" in snapshot_text:
    raise SystemExit("RingBuffer is outside this refactor")

oracle_text = oracle.read_text(encoding="utf-8")
if "IncrementalCacheState" in oracle_text or ".IsValid = 0" in oracle_text:
    raise SystemExit("Diagnostics oracle still controls gameplay cache")

all_source = "\n".join(
    p.read_text(encoding="utf-8")
    for root in (flow, Path("Entities/Unit/Components/FlowField"),
                 Path("Entities/Unit/Authoring/FlowField"))
    if root.exists()
    for p in root.rglob("*.cs"))
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
for required in ("BuildOrRefreshTimestepContactViews",
                 "ClassifyTimestepContacts",
                 "CommitTimestepContactViews"):
    if required not in timestep_text:
        raise SystemExit(f"Contact-view stage missing: {required}")
