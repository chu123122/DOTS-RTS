from pathlib import Path

FLOW = Path("Entities/Unit/Systems/FlowField")
PIPE = FLOW / "Runtime/ContactPipeline"


def fail(message: str) -> None:
    raise SystemExit(message)


def read(path: Path) -> str:
    if not path.exists():
        fail(f"Missing diagnostics target: {path}")
    return path.read_text(encoding="utf-8")


def preprocess(source: str, enabled: bool) -> str:
    output, stack = [], []
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
        fail("Unbalanced diagnostics preprocessor directives")
    return "\n".join(output)

contracts = PIPE / "Observability/Contracts"
for name in ("ContactPipelineTelemetry.cs", "IncrementalContactPipelineDiagnostics.cs",
             "ContactDiagnosticContracts.cs", "SimulationDebuggerContracts.cs",
             "ParallelSimulationDebuggerPairCapture.cs"):
    read(contracts / name)

for path in FLOW.rglob("*.cs"):
    source = read(path).replace('"Stage3ContactDiagnostic/v3"', '""')
    if "Stage3" in source:
        fail(f"Retired Stage3 code identifier remains: {path}")
    if "Stage3" in path.name:
        fail(f"Retired Stage3 source filename remains: {path}")

runtime_source = "\n".join(read(path) for path in PIPE.rglob("*.cs"))
release = preprocess(runtime_source, False)
for token in ("Statistics.Value", "IncrementalStatistics.Value",
              "ParallelJacobiIterationTelemetry", "JacobiBlockTelemetry"):
    if token in release:
        fail(f"Diagnostics telemetry survives gameplay preprocessing: {token}")

resources = "\n".join(read(path) for path in (PIPE / "State").rglob("*.cs"))
release_resources = preprocess(resources, False)
for token in ("PersistentClassificationTelemetryState", "ContactHeatSample"):
    if token in release_resources:
        fail(f"Diagnostics allocation survives gameplay resources: {token}")

spatial = read(FLOW / "Diagnostics/Capture/SimulationDebuggerSpatialReadback.cs")
publishing = read(FLOW / "Diagnostics/Capture/SimulationDebuggerSnapshotPublishing.cs")
lifecycle = read(FLOW / "Diagnostics/Capture/BaseFlowMovementDiagnosticsLifecycle.cs")
if "internal static class SimulationDebuggerSpatialReadback" not in spatial:
    fail("Spatial readback still extends the simulation system owner")
for token in ("EntityManager entityManager", "NativeList<PersistentSweptProxy> proxies"):
    if token not in spatial:
        fail(f"Spatial readback dependency is hidden: {token}")
if "SimulationDebuggerSpatialReadback.Capture(" not in publishing:
    fail("Snapshot publication bypasses explicit spatial readback")
if "_candidateStore" in spatial:
    fail("Spatial readback reaches through the candidate-store owner")
if "SystemAPI.TryGetSingleton(out ContactDiagnosticSelection" in lifecycle:
    fail("Legacy diagnostic selection still assumes singleton ownership")
diagnostics_entity_creation = lifecycle.split(
    "_incrementalDiagnosticsEntity = EntityManager.CreateEntity(", 1)[1].split(
        ");", 1)[0]
if "ContactDiagnosticSelection" in diagnostics_entity_creation:
    fail("Per-system publication entity still owns the World selection control")

for path in (FLOW / "Diagnostics/Presentation", FLOW / "Diagnostics/Recording",
             FLOW / "Diagnostics/Experiments"):
    if not path.is_dir():
        fail(f"Diagnostics layer missing: {path}")
print("Contact diagnostics contracts passed.")
