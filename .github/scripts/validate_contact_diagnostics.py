from pathlib import Path
import re

flow = Path('Entities/Unit/Systems/FlowField')
base = (flow / 'BaseFlowMovementSystem.cs').read_text(encoding='utf-8')
helper = (flow / 'Diagnostics/Capture/BaseFlowMovementDiagnosticsScheduling.cs').read_text(encoding='utf-8')
solver = (flow / 'Jobs/ContactPipeline/Core/SolveXpbdUnitContactsJob.cs').read_text(encoding='utf-8')
runtime = (flow / 'Diagnostics/Runtime/SimulationDebuggerRuntime.cs').read_text(encoding='utf-8')
snapshot = (flow / 'Diagnostics/Capture/SimulationDiagnosticsSnapshot.cs').read_text(encoding='utf-8')
forbidden_allocations = (
    'new NativeReference<PredictiveDiscContactStatistics>(Allocator.TempJob)',
    'new NativeReference<IncrementalContactPipelineStatistics>(Allocator.TempJob)',
    'new NativeList<Stage3ContactIterationDiagnostic>',
    'new NativeArray<Stage3ContactHeatSample>')
leaked = [token for token in forbidden_allocations if token in base]
if leaked:
    raise SystemExit('Gameplay scheduler still owns diagnostics allocation: ' + repr(leaked))
if 'SimulationDebuggerRuntime.TimestepContactSetCacheEnabled' in base:
    raise SystemExit('Gameplay option authority leaked into debugger runtime')
if 'ContactPipelineRuntimeOptions.TimestepContactSetCacheEnabled' not in base:
    raise SystemExit('Authoritative contact option is not used')
if '#if RTS_CONTACT_DIAGNOSTICS' not in helper:
    raise SystemExit('Diagnostics scheduling context lost compile boundary')
if '_contactTelemetry.Value' not in solver or '_incrementalTelemetry.Value' not in solver:
    raise SystemExit('Telemetry accessors missing private diagnostic backing storage')
production = '\n'.join(
    path.read_text(encoding='utf-8')
    for path in (flow / 'Jobs/ContactPipeline').rglob('*.cs'))
if re.search(r'(?m)^\s*Statistics\.Value', production):
    raise SystemExit('Direct contact telemetry ABI returned')
if 'IncrementalStatistics.Value' in production:
    raise SystemExit('Direct incremental telemetry ABI returned')
if 'SimulationDiagnosticsSnapshotRuntime' not in runtime or 'SimulationDiagnosticsSnapshotRuntime' not in snapshot:
    raise SystemExit('Unified current-frame snapshot handoff missing')
if 'class SimulationDiagnosticsRingBuffer' in snapshot:
    raise SystemExit('RingBuffer is explicitly out of scope')
