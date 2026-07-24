from pathlib import Path
import re

flow = Path('Entities/Unit/Systems/FlowField')
base = (flow / 'BaseFlowMovementSystem.cs').read_text(encoding='utf-8')
settings = Path('Entities/Unit/Components/FlowField/GridComponent.cs').read_text(encoding='utf-8')
helper = (flow / 'Diagnostics/Capture/BaseFlowMovementDiagnosticsLifecycle.cs').read_text(encoding='utf-8')
solver = (flow / 'Jobs/ContactPipeline/Core/SolveXpbdUnitContactsJob.cs').read_text(encoding='utf-8')
runtime = (flow / 'Diagnostics/Runtime/SimulationDebuggerRuntime.cs').read_text(encoding='utf-8')
snapshot = (flow / 'Diagnostics/Capture/PublishedSimulationDiagnosticsSnapshot.cs').read_text(encoding='utf-8')
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
if 'contactSolverSettings.EnableTimestepContactSetCache' not in base:
    raise SystemExit('Per-world timestep-cache setting is not used')
if 'EnableTimestepContactSetCache' not in settings:
    raise SystemExit('Per-world timestep-cache setting is missing from ECS settings')
if 'ContactPipelineRuntimeOptions' in base or (flow / 'Jobs/ContactPipeline/Core/ContactPipelineRuntimeOptions.cs').exists():
    raise SystemExit('Static contact-pipeline option authority returned')
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
if 'PublishedSimulationDiagnosticsRuntime' not in runtime or 'PublishedSimulationDiagnosticsRuntime' not in snapshot:
    raise SystemExit('Unified current-frame snapshot handoff missing')
if 'class SimulationDiagnosticsRingBuffer' in snapshot:
    raise SystemExit('RingBuffer is explicitly out of scope')
required_consumers = [
    flow / 'Diagnostics/Recording/IncrementalContactPipelineCsvRecorder.cs',
    flow / 'Diagnostics/Recording/SimulationDebuggerLocalRecorder.cs',
    flow / 'Diagnostics/Experiments/AdaptiveParameterTuner.cs',
    flow / 'Diagnostics/Experiments/AdaptiveParameterTuner.Scenarios.cs',
    flow / 'Diagnostics/Experiments/IncrementalContactPipelineExperimentRuntime.cs']
missing = [str(path) for path in required_consumers if not path.exists()]
if missing:
    raise SystemExit('Final diagnostics consumer ownership missing: ' + repr(missing))
