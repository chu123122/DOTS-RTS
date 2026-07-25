from pathlib import Path
import re

flow = Path('Entities/Unit/Systems/FlowField')
pipe = flow / 'Jobs/ContactPipeline'
base = (flow / 'BaseFlowMovementSystem.cs').read_text(encoding='utf-8')
settings = Path('Entities/Unit/Components/FlowField/GridComponent.cs').read_text(encoding='utf-8')
helper = (flow / 'Diagnostics/Capture/BaseFlowMovementDiagnosticsLifecycle.cs').read_text(encoding='utf-8')
runtime = (flow / 'Diagnostics/Runtime/SimulationDebuggerRuntime.cs').read_text(encoding='utf-8')
snapshot = (flow / 'Diagnostics/Capture/PublishedSimulationDiagnosticsSnapshot.cs').read_text(encoding='utf-8')
resources = (flow / 'ContactPipelineResources.cs').read_text(encoding='utf-8')


def preprocess_diagnostics(source: str, enabled: bool) -> str:
    output = []
    stack = []
    active = True
    for line in source.splitlines():
        directive = line.strip()
        if directive.startswith('#if '):
            symbol = directive[4:].strip()
            condition = enabled if symbol == 'RTS_CONTACT_DIAGNOSTICS' else True
            stack.append((active, condition))
            active = active and condition
        elif directive == '#else':
            parent, condition = stack[-1]
            stack[-1] = (parent, not condition)
            active = parent and not condition
        elif directive == '#endif':
            active = stack.pop()[0]
        elif active:
            output.append(line)
    return '\n'.join(output)

base_gameplay = preprocess_diagnostics(base, False)
forbidden_allocations = (
    'new NativeReference<PredictiveDiscContactStatistics>(Allocator.TempJob)',
    'new NativeReference<IncrementalContactPipelineStatistics>(Allocator.TempJob)',
    'new NativeList<Stage3ContactIterationDiagnostic>',
    'new NativeArray<Stage3ContactHeatSample>')
leaked = [token for token in forbidden_allocations if token in base_gameplay]
if leaked:
    raise SystemExit('Gameplay scheduler still owns diagnostics allocation: ' + repr(leaked))
if 'SimulationDebuggerRuntime.TimestepContactSetCacheEnabled' in base:
    raise SystemExit('Gameplay option authority leaked into debugger runtime')
if 'contactSolverSettings.EnableTimestepContactSetCache' not in base:
    raise SystemExit('Per-world timestep-cache setting is not used')
if 'EnableTimestepContactSetCache' not in settings:
    raise SystemExit('Per-world timestep-cache setting is missing from ECS settings')
if 'ContactPipelineRuntimeOptions' in base or (pipe / 'Core/ContactPipelineRuntimeOptions.cs').exists():
    raise SystemExit('Static contact-pipeline option authority returned')
if '#if RTS_CONTACT_DIAGNOSTICS' not in helper:
    raise SystemExit('Diagnostics scheduling context lost compile boundary')

production = '\n'.join(
    path.read_text(encoding='utf-8')
    for path in pipe.rglob('*.cs'))
release_production = preprocess_diagnostics(production, False)
for token in ('Statistics.Value', 'IncrementalStatistics.Value'):
    if token in release_production:
        raise SystemExit('Diagnostics telemetry access survives gameplay preprocessing: ' + token)
for required in (
    'public partial struct InteractionCertificationJob : IJob',
    'public partial struct SoftAvoidanceJob : IJob',
    'public partial struct ConstraintSolverJob : IJob'):
    if required not in production:
        raise SystemExit('Focused diagnostics stage boundary missing: ' + required)
if 'SolveXpbdUnitContactsJob' in production:
    raise SystemExit('Retired solver telemetry ABI returned')

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

release_parallel = preprocess_diagnostics('\n'.join(
    path.read_text(encoding='utf-8')
    for path in (pipe / 'Solver').rglob('*.cs')), False)
for token in ('ParallelJacobiIterationTelemetry', 'JacobiBlockTelemetry',
              'ReduceParallelJacobiBlocksJob', 'ReduceP1P6VelocityBodyBlocksJob'):
    if token in release_parallel:
        raise SystemExit('Gameplay telemetry scheduling returned: ' + token)

release_resources = preprocess_diagnostics(resources, False)
if 'PersistentClassificationTelemetryState' in release_resources:
    raise SystemExit('Persistent classification telemetry is allocated in gameplay resources')
if 'DirtyBodyBlockOffsets' not in resources:
    raise SystemExit('Dedicated dirty-body block scratch owner missing')
