from pathlib import Path
import re

from validate_contact_type_ownership import main as validate_type_ownership


ROOT = Path('Entities/Unit/Systems/FlowField')
PIPELINE = ROOT / 'Jobs/ContactPipeline'


def read(path: Path) -> str:
    return path.read_text(encoding='utf-8')


def require_tokens(text: str, tokens: list[str], label: str) -> None:
    missing = [token for token in tokens if token not in text]
    if missing:
        raise SystemExit(f'{label} missing: {missing!r}')


def validate_legacy_removal() -> None:
    source = '\n'.join(read(path) for path in ROOT.rglob('*.cs'))
    forbidden = [
        'BuildAdaptiveFatAabbHotspots',
        'BuildAdaptiveHybridContactPairs',
        'BuildContactPairsFromFatAabbCache',
        'EnsureFatAabbRawCandidates',
        'UpdateAdaptiveFatAabbHistoryAfterSolve',
        'AreCorrectedDiscsInsideFatCache',
        'ShadowStatistics',
        'LegacyBroadPhaseStatistics',
        'LegacyBroadPhaseSource',
        'MappedFatCachePairs',
        'FlowMovementFrameState',
        'CalculateIndependentFlowForceJob',
        'UnitCollisionPair',
        'UnitCollisionPairComparer',
        'UnitContactMode',
        'SolveXpbdUnitContactsJob',
        'ContactFrameResources',
        'ContactPersistentState',
        'BaseFlowMovementComposition',
    ]
    returned = [
        symbol for symbol in forbidden
        if re.search(rf'\b{re.escape(symbol)}\b', source)
    ]
    if returned:
        raise SystemExit('Retired contact symbols returned: ' + ', '.join(returned))

    forbidden_paths = [
        ROOT / 'Jobs/Legacy',
        ROOT / 'Jobs/Compatibility',
        ROOT / 'Jobs/Compatibility.meta',
        ROOT / 'BaseFlowMovementComposition.cs',
        ROOT / 'BaseFlowMovementComposition.cs.meta',
        ROOT / 'ContactPipelineResources.cs',
        ROOT / 'ContactPipelineResources.cs.meta',
        ROOT / 'Jobs/FlowMovementFrameState.cs',
        ROOT / 'Jobs/FlowMovementFrameState.cs.meta',
        ROOT / 'Jobs/CalculateIndependentFlowForceJob.cs',
        PIPELINE / 'Core/SolveXpbdUnitContactsJob.cs',
        PIPELINE / 'Core/SolveXpbdUnitContactsJob.cs.meta',
        PIPELINE / 'Core/ContactPairTypes.cs',
        PIPELINE / 'Core/ContactPairTypes.cs.meta',
        PIPELINE / 'Core/CrowdEnvironmentAccess.cs',
        PIPELINE / 'Core/CrowdEnvironmentAccess.cs.meta',
    ]
    present = [str(path) for path in forbidden_paths if path.exists()]
    if present:
        raise SystemExit('Retired contact files returned: ' + repr(present))


def validate_configuration_scope() -> None:
    production = '\n'.join(read(path) for path in PIPELINE.rglob('*.cs'))
    occurrences = [
        path for path in PIPELINE.rglob('*.cs')
        if 'PersistentGuardEnvelopeMargin' in read(path)
    ]
    expected = PIPELINE / 'Core/ContactPipelineConfiguration.cs'
    if occurrences != [expected]:
        raise SystemExit(
            'Serialized FatAabb margin escaped configuration boundary: '
            + repr(occurrences)
        )
    if re.search(
        r'\b(?:SleepingBody|SleepingIsland|ContactIslandSleeping)\b',
        production,
    ):
        raise SystemExit(
            'Sleeping policy introduced without a dedicated design stage'
        )


def validate_diagnostics_layout() -> None:
    expected = [
        ROOT / 'Diagnostics/Capture/ContactPipelineTelemetry.cs',
        ROOT / 'Diagnostics/Capture/IncrementalContactPipelineDiagnostics.cs',
        ROOT / 'Diagnostics/Capture/SimulationDebuggerSnapshotPublishing.cs',
        ROOT / 'Diagnostics/Capture/Jobs/PublishPredictiveDiscContactStatisticsJob.cs',
        ROOT / 'Diagnostics/Capture/Jobs/Stage3ContactDiagnosticRecorder.cs',
        ROOT / 'Diagnostics/Instrumentation/ContactPipelineProfilerClock.cs',
        ROOT / 'Diagnostics/Validation/IncrementalContactOracle.cs',
        ROOT / 'Diagnostics/README.md',
    ]
    missing = [str(path) for path in expected if not path.exists()]
    if missing:
        raise SystemExit('Diagnostics ownership files missing: ' + repr(missing))

    legacy = [
        ROOT / 'Diagnostics/ContactPipelineTelemetry.cs',
        ROOT / 'Diagnostics/IncrementalContactPipelineDiagnostics.cs',
        ROOT / 'Diagnostics/SimulationDebuggerSnapshotPublishing.cs',
        ROOT / 'Jobs/PublishPredictiveDiscContactStatisticsJob.cs',
        ROOT / 'Jobs/Stage3ContactDiagnosticRecorder.cs',
        ROOT / 'Jobs/IncrementalContactOracle.cs',
        PIPELINE / 'Core/ContactPipelineProfilerClock.cs',
    ]
    returned = [str(path) for path in legacy if path.exists()]
    if returned:
        raise SystemExit(
            'Diagnostics files returned to runtime folders: ' + repr(returned)
        )

    csv_text = read(
        ROOT / 'Diagnostics/Recording/IncrementalContactPipelineCsvRecorder.cs'
    )
    if 'CsvSchemaVersion = 7' not in csv_text:
        raise SystemExit('Unexpected contact CSV schema')
    if (
        'LegacyCacheUseCount' in csv_text
        or 'LegacyBroadPhaseStatistics' in csv_text
    ):
        raise SystemExit('Legacy CSV columns returned')

    editor_settings = ROOT / 'Editor/SimulationDiagnosticsBuildSettings.cs'
    compile_status = ROOT / 'Diagnostics/SimulationDiagnosticsCompileStatus.cs'
    asset = ROOT / 'Editor/SimulationDiagnosticsBuildSettings.asset'
    for path in (editor_settings, compile_status, asset):
        if not path.exists():
            raise SystemExit(f'Missing diagnostics build artifact: {path}')
    editor_text = read(editor_settings)
    require_tokens(
        editor_text,
        [
            'RTS_CONTACT_DIAGNOSTICS',
            'PlayerSettings.GetScriptingDefineSymbols',
            'PlayerSettings.SetScriptingDefineSymbols',
            'Editor Diagnostics',
            'Development Diagnostics',
            'Release Gameplay Only',
        ],
        'Diagnostics build settings contract',
    )
    if 'UnityEditor' in read(compile_status):
        raise SystemExit('Runtime compile status must not reference UnityEditor')


def validate_parallel_jacobi() -> None:
    solver = read(PIPELINE / 'Solver/ParallelJacobiSolver.cs')
    p1p6 = read(PIPELINE / 'Solver/ParallelContactPipelineP1P6.cs')
    resources = read(ROOT / 'ConstraintSolverFrameResources.cs')
    stage_jobs = read(PIPELINE / 'Core/ContactPipelineStageJobs.cs')
    base = read(ROOT / 'BaseFlowMovementSystem.cs')

    require_tokens(
        solver,
        [
            'IJobParallelForDefer',
            'EvaluateParallelJacobiPairsJob',
            'GatherAndApplyParallelJacobiBodiesJob',
        ],
        'Parallel Jacobi solver contract',
    )
    require_tokens(
        p1p6,
        [
            'ScheduleParallelJacobiP1P6',
            'EvaluateParallelJacobiPairsWithDiagnosticsJob',
            'ParallelSimulationDebuggerPairCandidates',
            'CountParallelSimulationDebuggerPairBlocksJob',
            'PrefixParallelSimulationDebuggerPairsJob',
            'ScatterParallelSimulationDebuggerPairsJob',
            'ConstraintSolverOperation.MergeParallelDebuggerPairs',
        ],
        'P1-P6 parallel contract',
    )
    require_tokens(
        resources,
        [
            'ActiveIncidentOffsets',
            'ActiveIncidentPairIndices',
            'JacobiPairCorrections',
        ],
        'Constraint solver resource ownership',
    )
    require_tokens(
        stage_jobs,
        [
            'MergeParallelDebuggerPairs',
            'ConstraintSolverOperation',
            'MergeParallelSimulationDebuggerPairScratch',
        ],
        'Constraint solver stage ownership',
    )

    if 'Interlocked' in solver or 'Atomic' in solver:
        raise SystemExit('Parallel Jacobi must not use floating-point atomics')
    if 'requiresSerialJacobiCapture' in base:
        raise SystemExit(
            'Selected-pair capture must not change the Jacobi backend'
        )
    require_tokens(
        base,
        [
            'bool useParallelJacobi = usesJacobiScratch;',
            'captureParallelSelectedPairs',
        ],
        'Base movement backend selection',
    )


def validate_source_structure() -> None:
    roots = [
        PIPELINE,
        ROOT / 'Diagnostics/Capture',
        ROOT / 'Diagnostics/Instrumentation',
        ROOT / 'Diagnostics/Validation',
    ]
    for root in roots:
        for path in root.rglob('*.cs'):
            text = read(path)
            if text.count('{') != text.count('}'):
                raise SystemExit(f'Brace mismatch: {path}')

    temporary = [
        Path('.github/workflows/diagnose-final-ownership.yml'),
        Path('.github/workflows/update-parallel-jacobi-contract.yml'),
    ]
    present = [str(path) for path in temporary if path.exists()]
    if present:
        raise SystemExit('Temporary cutover workflows remain: ' + repr(present))


def main() -> None:
    validate_legacy_removal()
    validate_configuration_scope()
    validate_type_ownership()
    validate_diagnostics_layout()
    validate_parallel_jacobi()
    validate_source_structure()
    print('Contact pipeline static contracts passed.')


if __name__ == '__main__':
    main()
