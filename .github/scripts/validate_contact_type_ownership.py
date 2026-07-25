from pathlib import Path


def main() -> None:
    flow = Path('Entities/Unit/Systems/FlowField')
    pipeline = flow / 'Jobs/ContactPipeline'
    ownership = {
        'public struct BodyPair': pipeline / 'Core/BodyPair.cs',
        'public struct ContactConstraint': pipeline / 'Core/ContactConstraint.cs',
        'public struct SweptDiscCellEntry': pipeline / 'BroadPhase/SweptDiscTypes.cs',
        'public struct IncrementalContactCacheState': (
            pipeline / 'Persistent/IncrementalPredictiveContactTypes.cs'
        ),
        'public struct IncrementalContactPipelineStatistics': (
            flow / 'Diagnostics/Capture/ContactPipelineTelemetry.cs'
        ),
    }

    for symbol, expected_path in ownership.items():
        paths = [
            path
            for path in flow.rglob('*.cs')
            if symbol in path.read_text(encoding='utf-8')
        ]
        if paths != [expected_path]:
            raise SystemExit(f'{symbol} ownership mismatch: {paths}')

    runtime_types = (
        pipeline / 'Persistent/IncrementalPredictiveContactTypes.cs'
    ).read_text(encoding='utf-8')
    telemetry = (
        flow / 'Diagnostics/Capture/ContactPipelineTelemetry.cs'
    ).read_text(encoding='utf-8')

    if 'Nanoseconds' in runtime_types or 'OraclePairCount' in runtime_types:
        raise SystemExit('Telemetry leaked into authoritative runtime state')
    if 'IncrementalContactCacheState' in telemetry:
        raise SystemExit('Runtime cache state leaked into telemetry schema')

    temporary_workflows = list(
        Path('.github/workflows').glob('refactor-contact-pipeline-phase*.yml')
    )
    temporary_workflows += list(Path('.github/workflows').glob('audit-*.yml'))
    if temporary_workflows:
        raise SystemExit(
            'Temporary workflows remain: ' + repr(temporary_workflows)
        )

    print('Contact type ownership and temporary-artifact contracts passed.')


if __name__ == '__main__':
    main()
