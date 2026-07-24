from pathlib import Path

P1 = Path('Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs')
GUARD = Path('Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Prediction/ContactEnvelopeGuard.cs')


def replace_once(text: str, old: str, new: str, name: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f'{name}: expected one match, got {count}')
    return text.replace(old, new, 1)

p1 = P1.read_text(encoding='utf-8')
p1 = replace_once(
    p1,
    '        [ReadOnly] public NativeArray<PersistentSweptProxy> PersistentProxies;\n'
    '        [ReadOnly] public NativeArray<int> PersistentProxyIndexByBody;\n',
    '        [NativeDisableParallelForRestriction]\n'
    '        public NativeArray<PersistentSweptProxy> PersistentProxies;\n'
    '        [ReadOnly] public NativeArray<int> PersistentProxyIndexByBody;\n',
    'persistent proxy write access')
p1 = replace_once(
    p1,
    '            PersistentSweptProxies.Clear();\n'
    '            PersistentNeighborPairs.Clear();\n',
    '            PersistentSweptProxies.Clear();\n'
    '            PersistentProxyIndexByBody.Clear();\n'
    '            PersistentNeighborPairs.Clear();\n',
    'persistent mapping clear')
P1.write_text(p1, encoding='utf-8')

guard = GUARD.read_text(encoding='utf-8')
guard = replace_once(
    guard,
    '                IncrementalBodyDirtyFlags.Topology |\n'
    '                IncrementalBodyDirtyFlags.CorrectedEscape);',
    '                IncrementalBodyDirtyFlags.Motion |\n'
    '                IncrementalBodyDirtyFlags.CorrectedEscape);',
    'wall correction dirty source')
GUARD.write_text(guard, encoding='utf-8')

assert '[ReadOnly] public NativeArray<PersistentSweptProxy> PersistentProxies;' not in p1
assert 'PersistentProxyIndexByBody.Clear();' in p1
assert 'IncrementalBodyDirtyFlags.Topology |\n                IncrementalBodyDirtyFlags.CorrectedEscape' not in guard
print('Applied simulation integration batch 2 follow-up')
