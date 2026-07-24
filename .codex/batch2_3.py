from pathlib import Path
import re

PCLASS = Path('Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Persistent/PersistentParallelClassificationP1P6.cs')
P1 = Path('Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs')
GUARD = Path('Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Prediction/ContactEnvelopeGuard.cs')


def method(text, sig, next_sig, replacement):
    pat = re.compile(r'    ' + re.escape(sig) + r'.*?(?=\n    ' + re.escape(next_sig) + r')', re.S)
    out, n = pat.subn(replacement.rstrip(), text, count=1)
    if n != 1:
        raise RuntimeError(f'method {sig}: {n}')
    return out

pc = PCLASS.read_text()
pc = method(pc, 'private bool PreparePersistentPairSourceP1P6(', 'private void CommitPersistentClassificationP1P6(', '''    private bool PreparePersistentPairSourceP1P6(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        PrepareCurrentBodyLookup();
        bool cacheCanBePatched = IsPersistentCacheStructurallyReusableP1P6();
        SummarizePreparedIncrementalDirtyBodiesP1P6(
            ref incrementalStatistics,
            out int topologyDirtyCount,
            out bool entitySetDirty);
        incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        float dirtyRatio = States.Length > 0
            ? (float)topologyDirtyCount / States.Length
            : 1f;
        bool useFullRebuild = !cacheCanBePatched || entitySetDirty ||
                              dirtyRatio > IncrementalDirtyBodyRatioThreshold;
        if (useFullRebuild)
        {
            ClearPersistentClassificationCache();
            BuildCurrentIncrementalSweptProxies();
            long buildStart = ProfilerUnsafeUtility.Timestamp;
            long localBefore = incrementalStatistics.LocalBroadPhaseNanoseconds;
            FullRebuildPersistentNeighborTopology(ref incrementalStatistics);
            RebuildPersistentSpatialMembershipP1P6(
                IncrementalCacheState.Value.TopologyEpoch);
            long elapsed = TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - buildStart);
            long localElapsed =
                incrementalStatistics.LocalBroadPhaseNanoseconds - localBefore;
            long exclusive = elapsed - localElapsed;
            incrementalStatistics.FallbackNanoseconds += exclusive > 0L ? exclusive : 0L;
            incrementalStatistics.FullRebuildCount++;
            incrementalStatistics.UsedFullRebuild = 1;
        }
        else
        {
            long repairStart = ProfilerUnsafeUtility.Timestamp;
            long localBefore = incrementalStatistics.LocalBroadPhaseNanoseconds;
            if (topologyDirtyCount > 0)
            {
                IncrementallyRepairPersistentNeighborTopology(
                    ref incrementalStatistics);
                incrementalStatistics.IncrementalRepairCount++;
            }
            else
            {
                AdvancePersistentCacheTimestepP1P6(ref incrementalStatistics);
            }
            long elapsed = TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - repairStart);
            long localElapsed =
                incrementalStatistics.LocalBroadPhaseNanoseconds - localBefore;
            long exclusive = elapsed - localElapsed;
            incrementalStatistics.PairDiffNanoseconds += exclusive > 0L ? exclusive : 0L;
            incrementalStatistics.UsedIncrementalTopology = 1;
        }

        long classificationStart = ProfilerUnsafeUtility.Timestamp;
        if (TryReusePersistentContactViews(
                ref statistics,
                ref incrementalStatistics))
        {
            incrementalStatistics.SweptClassificationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - classificationStart);
            incrementalStatistics.PersistentNeighborPairCount =
                PersistentNeighborPairs.Length;
            incrementalStatistics.CurrentInteractionPairCount =
                PersistentNeighborPairs.Length;
            ValidateSoftAvoidancePairViewAgainstQuadraticOracle(
                ref incrementalStatistics);
            CommitFinalizedTimestepContactView(
                ref statistics,
                ref incrementalStatistics,
                false);
            return false;
        }

        long mappingStart = ProfilerUnsafeUtility.Timestamp;
        bool mapped = MapPersistentNeighborPairsToCurrentBodies();
        incrementalStatistics.PersistentPairMappingNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - mappingStart);
        if (mapped)
            return true;

        IncrementalContactCacheState invalidState = IncrementalCacheState.Value;
        invalidState.IsValid = 0;
        IncrementalCacheState.Value = invalidState;
        TimestepInteractionPairs.Clear();
        long fullSweepStart = ProfilerUnsafeUtility.Timestamp;
        BuildSweptInteractionPairs(ref statistics);
        incrementalStatistics.FullSweepSourceNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - fullSweepStart);
        incrementalStatistics.UsedFullRebuild = 1;
        incrementalStatistics.CurrentInteractionPairCount =
            TimestepInteractionPairs.Length;
        BuildSoftAvoidancePairViewFromInteractions(ref incrementalStatistics);
        FinalizeTimestepContactView(
            ref statistics,
            ref incrementalStatistics,
            false,
            0);
        return false;
    }
''')
PCLASS.write_text(pc)

p = P1.read_text()
start = p.index('    private void PrepareP1P6SubstepRepairClassification(')
end = p.index('\n    private void CommitP1P6SubstepRepairClassification(', start)
old = p[start:end]
marker = '        PreviousTimestepContactPairs.Clear();'
if marker not in old:
    raise RuntimeError('missing repair tail')
tail = old[old.index(marker):]
head = '''    private void PrepareP1P6SubstepRepairClassification(
        int substepIndex,
        NativeReference<ParallelJacobiRuntimeState> runtimeState)
    {
        ParallelPersistentClassificationState phase = default;
        PersistentClassificationResults.Clear();
        PersistentClassificationState.Value = phase;

        if (runtimeState.Value.IsValid == 0)
            return;
        if (!EnableTimestepContactSetCache ||
            !EnablePersistentContactCache ||
            IncrementalDirtyBodies.Length == 0)
        {
            RepairP1P6SubstepContactView(substepIndex, runtimeState);
            return;
        }

        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental = IncrementalStatistics.Value;
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        PrepareCurrentBodyLookup();
        if (!RefreshPreparedIncrementalDirtyBodiesP1P6(
                ref incremental,
                out int topologyDirtyCount))
        {
            incremental.ProxyValidationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - validationStart);
            BuildTimestepContactSet(
                ref statistics,
                ref incremental,
                true,
                true,
                substepIndex);
            InvalidateSoftIncidentIndexP1P6();
            RebuildPersistentIncidentPairLookupIfNeededP1P6();
            Statistics.Value = statistics;
            IncrementalStatistics.Value = incremental;
            return;
        }
        incremental.ProxyValidationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        float dirtyRatio = States.Length > 0
            ? (float)IncrementalDirtyBodies.Length / States.Length
            : 1f;
        if (dirtyRatio > IncrementalDirtyBodyRatioThreshold ||
            IncrementalCacheState.Value.IsValid == 0)
        {
            BuildTimestepContactSet(
                ref statistics,
                ref incremental,
                true,
                true,
                substepIndex);
            InvalidateSoftIncidentIndexP1P6();
            RebuildPersistentIncidentPairLookupIfNeededP1P6();
            Statistics.Value = statistics;
            IncrementalStatistics.Value = incremental;
            return;
        }

        long pairDiffStart = ProfilerUnsafeUtility.Timestamp;
        long localBroadPhaseBefore = incremental.LocalBroadPhaseNanoseconds;
        if (topologyDirtyCount > 0)
            IncrementallyRepairPersistentNeighborTopology(ref incremental, false);
        long pairDiffElapsed = TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - pairDiffStart);
        long localBroadPhaseElapsed =
            incremental.LocalBroadPhaseNanoseconds - localBroadPhaseBefore;
        long pairDiffExclusive = pairDiffElapsed - localBroadPhaseElapsed;
        incremental.PairDiffNanoseconds += pairDiffExclusive > 0L
            ? pairDiffExclusive
            : 0L;

'''
p = p[:start] + head + tail + p[end:]
P1.write_text(p)

g = GUARD.read_text()
g = g.replace(
    'IncrementalBodyDirtyFlags.Topology |\n                 IncrementalBodyDirtyFlags.CorrectedEscape',
    'IncrementalBodyDirtyFlags.Motion |\n                 IncrementalBodyDirtyFlags.CorrectedEscape')
g = g.replace(
    'IncrementalBodyDirtyFlags.Topology | source',
    'IncrementalBodyDirtyFlags.Motion | source')
GUARD.write_text(g)

all_text = '\n'.join(x.read_text() for x in (PCLASS, P1, GUARD))
if 'BuildCurrentIncrementalSweptProxies();\n\n        int topologyDirtyCount' in all_text:
    raise RuntimeError('substep full proxy build remains')
print('batch2_3 ok')
