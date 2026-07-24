from pathlib import Path
import re

ROOT = Path('.')
INC = ROOT / 'Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Persistent/IncrementalPredictiveContactPipeline.cs'
PERSIST = ROOT / 'Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Persistent/PersistentParallelClassificationP1P6.cs'
PARALLEL = ROOT / 'Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Solver/ParallelContactPipelineP1P6.cs'


def replace_method(text: str, start_signature: str, next_signature: str, replacement: str) -> str:
    pattern = re.compile(
        r'    ' + re.escape(start_signature) + r'.*?(?=\n    ' + re.escape(next_signature) + r')',
        re.S,
    )
    updated, count = pattern.subn(replacement.rstrip(), text, count=1)
    if count != 1:
        raise RuntimeError(f'Expected one replacement for {start_signature}, got {count}')
    return updated


inc = INC.read_text(encoding='utf-8')

inc = replace_method(
    inc,
    'private bool BuildContactPairsFromPersistentNeighborSet(',
    'private bool TryReusePersistentContactViews(',
    '''    private bool BuildContactPairsFromPersistentNeighborSet(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        bool forceFullRebuild,
        int scheduleStartSubstep,
        out bool persistentViewReady)
    {
        persistentViewReady = false;
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        PrepareCurrentBodyLookup();
        ClearIncrementalDirtyBodySet();
        bool cacheCanBePatched = !forceFullRebuild &&
                                 IsPersistentCacheStructurallyReusableP1P6();
        incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        if (!cacheCanBePatched)
        {
            ClearPersistentClassificationCache();
            BuildCurrentIncrementalSweptProxies();
            long buildStart = ProfilerUnsafeUtility.Timestamp;
            long localBroadPhaseBefore = incrementalStatistics.LocalBroadPhaseNanoseconds;
            FullRebuildPersistentNeighborTopology(ref incrementalStatistics);
            RebuildPersistentSpatialMembershipP1P6(
                IncrementalCacheState.Value.TopologyEpoch);
            long buildElapsed = TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - buildStart);
            long localBroadPhaseElapsed =
                incrementalStatistics.LocalBroadPhaseNanoseconds - localBroadPhaseBefore;
            long fallbackExclusive = buildElapsed - localBroadPhaseElapsed;
            incrementalStatistics.FallbackNanoseconds +=
                fallbackExclusive > 0L ? fallbackExclusive : 0L;
            incrementalStatistics.FullRebuildCount++;
            incrementalStatistics.UsedFullRebuild = 1;
        }
        else
        {
            IncrementalContactCacheState state = IncrementalCacheState.Value;
            state.Timestep++;
            state.LastUpdateWasFullRebuild = 0;
            state.BodyCount = States.Length;
            state.NeighborPairCount = PersistentNeighborPairs.Length;
            IncrementalCacheState.Value = state;
            incrementalStatistics.Timestep = state.Timestep;
            incrementalStatistics.NeighborPairRetainedCount =
                PersistentNeighborPairs.Length;
            incrementalStatistics.UsedIncrementalTopology = 1;
        }

        long classificationStart = ProfilerUnsafeUtility.Timestamp;
        if (TryReusePersistentContactViews(
                ref statistics,
                ref incrementalStatistics))
        {
            incrementalStatistics.SweptClassificationNanoseconds +=
                TimestampToNanoseconds(
                    ProfilerUnsafeUtility.Timestamp - classificationStart);
            incrementalStatistics.PersistentNeighborPairCount =
                PersistentNeighborPairs.Length;
            persistentViewReady = true;
            return true;
        }

        long mappingStart = ProfilerUnsafeUtility.Timestamp;
        bool mapped = MapPersistentNeighborPairsToCurrentBodies();
        incrementalStatistics.PersistentPairMappingNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - mappingStart);
        if (!mapped)
        {
            IncrementalContactCacheState invalidState = IncrementalCacheState.Value;
            invalidState.IsValid = 0;
            IncrementalCacheState.Value = invalidState;
            TimestepInteractionPairs.Clear();
            long fullSweepStart = ProfilerUnsafeUtility.Timestamp;
            BuildSweptInteractionPairs(ref statistics);
            incrementalStatistics.FullSweepSourceNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - fullSweepStart);
            incrementalStatistics.UsedFullRebuild = 1;
            return false;
        }

        classificationStart = ProfilerUnsafeUtility.Timestamp;
        ClassifyOrReusePersistentNeighborPairs(
            ref statistics,
            ref incrementalStatistics,
            scheduleStartSubstep);
        incrementalStatistics.SweptClassificationNanoseconds +=
            TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - classificationStart);
        incrementalStatistics.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
        persistentViewReady = true;
        return true;
    }
''')

# Replace the full-array validator and full-array metadata update with O(1)
# structural gating plus dirty-body-only proxy refresh/update helpers.
inc = replace_method(
    inc,
    'private bool ValidateAndClassifyIncrementalDirtyBodies(',
    'private void ClearIncrementalDirtyBodySet()',
    '''    private bool IsPersistentCacheStructurallyReusableP1P6()
    {
        IncrementalContactCacheState state = IncrementalCacheState.Value;
        return state.IsValid != 0 &&
               state.BodyCount == States.Length &&
               PersistentSweptProxies.Length == States.Length &&
               CurrentIncrementalProxies.Length == PersistentSweptProxies.Length &&
               state.GuardMargin == math.max(0f, GuardEnvelopeMargin) &&
               state.PredictiveSkin == math.max(0f, PredictiveSkin) &&
               state.TimestepContactMargin == math.max(0f, TimestepContactMargin) &&
               state.SoftAvoidanceShell == math.max(0f, SoftAvoidanceShell) &&
               state.SoftAvoidanceResponseRate ==
                   math.max(0f, SoftAvoidanceResponseRate) &&
               state.RvoTimeHorizon == math.max(0f, RvoTimeHorizon) &&
               state.SubstepCount == math.max(1, SubstepCount) &&
               state.PredictivePairGenerationEnabled ==
                   (byte)(EnablePredictivePairGeneration ? 1 : 0) &&
               state.PredictiveContactsEnabled ==
                   (byte)(EnablePredictiveContacts ? 1 : 0) &&
               state.SoftAvoidanceVelocitySolver ==
                   (byte)SoftAvoidanceVelocitySolver;
    }

    private PersistentSweptProxy BuildCurrentIncrementalSweptProxyP1P6(
        int bodyIndex)
    {
        FlowMovementFrameState state = States[bodyIndex];
        PersistentSweptProxy proxy = new PersistentSweptProxy
        {
            Entity = state.Entity,
            BodyIndex = bodyIndex,
            IsValid = (byte)(state.IsInsideGrid ? 1 : 0)
        };
        if (!state.IsInsideGrid)
            return proxy;

        float guardMargin = math.max(0f, GuardEnvelopeMargin);
        CalculateIncrementalTightSweptBounds(
            state,
            out proxy.TightMin,
            out proxy.TightMax);
        proxy.GuardMin = proxy.TightMin - guardMargin;
        proxy.GuardMax = proxy.TightMax + guardMargin;
        proxy.TrajectoryStart = state.TimestepStartPosition.xz;
        proxy.TrajectoryEnd = state.TimestepPredictedPosition.xz;
        proxy.AvoidanceHorizonEnd = CalculateAvoidanceHorizonEnd(state);
        proxy.Radius = math.max(0f, state.Radius);
        proxy.MotionVersion = 1u;
        return proxy;
    }

    private bool RefreshIncrementalDirtyProxySetP1P6(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        out int topologyDirtyCount)
    {
        topologyDirtyCount = 0;
        if (!IsPersistentCacheStructurallyReusableP1P6())
            return false;

        for (int dirtyIndex = 0;
             dirtyIndex < IncrementalDirtyBodies.Length;
             dirtyIndex++)
        {
            IncrementalDirtyBody dirty = IncrementalDirtyBodies[dirtyIndex];
            int bodyIndex = dirty.BodyIndex;
            if ((uint)bodyIndex >= (uint)States.Length)
                return false;

            PersistentSweptProxy current =
                BuildCurrentIncrementalSweptProxyP1P6(bodyIndex);
            if (!TryFindCurrentIncrementalProxyP1P6(
                    current.Entity,
                    out _,
                    out int currentProxyIndex) ||
                !TryFindPersistentProxy(
                    current.Entity,
                    out PersistentSweptProxy previous) ||
                previous.IsValid != current.IsValid)
                return false;

            AssignMotionVersion(ref current, previous);
            IncrementalBodyDirtyFlags flags = dirty.Flags |
                                               IncrementalBodyDirtyFlags.Motion;
            if (current.IsValid != 0 && !AabbContains(
                    previous.GuardMin,
                    previous.GuardMax,
                    current.TightMin,
                    current.TightMax))
            {
                flags |= IncrementalBodyDirtyFlags.Topology;
            }

            if ((flags & IncrementalBodyDirtyFlags.Topology) != 0)
            {
                topologyDirtyCount++;
                incrementalStatistics.TopologyDirtyBodyCount++;
            }
            else
            {
                incrementalStatistics.MotionDirtyBodyCount++;
            }

            current.BodyIndex = bodyIndex;
            CurrentIncrementalProxies[currentProxyIndex] = current;
            dirty.Flags = flags;
            IncrementalDirtyBodies[dirtyIndex] = dirty;
            IncrementalDirtyFlagsByBody[bodyIndex] = (byte)flags;
        }
        return true;
    }

    private void UpdatePersistentProxyMetadataForDirtyBodiesP1P6()
    {
        for (int dirtyIndex = 0;
             dirtyIndex < IncrementalDirtyBodies.Length;
             dirtyIndex++)
        {
            IncrementalDirtyBody dirty = IncrementalDirtyBodies[dirtyIndex];
            int bodyIndex = dirty.BodyIndex;
            if ((uint)bodyIndex >= (uint)States.Length)
                continue;

            Entity entity = States[bodyIndex].Entity;
            if (!TryFindCurrentIncrementalProxyP1P6(
                    entity,
                    out PersistentSweptProxy current,
                    out _) )
                continue;
            int persistentProxyIndex = FindPersistentProxyIndex(entity);
            if (persistentProxyIndex < 0)
                continue;

            PersistentSweptProxy previous =
                PersistentSweptProxies[persistentProxyIndex];
            IncrementalBodyDirtyFlags flags = GetDirtyFlags(bodyIndex);
            if ((flags & IncrementalBodyDirtyFlags.Topology) != 0)
            {
                PersistentSweptProxies[persistentProxyIndex] = current;
                continue;
            }

            // Trusted clean space keeps its old guard. Only the local hotspot's
            // trajectory metadata and motion version are refreshed.
            previous.BodyIndex = current.BodyIndex;
            previous.TightMin = current.TightMin;
            previous.TightMax = current.TightMax;
            previous.TrajectoryStart = current.TrajectoryStart;
            previous.TrajectoryEnd = current.TrajectoryEnd;
            previous.AvoidanceHorizonEnd = current.AvoidanceHorizonEnd;
            previous.Radius = current.Radius;
            previous.MotionVersion = current.MotionVersion;
            previous.IsValid = current.IsValid;
            PersistentSweptProxies[persistentProxyIndex] = previous;
        }
    }
''')

old_clear = '''    private void ClearIncrementalDirtyBodySet()
    {
        IncrementalDirtyBodies.Clear();
        for (int bodyIndex = 0;
             bodyIndex < IncrementalDirtyFlagsByBody.Length;
             bodyIndex++)
            IncrementalDirtyFlagsByBody[bodyIndex] = 0;
    }'''
new_clear = '''    private void ClearIncrementalDirtyBodySet()
    {
        for (int dirtyIndex = 0;
             dirtyIndex < IncrementalDirtyBodies.Length;
             dirtyIndex++)
        {
            int bodyIndex = IncrementalDirtyBodies[dirtyIndex].BodyIndex;
            if ((uint)bodyIndex < (uint)IncrementalDirtyFlagsByBody.Length)
                IncrementalDirtyFlagsByBody[bodyIndex] = 0;
        }
        IncrementalDirtyBodies.Clear();
    }'''
if old_clear not in inc:
    raise RuntimeError('ClearIncrementalDirtyBodySet anchor not found')
inc = inc.replace(old_clear, new_clear, 1)

old_build = '''    private void BuildCurrentIncrementalSweptProxies()
    {
        CurrentIncrementalProxies.Clear();
        float guardMargin = math.max(0f, GuardEnvelopeMargin);

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            PersistentSweptProxy proxy = new PersistentSweptProxy
            {
                Entity = state.Entity,
                BodyIndex = bodyIndex,
                IsValid = (byte)(state.IsInsideGrid ? 1 : 0)
            };

            if (state.IsInsideGrid)
            {
                CalculateIncrementalTightSweptBounds(
                    state,
                    out proxy.TightMin,
                    out proxy.TightMax);
                proxy.GuardMin = proxy.TightMin - guardMargin;
                proxy.GuardMax = proxy.TightMax + guardMargin;
                proxy.TrajectoryStart = state.TimestepStartPosition.xz;
                proxy.TrajectoryEnd = state.TimestepPredictedPosition.xz;
                proxy.AvoidanceHorizonEnd = CalculateAvoidanceHorizonEnd(state);
                proxy.Radius = math.max(0f, state.Radius);
                proxy.MotionVersion = 1u;
            }

            CurrentIncrementalProxies.Add(proxy);
        }

        CurrentIncrementalProxies.AsArray().Sort(new PersistentSweptProxyComparer());
    }'''
new_build = '''    private void BuildCurrentIncrementalSweptProxies()
    {
        CurrentIncrementalProxies.Clear();
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            CurrentIncrementalProxies.Add(
                BuildCurrentIncrementalSweptProxyP1P6(bodyIndex));
        }
        CurrentIncrementalProxies.AsArray().Sort(new PersistentSweptProxyComparer());
    }'''
if old_build not in inc:
    raise RuntimeError('BuildCurrentIncrementalSweptProxies anchor not found')
inc = inc.replace(old_build, new_build, 1)

inc = replace_method(
    inc,
    'private bool TryIncrementallyRepairEscapedContactSet(',
    'private bool MapDirtyIncidentNeighborPairsToCurrentBodies()',
    '''    private bool TryIncrementallyRepairEscapedContactSet(
        int substepIndex,
        int scheduleStartSubstep,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        PrepareCurrentBodyLookup();
        int escapedBodyCount = IncrementalDirtyBodies.Length;
        if (escapedBodyCount == 0)
        {
            incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - validationStart);
            return true;
        }

        if (!RefreshIncrementalDirtyProxySetP1P6(
                ref incrementalStatistics,
                out int topologyDirtyCount))
        {
            incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - validationStart);
            return false;
        }

        float dirtyRatio = States.Length > 0
            ? (float)escapedBodyCount / States.Length
            : 1f;
        if (dirtyRatio > IncrementalDirtyBodyRatioThreshold ||
            IncrementalCacheState.Value.IsValid == 0)
        {
            incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - validationStart);
            return false;
        }

        incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);
        long pairDiffStart = ProfilerUnsafeUtility.Timestamp;
        long localBroadPhaseBefore = incrementalStatistics.LocalBroadPhaseNanoseconds;
        UpdatePersistentProxyMetadataForDirtyBodiesP1P6();
        if (topologyDirtyCount > 0)
        {
            IncrementallyRepairPersistentNeighborTopology(
                ref incrementalStatistics,
                false);
        }
        long pairDiffElapsed = TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - pairDiffStart);
        long localBroadPhaseElapsed =
            incrementalStatistics.LocalBroadPhaseNanoseconds - localBroadPhaseBefore;
        long pairDiffExclusive = pairDiffElapsed - localBroadPhaseElapsed;
        incrementalStatistics.PairDiffNanoseconds +=
            pairDiffExclusive > 0L ? pairDiffExclusive : 0L;

        PreviousTimestepContactPairs.Clear();
        PreviousTimestepContactPairs.AddRange(TimestepContactPairs.AsArray());
        long mappingStart = ProfilerUnsafeUtility.Timestamp;
        if (!MapDirtyIncidentNeighborPairsToCurrentBodies())
        {
            incrementalStatistics.PersistentPairMappingNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - mappingStart);
            return false;
        }
        incrementalStatistics.PersistentPairMappingNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - mappingStart);

        long contactViewStart = ProfilerUnsafeUtility.Timestamp;
        long classificationStart = contactViewStart;
        ClassifyAndPatchDirtyIncidentContacts(
            ref statistics,
            ref incrementalStatistics,
            scheduleStartSubstep);
        incrementalStatistics.SweptClassificationNanoseconds +=
            TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - classificationStart);
        RebuildEscapedTimestepContactView(
            ref statistics,
            ref incrementalStatistics);
        statistics.TimestepContactSetBuildNanoseconds +=
            TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - contactViewStart);

        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            int bodyIndex = IncrementalDirtyBodies[dirtyIndex].BodyIndex;
            FlowMovementFrameState state = States[bodyIndex];
            state.TimestepEscaped = 0;
            States[bodyIndex] = state;
        }

        IncrementalContactCacheState cacheState = IncrementalCacheState.Value;
        cacheState.LastUpdateWasFullRebuild = 0;
        cacheState.NeighborPairCount = PersistentNeighborPairs.Length;
        IncrementalCacheState.Value = cacheState;
        incrementalStatistics.IncrementalRepairCount++;
        incrementalStatistics.UsedIncrementalTopology = 1;
        incrementalStatistics.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
        return true;
    }
''')

INC.write_text(inc, encoding='utf-8')

persist = PERSIST.read_text(encoding='utf-8')
persist = replace_method(
    persist,
    'private bool PreparePersistentPairSourceP1P6(',
    'private void CommitPersistentClassificationP1P6(',
    '''    private bool PreparePersistentPairSourceP1P6(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        PrepareCurrentBodyLookup();
        ClearIncrementalDirtyBodySet();
        bool cacheCanBePatched = IsPersistentCacheStructurallyReusableP1P6();
        incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        if (!cacheCanBePatched)
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
            IncrementalContactCacheState state = IncrementalCacheState.Value;
            state.Timestep++;
            state.LastUpdateWasFullRebuild = 0;
            state.BodyCount = States.Length;
            state.NeighborPairCount = PersistentNeighborPairs.Length;
            IncrementalCacheState.Value = state;
            incrementalStatistics.Timestep = state.Timestep;
            incrementalStatistics.NeighborPairRetainedCount =
                PersistentNeighborPairs.Length;
            incrementalStatistics.UsedIncrementalTopology = 1;
        }

        long classificationStart = ProfilerUnsafeUtility.Timestamp;
        if (TryReusePersistentContactViews(
                ref statistics,
                ref incrementalStatistics))
        {
            incrementalStatistics.SweptClassificationNanoseconds +=
                TimestampToNanoseconds(
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
PERSIST.write_text(persist, encoding='utf-8')

parallel = PARALLEL.read_text(encoding='utf-8')
parallel = replace_method(
    parallel,
    'private void PrepareP1P6SubstepRepairClassification(',
    'private void CommitP1P6SubstepRepairClassification(',
    '''    private void PrepareP1P6SubstepRepairClassification(
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
        if (!RefreshIncrementalDirtyProxySetP1P6(
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
        UpdatePersistentProxyMetadataForDirtyBodiesP1P6();
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

        PreviousTimestepContactPairs.Clear();
        PreviousTimestepContactPairs.AddRange(TimestepContactPairs.AsArray());
        long mappingStart = ProfilerUnsafeUtility.Timestamp;
        if (!MapDirtyIncidentNeighborPairsToCurrentBodies())
        {
            incremental.PersistentPairMappingNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - mappingStart);
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
        incremental.PersistentPairMappingNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - mappingStart);

        RemoveDirtyPredictiveContactSchedules();
        PredictiveContactScratch.Clear();
        for (int contactIndex = 0;
             contactIndex < PersistentPredictiveContacts.Length;
             contactIndex++)
        {
            PersistentPredictiveContact contact =
                PersistentPredictiveContacts[contactIndex];
            if (IsDirtyEntity(contact.Key.EntityA) ||
                IsDirtyEntity(contact.Key.EntityB))
                continue;
            PredictiveContactScratch.Add(contact);
        }

        phase.BuildStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        phase.ClassificationStartTimestamp = phase.BuildStartTimestamp;
        phase.Timestep = IncrementalCacheState.Value.Timestep;
        phase.ClassificationEpoch = CalculateClassificationEpoch();
        phase.NeedsCommit = 2;
        PersistentClassificationResults.ResizeUninitialized(Pairs.Length);
        PersistentClassificationState.Value = phase;
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
    }
''')
PARALLEL.write_text(parallel, encoding='utf-8')

# Hard assertions: the global validator and full metadata sweep must be gone
# from production source, while dirty-only repair helpers must exist.
all_text = inc + persist + parallel
if 'ValidateAndClassifyIncrementalDirtyBodies' in all_text:
    raise RuntimeError('Global proxy validator still referenced')
if 'UpdatePersistentProxyMetadata();' in all_text:
    raise RuntimeError('Full proxy metadata sweep still referenced')
for token in (
    'IsPersistentCacheStructurallyReusableP1P6',
    'RefreshIncrementalDirtyProxySetP1P6',
    'UpdatePersistentProxyMetadataForDirtyBodiesP1P6',
):
    if token not in all_text:
        raise RuntimeError(f'Missing required token: {token}')

print('Applied local-hotspot persistent proxy patch')
