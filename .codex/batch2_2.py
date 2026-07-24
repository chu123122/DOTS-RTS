from pathlib import Path
import re

P = Path('Entities/Unit/Systems/FlowField/Jobs/ContactPipeline/Persistent/IncrementalPredictiveContactPipeline.cs')


def rep(text, old, new, name):
    if old not in text:
        raise RuntimeError('missing ' + name)
    return text.replace(old, new, 1)


def method(text, sig, next_sig, replacement):
    pat = re.compile(r'    ' + re.escape(sig) + r'.*?(?=\n    ' + re.escape(next_sig) + r')', re.S)
    out, n = pat.subn(replacement.rstrip(), text, count=1)
    if n != 1:
        raise RuntimeError(f'method {sig}: {n}')
    return out

text = P.read_text()
text = method(text, 'private bool BuildContactPairsFromPersistentNeighborSet(', 'private bool TryReusePersistentContactViews(', '''    private bool BuildContactPairsFromPersistentNeighborSet(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        bool forceFullRebuild,
        int scheduleStartSubstep,
        out bool persistentViewReady)
    {
        persistentViewReady = false;
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        PrepareCurrentBodyLookup();
        bool cacheCanBePatched = !forceFullRebuild && IsPersistentCacheStructurallyReusableP1P6();
        SummarizePreparedIncrementalDirtyBodiesP1P6(
            ref incrementalStatistics,
            out int topologyDirtyCount,
            out bool entitySetDirty);
        incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        float dirtyRatio = States.Length > 0 ? (float)topologyDirtyCount / States.Length : 1f;
        bool useFullRebuild = !cacheCanBePatched || entitySetDirty ||
                              dirtyRatio > IncrementalDirtyBodyRatioThreshold;
        if (useFullRebuild)
        {
            ClearPersistentClassificationCache();
            BuildCurrentIncrementalSweptProxies();
            long buildStart = ProfilerUnsafeUtility.Timestamp;
            long localBefore = incrementalStatistics.LocalBroadPhaseNanoseconds;
            FullRebuildPersistentNeighborTopology(ref incrementalStatistics);
            RebuildPersistentSpatialMembershipP1P6(IncrementalCacheState.Value.TopologyEpoch);
            long elapsed = TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - buildStart);
            long localElapsed = incrementalStatistics.LocalBroadPhaseNanoseconds - localBefore;
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
                IncrementallyRepairPersistentNeighborTopology(ref incrementalStatistics);
                incrementalStatistics.IncrementalRepairCount++;
            }
            else
            {
                AdvancePersistentCacheTimestepP1P6(ref incrementalStatistics);
            }
            long elapsed = TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - repairStart);
            long localElapsed = incrementalStatistics.LocalBroadPhaseNanoseconds - localBefore;
            long exclusive = elapsed - localElapsed;
            incrementalStatistics.PairDiffNanoseconds += exclusive > 0L ? exclusive : 0L;
            incrementalStatistics.UsedIncrementalTopology = 1;
        }

        long classificationStart = ProfilerUnsafeUtility.Timestamp;
        if (TryReusePersistentContactViews(ref statistics, ref incrementalStatistics))
        {
            incrementalStatistics.SweptClassificationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - classificationStart);
            incrementalStatistics.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
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
        incrementalStatistics.SweptClassificationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - classificationStart);
        incrementalStatistics.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
        persistentViewReady = true;
        return true;
    }
''')

text = method(text, 'private bool ValidateAndClassifyIncrementalDirtyBodies(', 'private void ClearIncrementalDirtyBodySet()', '''    private bool IsPersistentCacheStructurallyReusableP1P6()
    {
        IncrementalContactCacheState state = IncrementalCacheState.Value;
        return state.IsValid != 0 &&
               state.BodyCount == States.Length &&
               PersistentSweptProxies.Length == States.Length &&
               PersistentProxyIndexByBody.Length == States.Length &&
               state.GuardMargin == math.max(0f, GuardEnvelopeMargin) &&
               state.PredictiveSkin == math.max(0f, PredictiveSkin) &&
               state.TimestepContactMargin == math.max(0f, TimestepContactMargin) &&
               state.SoftAvoidanceShell == math.max(0f, SoftAvoidanceShell) &&
               state.SoftAvoidanceResponseRate == math.max(0f, SoftAvoidanceResponseRate) &&
               state.RvoTimeHorizon == math.max(0f, RvoTimeHorizon) &&
               state.SubstepCount == math.max(1, SubstepCount) &&
               state.PredictivePairGenerationEnabled == (byte)(EnablePredictivePairGeneration ? 1 : 0) &&
               state.PredictiveContactsEnabled == (byte)(EnablePredictiveContacts ? 1 : 0) &&
               state.SoftAvoidanceVelocitySolver == (byte)SoftAvoidanceVelocitySolver;
    }

    private static PersistentSweptProxy BuildPersistentProxyFromStateP1P6(
        int bodyIndex,
        FlowMovementFrameState state,
        float guardMargin,
        float softAvoidanceShell,
        float softAvoidanceResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon)
    {
        PersistentSweptProxy proxy = new PersistentSweptProxy
        {
            Entity = state.Entity,
            BodyIndex = bodyIndex,
            IsValid = (byte)(state.IsInsideGrid ? 1 : 0),
            Radius = math.max(0f, state.Radius)
        };
        if (!state.IsInsideGrid)
            return proxy;
        proxy.TightMin = state.TimestepInteractionEnvelopeMin;
        proxy.TightMax = state.TimestepInteractionEnvelopeMax;
        proxy.GuardMin = proxy.TightMin - math.max(0f, guardMargin);
        proxy.GuardMax = proxy.TightMax + math.max(0f, guardMargin);
        proxy.TrajectoryStart = state.TimestepStartPosition.xz;
        proxy.TrajectoryEnd = state.TimestepPredictedPosition.xz;
        proxy.AvoidanceHorizonEnd =
            softSolverMode == SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle &&
            softAvoidanceShell > 0f && softAvoidanceResponseRate > 0f
                ? state.TimestepStartPosition.xz +
                  state.BasePredictedVelocity.xz * math.max(0f, rvoTimeHorizon)
                : state.TimestepPredictedPosition.xz;
        proxy.MotionVersion = 1u;
        return proxy;
    }

    private static IncrementalBodyDirtyFlags ClassifyAndUpdatePersistentProxyForBodyP1P6(
        int bodyIndex,
        FlowMovementFrameState state,
        NativeArray<PersistentSweptProxy> persistentProxies,
        NativeArray<int> proxyIndexByBody,
        IncrementalContactCacheState cacheState,
        float guardMargin,
        float softAvoidanceShell,
        float softAvoidanceResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon)
    {
        if (cacheState.IsValid == 0 ||
            proxyIndexByBody.Length != cacheState.BodyCount ||
            persistentProxies.Length != cacheState.BodyCount ||
            (uint)bodyIndex >= (uint)proxyIndexByBody.Length)
            return IncrementalBodyDirtyFlags.None;

        int proxyIndex = proxyIndexByBody[bodyIndex];
        if ((uint)proxyIndex >= (uint)persistentProxies.Length)
            return IncrementalBodyDirtyFlags.EntitySet |
                   IncrementalBodyDirtyFlags.Topology |
                   IncrementalBodyDirtyFlags.Motion;

        PersistentSweptProxy previous = persistentProxies[proxyIndex];
        if (previous.Entity != state.Entity)
            return IncrementalBodyDirtyFlags.EntitySet |
                   IncrementalBodyDirtyFlags.Topology |
                   IncrementalBodyDirtyFlags.Motion;

        PersistentSweptProxy current = BuildPersistentProxyFromStateP1P6(
            bodyIndex, state, guardMargin, softAvoidanceShell,
            softAvoidanceResponseRate, softSolverMode, rvoTimeHorizon);
        AssignMotionVersion(ref current, previous);
        bool topologyDirty = previous.IsValid != current.IsValid ||
                             previous.Radius != current.Radius ||
                             (current.IsValid != 0 && !AabbContains(
                                 previous.GuardMin, previous.GuardMax,
                                 current.TightMin, current.TightMax));
        bool motionDirty = topologyDirty || current.MotionVersion != previous.MotionVersion;
        if (!motionDirty)
            return IncrementalBodyDirtyFlags.None;
        if (!topologyDirty)
        {
            current.GuardMin = previous.GuardMin;
            current.GuardMax = previous.GuardMax;
        }
        persistentProxies[proxyIndex] = current;
        return topologyDirty
            ? IncrementalBodyDirtyFlags.Motion | IncrementalBodyDirtyFlags.Topology
            : IncrementalBodyDirtyFlags.Motion;
    }

    private void PrepareInitialPersistentDirtyBodySet()
    {
        ClearIncrementalDirtyBodySet();
        if (!IsPersistentCacheStructurallyReusableP1P6())
            return;
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            IncrementalBodyDirtyFlags flags = ClassifyAndUpdatePersistentProxyForBodyP1P6(
                bodyIndex, States[bodyIndex], PersistentSweptProxies.AsArray(),
                PersistentProxyIndexByBody.AsArray(), IncrementalCacheState.Value,
                GuardEnvelopeMargin, SoftAvoidanceShell, SoftAvoidanceResponseRate,
                SoftAvoidanceVelocitySolver, RvoTimeHorizon);
            if (flags != IncrementalBodyDirtyFlags.None)
                SetIncrementalDirtyFlags(bodyIndex, flags);
        }
    }

    private bool RefreshPreparedIncrementalDirtyBodiesP1P6(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        out int topologyDirtyCount)
    {
        topologyDirtyCount = 0;
        if (!IsPersistentCacheStructurallyReusableP1P6())
            return false;
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            IncrementalDirtyBody dirty = IncrementalDirtyBodies[dirtyIndex];
            int bodyIndex = dirty.BodyIndex;
            if ((uint)bodyIndex >= (uint)States.Length)
                return false;
            IncrementalBodyDirtyFlags refreshed = ClassifyAndUpdatePersistentProxyForBodyP1P6(
                bodyIndex, States[bodyIndex], PersistentSweptProxies.AsArray(),
                PersistentProxyIndexByBody.AsArray(), IncrementalCacheState.Value,
                GuardEnvelopeMargin, SoftAvoidanceShell, SoftAvoidanceResponseRate,
                SoftAvoidanceVelocitySolver, RvoTimeHorizon);
            IncrementalBodyDirtyFlags merged = dirty.Flags | refreshed |
                                               IncrementalBodyDirtyFlags.Motion;
            if ((merged & IncrementalBodyDirtyFlags.EntitySet) != 0)
                return false;
            dirty.Flags = merged;
            IncrementalDirtyBodies[dirtyIndex] = dirty;
            IncrementalDirtyFlagsByBody[bodyIndex] = (byte)merged;
        }
        SummarizePreparedIncrementalDirtyBodiesP1P6(
            ref incrementalStatistics, out topologyDirtyCount, out _);
        return true;
    }

    private void SummarizePreparedIncrementalDirtyBodiesP1P6(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        out int topologyDirtyCount,
        out bool entitySetDirty)
    {
        topologyDirtyCount = 0;
        entitySetDirty = false;
        incrementalStatistics.ProxyCount = PersistentSweptProxies.Length;
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            IncrementalBodyDirtyFlags flags = IncrementalDirtyBodies[dirtyIndex].Flags;
            if ((flags & IncrementalBodyDirtyFlags.EntitySet) != 0)
                entitySetDirty = true;
            if ((flags & IncrementalBodyDirtyFlags.Topology) != 0)
            {
                topologyDirtyCount++;
                incrementalStatistics.TopologyDirtyBodyCount++;
            }
            else if ((flags & IncrementalBodyDirtyFlags.Motion) != 0)
            {
                incrementalStatistics.MotionDirtyBodyCount++;
            }
        }
    }

    private void AdvancePersistentCacheTimestepP1P6(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        IncrementalContactCacheState state = IncrementalCacheState.Value;
        state.Timestep++;
        state.LastUpdateWasFullRebuild = 0;
        state.BodyCount = States.Length;
        state.NeighborPairCount = PersistentNeighborPairs.Length;
        IncrementalCacheState.Value = state;
        incrementalStatistics.Timestep = state.Timestep;
        incrementalStatistics.NeighborPairRetainedCount = PersistentNeighborPairs.Length;
    }

    private void RebuildPersistentProxyIndexByBodyP1P6()
    {
        PersistentProxyIndexByBody.ResizeUninitialized(States.Length);
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
            PersistentProxyIndexByBody[bodyIndex] = -1;
        for (int proxyIndex = 0; proxyIndex < PersistentSweptProxies.Length; proxyIndex++)
        {
            int bodyIndex = PersistentSweptProxies[proxyIndex].BodyIndex;
            if ((uint)bodyIndex < (uint)PersistentProxyIndexByBody.Length)
                PersistentProxyIndexByBody[bodyIndex] = proxyIndex;
        }
    }
''')

text = rep(text, '''    private void ClearIncrementalDirtyBodySet()
    {
        IncrementalDirtyBodies.Clear();
        for (int bodyIndex = 0;
             bodyIndex < IncrementalDirtyFlagsByBody.Length;
             bodyIndex++)
            IncrementalDirtyFlagsByBody[bodyIndex] = 0;
    }''', '''    private void ClearIncrementalDirtyBodySet()
    {
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            int bodyIndex = IncrementalDirtyBodies[dirtyIndex].BodyIndex;
            if ((uint)bodyIndex < (uint)IncrementalDirtyFlagsByBody.Length)
                IncrementalDirtyFlagsByBody[bodyIndex] = 0;
        }
        IncrementalDirtyBodies.Clear();
    }''', 'dirty clear')

old = '''            FlowMovementFrameState state = States[bodyIndex];
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

            CurrentIncrementalProxies.Add(proxy);'''
new = '''            CurrentIncrementalProxies.Add(BuildPersistentProxyFromStateP1P6(
                bodyIndex, States[bodyIndex], guardMargin, SoftAvoidanceShell,
                SoftAvoidanceResponseRate, SoftAvoidanceVelocitySolver,
                RvoTimeHorizon));'''
text = rep(text, old, new, 'full proxy build')
text = rep(text,
'        PersistentSweptProxies.AddRange(CurrentIncrementalProxies.AsArray());\n\n        float cellSize',
'        PersistentSweptProxies.AddRange(CurrentIncrementalProxies.AsArray());\n        RebuildPersistentProxyIndexByBodyP1P6();\n\n        float cellSize', 'mapping rebuild')

text = method(text, 'private bool TryIncrementallyRepairEscapedContactSet(', 'private bool MapDirtyIncidentNeighborPairsToCurrentBodies()', '''    private bool TryIncrementallyRepairEscapedContactSet(
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
        if (!RefreshPreparedIncrementalDirtyBodiesP1P6(
                ref incrementalStatistics, out int topologyDirtyCount))
        {
            incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - validationStart);
            return false;
        }
        float dirtyRatio = States.Length > 0 ? (float)escapedBodyCount / States.Length : 1f;
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
        long localBefore = incrementalStatistics.LocalBroadPhaseNanoseconds;
        if (topologyDirtyCount > 0)
            IncrementallyRepairPersistentNeighborTopology(ref incrementalStatistics, false);
        long pairDiffElapsed = TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - pairDiffStart);
        long localElapsed = incrementalStatistics.LocalBroadPhaseNanoseconds - localBefore;
        long pairDiffExclusive = pairDiffElapsed - localElapsed;
        incrementalStatistics.PairDiffNanoseconds += pairDiffExclusive > 0L
            ? pairDiffExclusive
            : 0L;

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
            ref statistics, ref incrementalStatistics, scheduleStartSubstep);
        incrementalStatistics.SweptClassificationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - classificationStart);
        RebuildEscapedTimestepContactView(ref statistics, ref incrementalStatistics);
        statistics.TimestepContactSetBuildNanoseconds += TimestampToNanoseconds(
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

P.write_text(text)
print('batch2_2 ok')
