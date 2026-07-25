using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public struct PersistentPairClassificationResult
{
    public BodyPair RawPair;
    public PersistentPredictiveContact Contact;
    public byte WasReclassified;
}

public struct PersistentClassificationPhaseState
{
    public uint Timestep;
    public uint ClassificationEpoch;
    public byte NeedsCommit;
}

#if RTS_CONTACT_DIAGNOSTICS
public struct PersistentClassificationTelemetryState
{
    public long BuildStartTimestamp;
    public long ClassificationStartTimestamp;
}
#endif

/// <summary>
/// P5B/P5C support for the staged Jacobi pipeline.
///
/// P5B keeps a persistent cell -> proxy membership view. It is rebuilt only
/// when guarded-proxy topology changes and is used to query dirty bodies without
/// scanning every persistent proxy. Capacity failure invalidates the view and
/// falls back to the authoritative full scan.
///
/// P5C separates persistent-pair classification into a serial prepare phase,
/// a pair-exclusive parallel evaluation phase and a deterministic serial commit.
/// </summary>
public partial struct InteractionCertificationJob
{
    private const int PersistentClassificationBatchSize = 64;



    internal JobHandle ScheduleInitialPersistentContactSetP1P6(
        NativeReference<ParallelJacobiExecutionState> runtimeState,
        JobHandle dependency)
    {
        InteractionCertificationJob prepare = this;
        prepare.Operation = InteractionCertificationOperation.PreparePersistentClassificationP1P6;
        prepare.RuntimeState = runtimeState;
        JobHandle handle = prepare.Schedule(dependency);

        var evaluateJob = new EvaluatePersistentPairClassificationsP1P6Job
        {
            Bodies = Bodies,
            NavigationStates = NavigationStates,
            MotionIntents = MotionIntents,
            MotionEvidence = MotionEvidence,
            StepStates = StepStates,
            RawPairs = ClassificationBodyPairs.AsDeferredJobArray(),
            PersistentProxies = PersistentSweptProxies.AsDeferredJobArray(),
            PreviousContacts = PersistentPredictiveContacts.AsDeferredJobArray(),
            DirtyFlagsByBody = IncrementalDirtyFlagsByBody,
            PhaseState = PersistentClassificationState,
            Results = PersistentClassificationResults.AsDeferredJobArray(),
            PredictiveSkin = Configuration.PredictiveSkin,
            TimestepContactMargin = Configuration.TimestepContactMargin,
            SoftAvoidanceShell = Configuration.SoftAvoidanceShell,
            SoftAvoidanceResponseRate = Configuration.SoftAvoidanceResponseRate,
            SoftAvoidanceVelocitySolver = Configuration.SoftAvoidanceVelocitySolver,
            RvoTimeHorizon = Configuration.RvoTimeHorizon,
            EnablePredictivePairGeneration =
                (byte)(Configuration.EnablePredictivePairGeneration ? 1 : 0),
            EnablePredictiveContacts =
                (byte)(Configuration.EnablePredictiveContacts ? 1 : 0),
            SubstepCount = math.max(1, Configuration.SubstepCount),
            ScheduleStartSubstep = 0
        };
        handle = evaluateJob.Schedule(
            PersistentClassificationResults,
            PersistentClassificationBatchSize,
            handle);

        InteractionCertificationJob commit = this;
        commit.Operation = InteractionCertificationOperation.CommitPersistentClassificationP1P6;
        commit.RuntimeState = runtimeState;
        return commit.Schedule(handle);
    }



    [BurstCompile]
    private struct EvaluatePersistentPairClassificationsP1P6Job : IJobParallelForDefer
    {
        [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
        [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
        [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
        [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
        [ReadOnly] public NativeArray<CrowdBodyStepState> StepStates;
        [ReadOnly] public NativeArray<BodyPair> RawPairs;
        [ReadOnly] public NativeArray<PersistentSweptProxy> PersistentProxies;
        [ReadOnly] public NativeArray<PersistentPredictiveContact> PreviousContacts;
        [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
        [ReadOnly] public NativeReference<PersistentClassificationPhaseState> PhaseState;
        public NativeArray<PersistentPairClassificationResult> Results;

        public float PredictiveSkin;
        public float TimestepContactMargin;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
        public float RvoTimeHorizon;
        public byte EnablePredictivePairGeneration;
        public byte EnablePredictiveContacts;
        public int SubstepCount;
        public int ScheduleStartSubstep;

        public void Execute(int pairIndex)
        {
            PersistentClassificationPhaseState phase = PhaseState.Value;
            BodyPair rawPair = RawPairs[pairIndex];
            CrowdBodySnapshot bodyASnapshot = Bodies[rawPair.BodyA];
            CrowdNavigationState bodyANavigation = NavigationStates[rawPair.BodyA];
            CrowdMotionIntent bodyAIntent = MotionIntents[rawPair.BodyA];
            CrowdMotionEvidence bodyAEvidence = MotionEvidence[rawPair.BodyA];
            CrowdBodyStepState bodyAStep = StepStates[rawPair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = Bodies[rawPair.BodyB];
            CrowdNavigationState bodyBNavigation = NavigationStates[rawPair.BodyB];
            CrowdMotionIntent bodyBIntent = MotionIntents[rawPair.BodyB];
            CrowdMotionEvidence bodyBEvidence = MotionEvidence[rawPair.BodyB];
            CrowdBodyStepState bodyBStep = StepStates[rawPair.BodyB];
            StableEntityPairKey key = StableEntityPairKey.Create(
                bodyASnapshot.Entity,
                bodyBSnapshot.Entity);

            bool hasProxyA = TryFindPersistentProxyP1P6(
                PersistentProxies,
                key.EntityA,
                out PersistentSweptProxy proxyA);
            bool hasProxyB = TryFindPersistentProxyP1P6(
                PersistentProxies,
                key.EntityB,
                out PersistentSweptProxy proxyB);
            bool hasPrevious = TryFindPersistentContactP1P6(
                PreviousContacts,
                key,
                out PersistentPredictiveContact previous);
            bool dirtyEndpoint =
                (IncrementalBodyDirtyFlags)DirtyFlagsByBody[rawPair.BodyA] !=
                    IncrementalBodyDirtyFlags.None ||
                (IncrementalBodyDirtyFlags)DirtyFlagsByBody[rawPair.BodyB] !=
                    IncrementalBodyDirtyFlags.None;
            bool canReuse = !dirtyEndpoint && hasPrevious &&
                            hasProxyA && hasProxyB &&
                            previous.ClassificationEpoch == phase.ClassificationEpoch &&
                            previous.MotionVersionA == proxyA.MotionVersion &&
                            previous.MotionVersionB == proxyB.MotionVersion;

            PersistentPairClassificationResult result = new PersistentPairClassificationResult
            {
                RawPair = rawPair,
                WasReclassified = (byte)(canReuse ? 0 : 1)
            };
            if (canReuse)
            {
                previous.LastSeenTimestep = phase.Timestep;
                result.Contact = previous;
            }
            else
            {
                result.Contact = ClassifyPersistentPairP1P6(
                    key,
                    rawPair,
                    bodyASnapshot,
                    bodyAEvidence,
                    bodyAStep,
                    bodyBSnapshot,
                    bodyBEvidence,
                    bodyBStep,
                    proxyA,
                    proxyB,
                    phase.Timestep,
                    phase.ClassificationEpoch,
                    ScheduleStartSubstep,
                    SubstepCount,
                    PredictiveSkin,
                    TimestepContactMargin,
                    SoftAvoidanceShell,
                    SoftAvoidanceResponseRate,
                    SoftAvoidanceVelocitySolver,
                    RvoTimeHorizon,
                    EnablePredictivePairGeneration != 0,
                    EnablePredictiveContacts != 0);
            }
            Results[pairIndex] = result;
        }
    }



    private void PreparePersistentClassificationP1P6(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        PersistentClassificationPhaseState phase = new PersistentClassificationPhaseState
        {
            NeedsCommit = 0
        };
#if RTS_CONTACT_DIAGNOSTICS
        PersistentClassificationTelemetryState telemetry = new PersistentClassificationTelemetryState
        {
            BuildStartTimestamp = ProfilerUnsafeUtility.Timestamp
        };
        PersistentClassificationTelemetry.Value = telemetry;
#endif
        PersistentClassificationResults.Clear();
        if (runtimeState.Value.IsValid == 0 ||
            !EnableTimestepContactSetCache ||
            !EnablePersistentContactCache)
        {
            PersistentClassificationState.Value = phase;
            return;
        }

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        PreviousTimestepContactPairs.Clear();

        bool needsClassification = RefreshPersistentPairSourceForClassification(
            ref statistics,
            ref incremental);
        if (needsClassification)
        {
#if RTS_CONTACT_DIAGNOSTICS
            telemetry.ClassificationStartTimestamp = ProfilerUnsafeUtility.Timestamp;
            PersistentClassificationTelemetry.Value = telemetry;
#endif
            phase.Timestep = IncrementalCacheState.Value.Timestep;
            phase.ClassificationEpoch = CalculateClassificationEpoch();
            ClassificationBodyPairs.Clear();
            ClassificationBodyPairs.AddRange(TimestepInteractionPairs.AsArray());
            PersistentClassificationResults.ResizeUninitialized(
                ClassificationBodyPairs.Length);
            phase.NeedsCommit = 1;
        }
        else
        {
#if RTS_CONTACT_DIAGNOSTICS
            FinalizePersistentBuildTimingP1P6(
                telemetry.BuildStartTimestamp,
                ref statistics);
#endif
        }

        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
        PersistentClassificationState.Value = phase;
    }

    private bool RefreshPersistentPairSourceForClassification(
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
        incrementalStatistics.ProxyValidationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        float dirtyRatio = Bodies.Length > 0
            ? (float)topologyDirtyCount / Bodies.Length
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
            long elapsed = ContactPipelineMath.TimestampToNanoseconds(
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
            long elapsed = ContactPipelineMath.TimestampToNanoseconds(
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
            incrementalStatistics.SweptClassificationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - classificationStart);
            incrementalStatistics.PersistentNeighborPairCount =
                PersistentNeighborPairs.Length;
            incrementalStatistics.CurrentInteractionPairCount =
                PersistentNeighborPairs.Length;
            ValidateSoftAvoidancePairViewAgainstQuadraticOracle(
                ref incrementalStatistics);
            CommitTimestepContactViews(
                ref statistics,
                ref incrementalStatistics,
                false);
            ValidateIncrementalContactSetAgainstQuadraticOracle(
                ref incrementalStatistics);
            return false;
        }

        long mappingStart = ProfilerUnsafeUtility.Timestamp;
        bool mapped = MapPersistentNeighborPairsToCurrentBodies();
        incrementalStatistics.PersistentPairMappingNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - mappingStart);
        if (mapped)
            return true;

        IncrementalContactCacheState invalidState = IncrementalCacheState.Value;
        invalidState.IsValid = 0;
        IncrementalCacheState.Value = invalidState;
        TimestepInteractionPairs.Clear();
        long fullSweepStart = ProfilerUnsafeUtility.Timestamp;
        BuildSweptInteractionPairs(ref statistics);
        incrementalStatistics.FullSweepSourceNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - fullSweepStart);
        incrementalStatistics.UsedFullRebuild = 1;
        incrementalStatistics.CurrentInteractionPairCount =
            TimestepInteractionPairs.Length;
        BuildSoftAvoidancePairViewFromInteractions(ref incrementalStatistics);
        ClassifyTimestepContacts(
            ref statistics,
            ref incrementalStatistics,
            0);
        CommitTimestepContactViews(
            ref statistics,
            ref incrementalStatistics,
            false);
        ValidateIncrementalContactSetAgainstQuadraticOracle(
            ref incrementalStatistics);
        return false;
    }
    private void CommitPersistentClassificationP1P6(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        PersistentClassificationPhaseState phase =
            PersistentClassificationState.Value;
        if (runtimeState.Value.IsValid == 0 || phase.NeedsCommit == 0)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        PredictiveContactScratch.Clear();
        PredictiveContactSchedule.Clear();
        SoftAvoidancePairs.Clear();
        Pairs.Clear();

        int retainedContactCount = 0;
        for (int pairIndex = 0;
             pairIndex < PersistentClassificationResults.Length;
             pairIndex++)
        {
            PersistentPairClassificationResult result =
                PersistentClassificationResults[pairIndex];
            BodyPair rawPair = result.RawPair;
            PersistentPredictiveContact contact = result.Contact;
            PredictiveContactScratch.Add(contact);
            if (result.WasReclassified != 0)
            {
                incremental.ReclassifiedPairEvaluationCount++;
                incremental.SweptClassificationEvaluationCount++;
            }
            else
            {
                incremental.ClassificationReuseCount++;
                incremental.ClassificationSkippedCount++;
            }

            AccumulatePersistentClassificationStatistics(contact, ref statistics);
            if (contact.SoftAvoidanceCandidate != 0)
            {
                SoftAvoidancePairs.Add(new BodyPair
                {
                    BodyA = rawPair.BodyA,
                    BodyB = rawPair.BodyB
                });
            }
            if (contact.Lifecycle == PersistentContactLifecycle.Expired)
                continue;

            retainedContactCount++;
            if (contact.Lifecycle == PersistentContactLifecycle.Dormant)
            {
                PredictiveContactSchedule.Add(new PredictiveContactScheduleEntry
                {
                    Key = contact.Key,
                    Substep = contact.NextCheckSubstep
                });
                continue;
            }
            Pairs.Add(BuildContactConstraintFromPersistentContact(
                rawPair.BodyA,
                rawPair.BodyB,
                contact));
        }

        if (PredictiveContactScratch.Length > 1)
            PredictiveContactScratch.AsArray().Sort(
                new PersistentPredictiveContactComparer());
        if (PredictiveContactSchedule.Length > 1)
            PredictiveContactSchedule.AsArray().Sort(
                new PredictiveContactScheduleEntryComparer());
        PredictiveContactScheduleCursor.Value = 0;
        if (Pairs.Length > 1)
            Pairs.AsArray().Sort(new ContactConstraintComparer());
        if (SoftAvoidancePairs.Length > 1)
            SoftAvoidancePairs.AsArray().Sort(new BodyPairComparer());

        PersistentPredictiveContacts.Clear();
        PersistentPredictiveContacts.AddRange(PredictiveContactScratch.AsArray());
        RebuildPersistentContactViews();
        statistics.CandidatePairCount += TimestepInteractionPairs.Length;
        statistics.ContactPairCount += retainedContactCount;
        incremental.CurrentInteractionPairCount = TimestepInteractionPairs.Length;
        incremental.CurrentSoftAvoidancePairCount = SoftAvoidancePairs.Length;
        RefreshCurrentContactStateGauges(ref incremental, Pairs.Length);
        incremental.PersistentViewRebuildCount++;
        incremental.PersistentNeighborPairCount = PersistentNeighborPairs.Length;

        IncrementalContactCacheState cacheState = IncrementalCacheState.Value;
        cacheState.ClassificationEpoch = CalculateClassificationEpoch();
        IncrementalCacheState.Value = cacheState;

        ValidateSoftAvoidancePairViewAgainstQuadraticOracle(ref incremental);
        CommitTimestepContactViews(ref statistics, ref incremental, false);
        ValidateIncrementalContactSetAgainstQuadraticOracle(ref incremental);
#if RTS_CONTACT_DIAGNOSTICS
        PersistentClassificationTelemetryState telemetry =
            PersistentClassificationTelemetry.Value;
        incremental.SweptClassificationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - telemetry.ClassificationStartTimestamp);
        FinalizePersistentBuildTimingP1P6(
            telemetry.BuildStartTimestamp,
            ref statistics);
#endif

        phase.NeedsCommit = 0;
        PersistentClassificationState.Value = phase;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

#if RTS_CONTACT_DIAGNOSTICS
    private void FinalizePersistentBuildTimingP1P6(
        long startTimestamp,
        ref PredictiveDiscContactStatistics statistics)
    {
        long elapsed = ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - startTimestamp);
        statistics.TimestepContactSetBuildNanoseconds += elapsed;
        statistics.PairGenerationNanoseconds += elapsed;
    }
#endif

    private bool RebuildPersistentSpatialMembershipP1P6(uint targetEpoch)
    {
        if (!PersistentSpatialMembership.IsCreated ||
            !PersistentSpatialMembershipEpoch.IsCreated ||
            CellRadius <= 0f ||
            math.any(GridDimensions <= 0))
            return false;

        int requiredEntryCount = 0;
        float cellSize = CellRadius * 2f;
        for (int proxyIndex = 0;
             proxyIndex < PersistentSweptProxies.Length;
             proxyIndex++)
        {
            PersistentSweptProxy proxy = PersistentSweptProxies[proxyIndex];
            if (proxy.IsValid == 0)
                continue;
            if (!TryGetPersistentMembershipCellBoundsP1P6(
                    proxy,
                    cellSize,
                    out int2 minCell,
                    out int2 maxCell))
                continue;
            requiredEntryCount +=
                (maxCell.x - minCell.x + 1) *
                (maxCell.y - minCell.y + 1);
        }

        if (requiredEntryCount > PersistentSpatialMembership.Capacity)
        {
            PersistentSpatialMembership.Clear();
            PersistentSpatialMembershipEpoch.Value = uint.MaxValue;
            return false;
        }

        PersistentSpatialMembership.Clear();
        for (int proxyIndex = 0;
             proxyIndex < PersistentSweptProxies.Length;
             proxyIndex++)
        {
            PersistentSweptProxy proxy = PersistentSweptProxies[proxyIndex];
            if (proxy.IsValid == 0 ||
                !TryGetPersistentMembershipCellBoundsP1P6(
                    proxy,
                    cellSize,
                    out int2 minCell,
                    out int2 maxCell))
                continue;
            for (int x = minCell.x; x <= maxCell.x; x++)
            {
                for (int y = minCell.y; y <= maxCell.y; y++)
                {
                    int cellIndex = FlowFieldUtils.GetFlatIndex(
                        new int2(x, y),
                        GridDimensions);
                    PersistentSpatialMembership.Add(cellIndex, proxyIndex);
                }
            }
        }
        PersistentSpatialMembershipEpoch.Value = targetEpoch;
        return true;
    }

    private bool TryAppendPersistentSpatialNeighborsP1P6(
        int dirtyProxyIndex,
        uint expectedEpoch,
        uint validatedTimestep,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        if (!PersistentSpatialMembership.IsCreated ||
            !PersistentSpatialMembershipEpoch.IsCreated ||
            PersistentSpatialMembershipEpoch.Value != expectedEpoch ||
            !PersistentSpatialVisitStamp.IsCreated ||
            (uint)dirtyProxyIndex >= (uint)PersistentSweptProxies.Length)
            return false;

        PersistentSweptProxy dirtyProxy = PersistentSweptProxies[dirtyProxyIndex];
        float cellSize = CellRadius * 2f;
        if (!TryGetPersistentMembershipCellBoundsP1P6(
                dirtyProxy,
                cellSize,
                out int2 minCell,
                out int2 maxCell))
            return true;

        uint stamp = PersistentSpatialVisitStamp.Value + 1u;
        if (stamp == 0u)
        {
            for (int i = 0; i < PersistentSpatialVisitStampByProxy.Length; i++)
                PersistentSpatialVisitStampByProxy[i] = 0u;
            stamp = 1u;
        }
        PersistentSpatialVisitStamp.Value = stamp;
        if ((uint)dirtyProxyIndex < (uint)PersistentSpatialVisitStampByProxy.Length)
            PersistentSpatialVisitStampByProxy[dirtyProxyIndex] = stamp;

        for (int x = minCell.x; x <= maxCell.x; x++)
        {
            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                int cellIndex = FlowFieldUtils.GetFlatIndex(
                    new int2(x, y),
                    GridDimensions);
                NativeParallelMultiHashMapIterator<int> iterator;
                if (!PersistentSpatialMembership.TryGetFirstValue(
                        cellIndex,
                        out int otherProxyIndex,
                        out iterator))
                    continue;
                do
                {
                    if ((uint)otherProxyIndex >=
                            (uint)PersistentSpatialVisitStampByProxy.Length ||
                        PersistentSpatialVisitStampByProxy[otherProxyIndex] == stamp)
                        continue;
                    PersistentSpatialVisitStampByProxy[otherProxyIndex] = stamp;
                    PersistentSweptProxy other =
                        PersistentSweptProxies[otherProxyIndex];
                    if (other.IsValid == 0 || other.Entity == dirtyProxy.Entity)
                        continue;
                    incrementalStatistics.LocalProxyQueryCount++;
                    if (!ContactPipelineShared.AabbOverlaps(
                            dirtyProxy.GuardMin,
                            dirtyProxy.GuardMax,
                            other.GuardMin,
                            other.GuardMax))
                        continue;
                    IncrementalNeighborPairScratch.Add(new PersistentNeighborPair
                    {
                        Key = StableEntityPairKey.Create(
                            dirtyProxy.Entity,
                            other.Entity),
                        TopologyEpoch = expectedEpoch,
                        LastValidatedTimestep = validatedTimestep
                    });
                }
                while (PersistentSpatialMembership.TryGetNextValue(
                    out otherProxyIndex,
                    ref iterator));
            }
        }
        return true;
    }

    private bool TryGetPersistentMembershipCellBoundsP1P6(
        PersistentSweptProxy proxy,
        float cellSize,
        out int2 minCell,
        out int2 maxCell)
    {
        minCell = (int2)math.floor((proxy.GuardMin - GridOrigin.xz) / cellSize);
        maxCell = (int2)math.floor((proxy.GuardMax - GridOrigin.xz) / cellSize);
        if (maxCell.x < 0 || maxCell.y < 0 ||
            minCell.x >= GridDimensions.x || minCell.y >= GridDimensions.y)
            return false;
        minCell = math.clamp(minCell, int2.zero, GridDimensions - 1);
        maxCell = math.clamp(maxCell, int2.zero, GridDimensions - 1);
        return true;
    }

    private static bool TryFindPersistentProxyP1P6(
        NativeArray<PersistentSweptProxy> proxies,
        Entity entity,
        out PersistentSweptProxy proxy)
    {
        int low = 0;
        int high = proxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            PersistentSweptProxy candidate = proxies[middle];
            int comparison = StableEntityPairKey.CompareEntity(candidate.Entity, entity);
            if (comparison == 0)
            {
                proxy = candidate;
                return candidate.IsValid != 0;
            }
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        proxy = default;
        return false;
    }

    private static bool TryFindPersistentContactP1P6(
        NativeArray<PersistentPredictiveContact> contacts,
        StableEntityPairKey key,
        out PersistentPredictiveContact contact)
    {
        int low = 0;
        int high = contacts.Length - 1;
        var comparer = new StableEntityPairKeyComparer();
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            PersistentPredictiveContact candidate = contacts[middle];
            int comparison = comparer.Compare(candidate.Key, key);
            if (comparison == 0)
            {
                contact = candidate;
                return true;
            }
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        contact = default;
        return false;
    }

    private static PersistentPredictiveContact ClassifyPersistentPairP1P6(
        StableEntityPairKey key,
        BodyPair rawPair,
        CrowdBodySnapshot bodyA,
        CrowdMotionEvidence evidenceA,
        CrowdBodyStepState stepA,
        CrowdBodySnapshot bodyB,
        CrowdMotionEvidence evidenceB,
        CrowdBodyStepState stepB,
        PersistentSweptProxy proxyA,
        PersistentSweptProxy proxyB,
        uint timestep,
        uint classificationEpoch,
        int scheduleStartSubstep,
        int substepCount,
        float predictiveSkin,
        float timestepContactMargin,
        float softAvoidanceShell,
        float softAvoidanceResponseRate,
        SoftAvoidanceVelocitySolverMode softAvoidanceVelocitySolver,
        float rvoTimeHorizon,
        bool enablePredictivePairGeneration,
        bool enablePredictiveContacts)
    {
        float radiusSum = bodyA.Radius + bodyB.Radius;
        float3 relativeStart =
            evidenceB.TrajectoryStart - evidenceA.TrajectoryStart;
        float3 relativeDisplacement =
            (evidenceB.BaselineEnd - evidenceB.TrajectoryStart) -
            (evidenceA.BaselineEnd - evidenceA.TrajectoryStart);
        relativeStart.y = 0f;
        relativeDisplacement.y = 0f;
        float relativeLengthSq = math.lengthsq(relativeDisplacement);
        float closestTime = relativeLengthSq > 0.0000001f
            ? math.clamp(
                -math.dot(relativeStart, relativeDisplacement) /
                relativeLengthSq,
                0f,
                1f)
            : 0f;
        float minDistanceSq = math.lengthsq(
            relativeStart + closestTime * relativeDisplacement);
        float candidateDistance = radiusSum + math.max(0f, predictiveSkin);
        float retainedDistance = candidateDistance +
                                 math.max(0f, timestepContactMargin) * 2f;
        float startDistanceSq = math.lengthsq(relativeStart);
        float3 endDelta =
            evidenceB.BaselineEnd - evidenceA.BaselineEnd;
        endDelta.y = 0f;
        float endDistanceSq = math.lengthsq(endDelta);
        float radiusSumSq = radiusSum * radiusSum;

        PersistentContactLifecycle lifecycle;
        ContactConstraintMode contactMode = ContactConstraintMode.Regular;
        if (minDistanceSq > retainedDistance * retainedDistance ||
            (startDistanceSq > radiusSumSq && !enablePredictivePairGeneration))
        {
            lifecycle = PersistentContactLifecycle.Expired;
        }
        else if (startDistanceSq <= radiusSumSq)
        {
            lifecycle = PersistentContactLifecycle.Actual;
        }
        else if (minDistanceSq > candidateDistance * candidateDistance)
        {
            lifecycle = PersistentContactLifecycle.Dormant;
        }
        else
        {
            bool preventSideExchange =
                endDistanceSq >= radiusSumSq &&
                minDistanceSq <= radiusSumSq;
            lifecycle = preventSideExchange && enablePredictiveContacts
                ? PersistentContactLifecycle.Predictive
                : PersistentContactLifecycle.Approaching;
            contactMode = lifecycle == PersistentContactLifecycle.Predictive
                ? ContactConstraintMode.Predictive
                : ContactConstraintMode.Regular;
        }

        float3 stableNormal = evidenceA.TrajectoryStart -
                              evidenceB.TrajectoryStart;
        stableNormal.y = 0f;
        stableNormal = math.normalizesafe(
            stableNormal,
            ContactPipelineMath.DeterministicFallbackNormal(rawPair.BodyA, rawPair.BodyB));

        ushort firstPossibleSubstep = 0;
        if (lifecycle == PersistentContactLifecycle.Dormant)
        {
            int totalSubstepCount = math.max(1, substepCount);
            if (relativeLengthSq <= 0.0000001f ||
                scheduleStartSubstep >= totalSubstepCount)
            {
                firstPossibleSubstep = ushort.MaxValue;
            }
            else
            {
                int remaining = math.max(
                    1,
                    totalSubstepCount - scheduleStartSubstep);
                int closestOffset = math.clamp(
                    (int)math.floor(closestTime * remaining),
                    0,
                    remaining - 1);
                firstPossibleSubstep = (ushort)(scheduleStartSubstep +
                    math.max(0, closestOffset - 1));
            }
        }

        return new PersistentPredictiveContact
        {
            Key = key,
            StableNormal = stableNormal,
            Lifecycle = lifecycle,
            ContactMode = contactMode,
            FixedSide = contactMode == ContactConstraintMode.Predictive
                ? (sbyte)1
                : (sbyte)0,
            SoftAvoidanceCandidate = (byte)(CouldEnterSoftRangeP1P6(
                bodyA,
                evidenceA,
                stepA,
                bodyB,
                evidenceB,
                stepB,
                softAvoidanceShell,
                softAvoidanceResponseRate,
                softAvoidanceVelocitySolver,
                rvoTimeHorizon) ? 1 : 0),
            FirstPossibleSubstep = firstPossibleSubstep,
            NextCheckSubstep = firstPossibleSubstep,
            ClosestTime = closestTime,
            LastSeenTimestep = timestep,
            MotionVersionA = proxyA.MotionVersion,
            MotionVersionB = proxyB.MotionVersion,
            ClassificationEpoch = classificationEpoch
        };
    }

    private static bool CouldEnterSoftRangeP1P6(
        CrowdBodySnapshot bodyA,
        CrowdMotionEvidence evidenceA,
        CrowdBodyStepState stepA,
        CrowdBodySnapshot bodyB,
        CrowdMotionEvidence evidenceB,
        CrowdBodyStepState stepB,
        float softShell,
        float responseRate,
        SoftAvoidanceVelocitySolverMode solverMode,
        float rvoTimeHorizon)
    {
        if (softShell <= 0f || responseRate <= 0f)
            return false;
        float maxDistance = bodyA.Radius + bodyB.Radius + math.max(0f, softShell);
        float3 relativeStart = evidenceB.TrajectoryStart -
                               evidenceA.TrajectoryStart;
        float3 relativeTimestepDisplacement =
            (evidenceB.BaselineEnd - evidenceB.TrajectoryStart) -
            (evidenceA.BaselineEnd - evidenceA.TrajectoryStart);
        relativeStart.y = 0f;
        relativeTimestepDisplacement.y = 0f;
        if (CouldRelativePathApproachP1P6(
                relativeStart,
                relativeTimestepDisplacement,
                maxDistance))
            return true;
        if (solverMode != SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle)
            return false;
        float3 relativeHorizonDisplacement =
            (stepB.BaseVelocity - stepA.BaseVelocity) *
            math.max(0f, rvoTimeHorizon);
        relativeHorizonDisplacement.y = 0f;
        return CouldRelativePathApproachP1P6(
            relativeStart,
            relativeHorizonDisplacement,
            maxDistance);
    }

    private static bool CouldRelativePathApproachP1P6(
        float3 relativeStart,
        float3 relativeDisplacement,
        float maxDistance)
    {
        float relativeLengthSq = math.lengthsq(relativeDisplacement);
        float closestTime = relativeLengthSq > 0.0000001f
            ? math.clamp(
                -math.dot(relativeStart, relativeDisplacement) /
                relativeLengthSq,
                0f,
                1f)
            : 0f;
        float minDistanceSq = math.lengthsq(
            relativeStart + closestTime * relativeDisplacement);
        return minDistanceSq <= maxDistance * maxDistance;
    }
}
}
