using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct SolveXpbdUnitContactsJob
{
    private const float IncrementalDirtyBodyRatioThreshold = 0.35f;

    private bool BuildContactPairsFromPersistentNeighborSet(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        bool forceFullRebuild,
        int scheduleStartSubstep,
        out bool persistentViewReady)
    {
        persistentViewReady = false;
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        PrepareCurrentBodyLookup();
        BuildCurrentIncrementalSweptProxies();
        bool cacheCanBePatched = !forceFullRebuild &&
                                 ValidateAndClassifyIncrementalDirtyBodies(
                                     ref incrementalStatistics);
        incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        int topologyDirtyCount = incrementalStatistics.TopologyDirtyBodyCount;
        float dirtyRatio = States.Length > 0
            ? (float)topologyDirtyCount / States.Length
            : 1f;
        bool useFullRebuild = !cacheCanBePatched ||
                              dirtyRatio > IncrementalDirtyBodyRatioThreshold;

        if (useFullRebuild)
        {
            ClearPersistentClassificationCache();
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
            long repairStart = ProfilerUnsafeUtility.Timestamp;
            long localBroadPhaseBefore = incrementalStatistics.LocalBroadPhaseNanoseconds;
            UpdatePersistentProxyMetadata();
            if (topologyDirtyCount > 0)
            {
                IncrementallyRepairPersistentNeighborTopology(
                    ref incrementalStatistics);
                incrementalStatistics.IncrementalRepairCount++;
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
            }

            long repairElapsed = TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - repairStart);
            long localBroadPhaseElapsed =
                incrementalStatistics.LocalBroadPhaseNanoseconds - localBroadPhaseBefore;
            long pairDiffExclusive = repairElapsed - localBroadPhaseElapsed;
            incrementalStatistics.PairDiffNanoseconds +=
                pairDiffExclusive > 0L ? pairDiffExclusive : 0L;
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

    private bool TryReusePersistentContactViews(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        IncrementalContactCacheState cacheState = IncrementalCacheState.Value;
        uint classificationEpoch = CalculateClassificationEpoch();
        if (IncrementalDirtyBodies.Length != 0 ||
            cacheState.ContactViewsValid == 0 ||
            cacheState.ClassificationEpoch != classificationEpoch)
            return false;

        Pairs.Clear();
        SoftAvoidancePairs.Clear();
        PredictiveContactSchedule.Clear();
        for (int keyIndex = 0;
             keyIndex < PersistentActiveContactKeys.Length;
             keyIndex++)
        {
            StableEntityPairKey key = PersistentActiveContactKeys[keyIndex];
            if (!TryFindCurrentBodyIndex(key.EntityA, out int bodyA) ||
                !TryFindCurrentBodyIndex(key.EntityB, out int bodyB) ||
                !TryFindPersistentPredictiveContact(
                    key,
                    out PersistentPredictiveContact contact))
                return false;
            Pairs.Add(BuildUnitCollisionPairFromPersistentContact(
                bodyA,
                bodyB,
                contact));
        }
        for (int keyIndex = 0;
             keyIndex < PersistentSoftAvoidancePairKeys.Length;
             keyIndex++)
        {
            StableEntityPairKey key = PersistentSoftAvoidancePairKeys[keyIndex];
            if (!TryFindCurrentBodyIndex(key.EntityA, out int bodyA) ||
                !TryFindCurrentBodyIndex(key.EntityB, out int bodyB))
                return false;
            SoftAvoidancePairs.Add(new UnitCollisionPair
            {
                BodyA = math.min(bodyA, bodyB),
                BodyB = math.max(bodyA, bodyB)
            });
        }
        PredictiveContactSchedule.AddRange(
            PersistentDormantContactSchedule.AsArray());
        PredictiveContactScheduleCursor.Value = 0;
        if (Pairs.Length > 1)
            Pairs.AsArray().Sort(new UnitCollisionPairComparer());
        if (SoftAvoidancePairs.Length > 1)
            SoftAvoidancePairs.AsArray().Sort(new UnitCollisionPairComparer());

        int sweptCount = cacheState.DormantContactCount +
                         cacheState.ApproachingContactCount +
                         cacheState.PredictiveContactCount +
                         cacheState.ActualContactCount;
        statistics.CandidatePairCount += PersistentNeighborPairs.Length;
        statistics.ContactPairCount += sweptCount;
        incrementalStatistics.ClassificationReuseCount +=
            PersistentNeighborPairs.Length;
        incrementalStatistics.ClassificationSkippedCount +=
            PersistentNeighborPairs.Length;
        incrementalStatistics.CurrentInteractionPairCount =
            PersistentNeighborPairs.Length;
        incrementalStatistics.CurrentSoftAvoidancePairCount =
            SoftAvoidancePairs.Length;
        incrementalStatistics.CurrentSweptContactCount = sweptCount;
        incrementalStatistics.CurrentDormantPairCount =
            cacheState.DormantContactCount;
        incrementalStatistics.CurrentApproachingPairCount =
            cacheState.ApproachingContactCount;
        incrementalStatistics.CurrentPredictivePairCount =
            cacheState.PredictiveContactCount;
        incrementalStatistics.CurrentActualPairCount =
            cacheState.ActualContactCount;
        incrementalStatistics.ExpiredPairCount +=
            cacheState.ExpiredContactCount;
        incrementalStatistics.PersistentViewReuseCount++;
        UpdateActiveConstraintGauges(
            ref incrementalStatistics,
            Pairs.Length);
        return true;
    }

    private void RebuildPersistentContactViews()
    {
        PersistentActiveContactKeys.Clear();
        PersistentSoftAvoidancePairKeys.Clear();
        PersistentDormantContactSchedule.Clear();
        int dormant = 0;
        int approaching = 0;
        int predictive = 0;
        int actual = 0;
        int expired = 0;
        for (int contactIndex = 0;
             contactIndex < PersistentPredictiveContacts.Length;
             contactIndex++)
        {
            PersistentPredictiveContact contact =
                PersistentPredictiveContacts[contactIndex];
            if (contact.SoftAvoidanceCandidate != 0)
                PersistentSoftAvoidancePairKeys.Add(contact.Key);
            switch (contact.Lifecycle)
            {
                case PersistentContactLifecycle.Dormant:
                    dormant++;
                    PersistentDormantContactSchedule.Add(
                        new PredictiveContactScheduleEntry
                        {
                            Key = contact.Key,
                            Substep = contact.NextCheckSubstep
                        });
                    break;
                case PersistentContactLifecycle.Approaching:
                    approaching++;
                    PersistentActiveContactKeys.Add(contact.Key);
                    break;
                case PersistentContactLifecycle.Predictive:
                    predictive++;
                    PersistentActiveContactKeys.Add(contact.Key);
                    break;
                case PersistentContactLifecycle.Actual:
                case PersistentContactLifecycle.Separating:
                    actual++;
                    PersistentActiveContactKeys.Add(contact.Key);
                    break;
                default:
                    expired++;
                    break;
            }
        }
        if (PersistentActiveContactKeys.Length > 1)
            PersistentActiveContactKeys.AsArray().Sort(
                new StableEntityPairKeyComparer());
        if (PersistentSoftAvoidancePairKeys.Length > 1)
            PersistentSoftAvoidancePairKeys.AsArray().Sort(
                new StableEntityPairKeyComparer());
        if (PersistentDormantContactSchedule.Length > 1)
            PersistentDormantContactSchedule.AsArray().Sort(
                new PredictiveContactScheduleEntryComparer());

        IncrementalContactCacheState state = IncrementalCacheState.Value;
        state.DormantContactCount = dormant;
        state.ApproachingContactCount = approaching;
        state.PredictiveContactCount = predictive;
        state.ActualContactCount = actual;
        state.ExpiredContactCount = expired;
        state.ContactViewsValid = 1;
        IncrementalCacheState.Value = state;
    }

    private void ClassifyOrReusePersistentNeighborPairs(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int scheduleStartSubstep)
    {
        PredictiveContactScratch.Clear();
        PredictiveContactSchedule.Clear();
        SoftAvoidancePairs.Clear();
        Pairs.Clear();
        uint timestep = IncrementalCacheState.Value.Timestep;
        uint classificationEpoch = CalculateClassificationEpoch();
        statistics.CandidatePairCount += TimestepInteractionPairs.Length;

        int retainedContactCount = 0;
        for (int pairIndex = 0;
             pairIndex < TimestepInteractionPairs.Length;
             pairIndex++)
        {
            UnitCollisionPair rawPair = TimestepInteractionPairs[pairIndex];
            FlowMovementFrameState bodyA = States[rawPair.BodyA];
            FlowMovementFrameState bodyB = States[rawPair.BodyB];
            StableEntityPairKey key = StableEntityPairKey.Create(
                bodyA.Entity,
                bodyB.Entity);

            bool hasProxyA = TryFindPersistentProxy(
                key.EntityA,
                out PersistentSweptProxy proxyA);
            bool hasProxyB = TryFindPersistentProxy(
                key.EntityB,
                out PersistentSweptProxy proxyB);
            bool hasPrevious = TryFindPersistentPredictiveContact(
                key,
                out PersistentPredictiveContact previous);
            bool dirtyEndpoint =
                GetDirtyFlags(rawPair.BodyA) != IncrementalBodyDirtyFlags.None ||
                GetDirtyFlags(rawPair.BodyB) != IncrementalBodyDirtyFlags.None;
            bool canReuse = !dirtyEndpoint && hasPrevious &&
                            hasProxyA && hasProxyB &&
                            previous.ClassificationEpoch == classificationEpoch &&
                            previous.MotionVersionA == proxyA.MotionVersion &&
                            previous.MotionVersionB == proxyB.MotionVersion;

            PersistentPredictiveContact contact;
            if (canReuse)
            {
                contact = previous;
                contact.LastSeenTimestep = timestep;
                incrementalStatistics.ClassificationReuseCount++;
                incrementalStatistics.ClassificationSkippedCount++;
            }
            else
            {
                contact = ClassifyPersistentNeighborPair(
                    key,
                    rawPair,
                    bodyA,
                    bodyB,
                    proxyA,
                    proxyB,
                    timestep,
                    classificationEpoch,
                    scheduleStartSubstep);
                incrementalStatistics.ReclassifiedPairEvaluationCount++;
                incrementalStatistics.SweptClassificationEvaluationCount++;
            }

            PredictiveContactScratch.Add(contact);
            AccumulatePersistentClassificationStatistics(
                contact,
                ref statistics);
            if (contact.SoftAvoidanceCandidate != 0)
            {
                SoftAvoidancePairs.Add(new UnitCollisionPair
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
                    Key = key,
                    Substep = contact.NextCheckSubstep
                });
                continue;
            }

            Pairs.Add(BuildUnitCollisionPairFromPersistentContact(
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
            Pairs.AsArray().Sort(new UnitCollisionPairComparer());
        if (SoftAvoidancePairs.Length > 1)
            SoftAvoidancePairs.AsArray().Sort(new UnitCollisionPairComparer());

        PersistentPredictiveContacts.Clear();
        PersistentPredictiveContacts.AddRange(PredictiveContactScratch.AsArray());
        RebuildPersistentContactViews();
        statistics.ContactPairCount += retainedContactCount;
        incrementalStatistics.CurrentInteractionPairCount =
            TimestepInteractionPairs.Length;
        incrementalStatistics.CurrentSoftAvoidancePairCount =
            SoftAvoidancePairs.Length;
        RefreshCurrentContactStateGauges(
            ref incrementalStatistics,
            Pairs.Length);
        incrementalStatistics.PersistentViewRebuildCount++;

        IncrementalContactCacheState cacheState = IncrementalCacheState.Value;
        cacheState.ClassificationEpoch = classificationEpoch;
        IncrementalCacheState.Value = cacheState;
    }

    private PersistentPredictiveContact ClassifyPersistentNeighborPair(
        StableEntityPairKey key,
        UnitCollisionPair rawPair,
        FlowMovementFrameState bodyA,
        FlowMovementFrameState bodyB,
        PersistentSweptProxy proxyA,
        PersistentSweptProxy proxyB,
        uint timestep,
        uint classificationEpoch,
        int scheduleStartSubstep)
    {
        float radiusSum = bodyA.Radius + bodyB.Radius;
        float3 relativeStart =
            bodyB.TimestepStartPosition - bodyA.TimestepStartPosition;
        float3 relativeDisplacement =
            (bodyB.TimestepPredictedPosition - bodyB.TimestepStartPosition) -
            (bodyA.TimestepPredictedPosition - bodyA.TimestepStartPosition);
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
        float candidateDistance = radiusSum + math.max(0f, PredictiveSkin);
        float retainedDistance = candidateDistance +
                                 math.max(0f, TimestepContactMargin) * 2f;
        float startDistanceSq = math.lengthsq(relativeStart);
        float3 endDelta =
            bodyB.TimestepPredictedPosition - bodyA.TimestepPredictedPosition;
        endDelta.y = 0f;
        float endDistanceSq = math.lengthsq(endDelta);
        float radiusSumSq = radiusSum * radiusSum;

        PersistentContactLifecycle lifecycle;
        UnitContactMode contactMode = UnitContactMode.Regular;
        if (minDistanceSq > retainedDistance * retainedDistance ||
            (startDistanceSq > radiusSumSq && !EnablePredictivePairGeneration))
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
            lifecycle = preventSideExchange && EnablePredictiveContacts
                ? PersistentContactLifecycle.Predictive
                : PersistentContactLifecycle.Approaching;
            contactMode = lifecycle == PersistentContactLifecycle.Predictive
                ? UnitContactMode.Predictive
                : UnitContactMode.Regular;
        }

        // Keep A1 classification exactly equivalent to A0. Previous-frame
        // normals are not a correctness input and are therefore not blended.
        float3 stableNormal = bodyA.TimestepStartPosition -
                              bodyB.TimestepStartPosition;
        stableNormal.y = 0f;
        stableNormal = math.normalizesafe(
            stableNormal,
            DeterministicFallbackNormal(rawPair.BodyA, rawPair.BodyB));

        ushort firstPossibleSubstep = 0;
        if (lifecycle == PersistentContactLifecycle.Dormant)
        {
            int totalSubstepCount = math.max(1, SubstepCount);
            if (relativeLengthSq <= 0.0000001f ||
                scheduleStartSubstep >= totalSubstepCount)
            {
                firstPossibleSubstep = ushort.MaxValue;
            }
            else
            {
                int remainingSubstepCount = math.max(
                    1,
                    totalSubstepCount - scheduleStartSubstep);
                int closestSubstepOffset = math.clamp(
                    (int)math.floor(closestTime * remainingSubstepCount),
                    0,
                    remainingSubstepCount - 1);
                firstPossibleSubstep = (ushort)(scheduleStartSubstep +
                    math.max(0, closestSubstepOffset - 1));
            }
        }

        return new PersistentPredictiveContact
        {
            Key = key,
            StableNormal = stableNormal,
            Lifecycle = lifecycle,
            ContactMode = contactMode,
            FixedSide = contactMode == UnitContactMode.Predictive
                ? (sbyte)1
                : (sbyte)0,
            SoftAvoidanceCandidate = (byte)(CouldEnterSoftAvoidanceRange(
                bodyA,
                bodyB) ? 1 : 0),
            FirstPossibleSubstep = firstPossibleSubstep,
            NextCheckSubstep = firstPossibleSubstep,
            ClosestTime = closestTime,
            LastSeenTimestep = timestep,
            MotionVersionA = proxyA.MotionVersion,
            MotionVersionB = proxyB.MotionVersion,
            ClassificationEpoch = classificationEpoch
        };
    }

    private static UnitCollisionPair BuildUnitCollisionPairFromPersistentContact(
        int firstBodyIndex,
        int secondBodyIndex,
        PersistentPredictiveContact contact)
    {
        return new UnitCollisionPair
        {
            BodyA = math.min(firstBodyIndex, secondBodyIndex),
            BodyB = math.max(firstBodyIndex, secondBodyIndex),
            PredictiveNormal = contact.StableNormal,
            ContactMode = contact.ContactMode,
            FirstActivatedSubstep = -1
        };
    }

    private static void AccumulatePersistentClassificationStatistics(
        PersistentPredictiveContact contact,
        ref PredictiveDiscContactStatistics statistics)
    {
        switch (contact.Lifecycle)
        {
            case PersistentContactLifecycle.Actual:
                statistics.ActualGeneratedPairCount++;
                break;
            case PersistentContactLifecycle.Predictive:
                statistics.PredictiveGeneratedPairCount++;
                statistics.PredictivePairCount++;
                statistics.PotentialPredictivePairCount++;
                break;
            case PersistentContactLifecycle.Approaching:
                statistics.PredictiveGeneratedPairCount++;
                break;
            case PersistentContactLifecycle.Dormant:
                statistics.TimestepContactSetDormantPairCount++;
                break;
        }
    }

    private uint CalculateClassificationEpoch()
    {
        uint flags = 0u;
        if (EnablePredictivePairGeneration)
            flags |= 1u;
        if (EnablePredictiveContacts)
            flags |= 2u;
        flags |= (uint)math.max(1, SubstepCount) << 8;
        flags |= (uint)SoftAvoidanceVelocitySolver << 24;

        uint first = math.hash(new uint4(
            math.asuint(math.max(0f, PredictiveSkin)),
            math.asuint(math.max(0f, TimestepContactMargin)),
            math.asuint(math.max(0f, SoftAvoidanceShell)),
            math.asuint(math.max(0f, SoftAvoidanceResponseRate))));
        uint second = math.hash(new uint2(
            math.asuint(math.max(0f, RvoTimeHorizon)),
            flags));
        return math.hash(new uint2(first, second));
    }

    private bool TryFindPersistentPredictiveContact(
        StableEntityPairKey key,
        out PersistentPredictiveContact contact)
    {
        int contactIndex = FindPersistentPredictiveContactIndex(key);
        if (contactIndex >= 0)
        {
            contact = PersistentPredictiveContacts[contactIndex];
            return true;
        }
        contact = default;
        return false;
    }

    private void BuildTimestepPredictiveSchedule(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int scheduleStartSubstep)
    {
        PrepareCurrentBodyLookup();
        PredictiveContactScratch.Clear();
        PredictiveContactSchedule.Clear();
        uint timestep = IncrementalCacheState.Value.Timestep;
        int totalSubstepCount = EnableTimestepContactSetCache
            ? math.max(1, SubstepCount)
            : 1;
        scheduleStartSubstep = math.clamp(
            scheduleStartSubstep,
            0,
            totalSubstepCount - 1);
        int remainingSubstepCount = totalSubstepCount - scheduleStartSubstep;

        for (int pairIndex = 0; pairIndex < Pairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = Pairs[pairIndex];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            StableEntityPairKey key = StableEntityPairKey.Create(bodyA.Entity, bodyB.Entity);

            PersistentContactLifecycle lifecycle;
            float3 currentDelta = bodyA.TimestepStartPosition - bodyB.TimestepStartPosition;
            currentDelta.y = 0f;
            float radiusSum = bodyA.Radius + bodyB.Radius;
            if (math.lengthsq(currentDelta) <= radiusSum * radiusSum)
                lifecycle = PersistentContactLifecycle.Actual;
            else if (pair.IsDormant != 0)
                lifecycle = PersistentContactLifecycle.Dormant;
            else if (pair.ContactMode == UnitContactMode.Predictive)
                lifecycle = PersistentContactLifecycle.Predictive;
            else
                lifecycle = PersistentContactLifecycle.Approaching;

            // 调度与稳定法线属于中层 InteractionSet 的派生结果。
            // 不读取上一帧接触状态，保证 A0B1 与 A1B1 只有来源成本不同。
            float3 stableNormal = pair.PredictiveNormal;
            sbyte fixedSide = pair.ContactMode == UnitContactMode.Predictive
                ? (sbyte)1
                : (sbyte)0;

            PersistentSweptProxy proxyA = default;
            PersistentSweptProxy proxyB = default;
            if (EnablePersistentContactCache)
            {
                TryFindPersistentProxy(bodyA.Entity, out proxyA);
                TryFindPersistentProxy(bodyB.Entity, out proxyB);
            }

            ushort firstPossibleSubstep = 0;
            if (lifecycle == PersistentContactLifecycle.Dormant)
            {
                if (!HasRelativeTimestepTrajectory(bodyA, bodyB))
                {
                    firstPossibleSubstep = ushort.MaxValue;
                }
                else
                {
                    float closestTime = CalculatePairClosestTime(bodyA, bodyB);
                    int closestSubstepOffset = math.clamp(
                        (int)math.floor(closestTime * remainingSubstepCount),
                        0,
                        remainingSubstepCount - 1);
                    // Wake one substep early. The retained contact margin is the safety
                    // budget for solver/RVO deviations; any larger deviation triggers
                    // the envelope-escape repair path.
                    firstPossibleSubstep = (ushort)(scheduleStartSubstep +
                        math.max(0, closestSubstepOffset - 1));
                }
            }

            PersistentPredictiveContact contact = new PersistentPredictiveContact
            {
                Key = key,
                StableNormal = stableNormal,
                Lifecycle = lifecycle,
                FixedSide = fixedSide,
                FirstPossibleSubstep = firstPossibleSubstep,
                NextCheckSubstep = firstPossibleSubstep,
                LastSeenTimestep = timestep,
                MotionVersionA = proxyA.MotionVersion,
                MotionVersionB = proxyB.MotionVersion
            };
            PredictiveContactScratch.Add(contact);

            if (lifecycle == PersistentContactLifecycle.Dormant)
            {
                PredictiveContactSchedule.Add(new PredictiveContactScheduleEntry
                {
                    Key = key,
                    Substep = firstPossibleSubstep
                });
            }
        }

        if (PredictiveContactScratch.Length > 1)
            PredictiveContactScratch.AsArray().Sort(new PersistentPredictiveContactComparer());
        if (PredictiveContactSchedule.Length > 1)
            PredictiveContactSchedule.AsArray().Sort(new PredictiveContactScheduleEntryComparer());
        PredictiveContactScheduleCursor.Value = 0;
        PersistentPredictiveContacts.Clear();
        if (EnablePersistentContactCache)
            PersistentPredictiveContacts.AddRange(PredictiveContactScratch.AsArray());

        // Dormant contacts live in B's timestep schedule, not in the active
        // XPBD view, so A0/A1 share the same constraint-utilization semantics.
        int activeWriteIndex = 0;
        for (int pairIndex = 0; pairIndex < Pairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = Pairs[pairIndex];
            if (pair.IsDormant != 0)
                continue;
            Pairs[activeWriteIndex++] = pair;
        }
        Pairs.ResizeUninitialized(activeWriteIndex);
        RefreshCurrentContactStateGauges(
            ref incrementalStatistics,
            activeWriteIndex);
    }

    private static float CalculatePairClosestTime(
        FlowMovementFrameState bodyA,
        FlowMovementFrameState bodyB)
    {
        float3 relativeStart =
            bodyB.TimestepStartPosition - bodyA.TimestepStartPosition;
        float3 relativeDisplacement =
            (bodyB.TimestepPredictedPosition - bodyB.TimestepStartPosition) -
            (bodyA.TimestepPredictedPosition - bodyA.TimestepStartPosition);
        relativeStart.y = 0f;
        relativeDisplacement.y = 0f;
        float relativeLengthSq = math.lengthsq(relativeDisplacement);
        return relativeLengthSq > 0.0000001f
            ? math.clamp(
                -math.dot(relativeStart, relativeDisplacement) / relativeLengthSq,
                0f,
                1f)
            : 0f;
    }

    private static bool HasRelativeTimestepTrajectory(
        FlowMovementFrameState bodyA,
        FlowMovementFrameState bodyB)
    {
        float3 relativeDisplacement =
            (bodyB.TimestepPredictedPosition - bodyB.TimestepStartPosition) -
            (bodyA.TimestepPredictedPosition - bodyA.TimestepStartPosition);
        relativeDisplacement.y = 0f;
        return math.lengthsq(relativeDisplacement) > 0.0000001f;
    }

    private void ActivateScheduledPredictiveContactsForSubstep(
        int substepIndex,
        int substepCount,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        int cursor = PredictiveContactScheduleCursor.Value;
        if (cursor >= PredictiveContactSchedule.Length)
            return;

        long activationStart = ProfilerUnsafeUtility.Timestamp;
        PredictiveContactScheduleScratch.Clear();
        bool addedPair = false;
        bool hasRescheduledEntries = false;
        while (cursor < PredictiveContactSchedule.Length)
        {
            PredictiveContactScheduleEntry entry = PredictiveContactSchedule[cursor];
            if (entry.Substep > substepIndex)
                break;
            cursor++;

            incrementalStatistics.ScheduledWakeupCount++;
            if (!TryFindCurrentBodyIndex(entry.Key.EntityA, out int bodyA) ||
                !TryFindCurrentBodyIndex(entry.Key.EntityB, out int bodyB))
            {
                MarkPersistentContactExpired(entry.Key);
                continue;
            }

            if (TryBuildCurrentScheduledPair(bodyA, bodyB, out UnitCollisionPair pair))
            {
                if (FindPairIndex(TimestepContactPairs, pair.BodyA, pair.BodyB) < 0)
                {
                    TimestepContactPairs.Add(pair);
                    addedPair = true;
                }
                UpdatePersistentContactAfterScheduledCheck(
                    entry.Key,
                    pair,
                    ushort.MaxValue);
                continue;
            }
            else if (substepIndex + 1 < substepCount)
            {
                ushort nextSubstep = (ushort)(substepIndex + 1);
                PredictiveContactScheduleScratch.Add(
                    new PredictiveContactScheduleEntry
                    {
                        Key = entry.Key,
                        Substep = nextSubstep
                    });
                UpdatePersistentContactNextCheck(entry.Key, nextSubstep);
                hasRescheduledEntries = true;
            }
            else
            {
                MarkPersistentContactExpired(entry.Key);
            }
        }

        if (hasRescheduledEntries)
        {
            for (int futureIndex = cursor;
                 futureIndex < PredictiveContactSchedule.Length;
                 futureIndex++)
            {
                PredictiveContactScheduleScratch.Add(
                    PredictiveContactSchedule[futureIndex]);
            }
            if (PredictiveContactScheduleScratch.Length > 1)
            {
                PredictiveContactScheduleScratch.AsArray().Sort(
                    new PredictiveContactScheduleEntryComparer());
            }
            PredictiveContactSchedule.Clear();
            PredictiveContactSchedule.AddRange(
                PredictiveContactScheduleScratch.AsArray());
            cursor = 0;
        }

        PredictiveContactScheduleCursor.Value = cursor;
        if (addedPair)
            TimestepContactPairs.AsArray().Sort(new UnitCollisionPairComparer());
        UpdateActiveConstraintGauges(
            ref incrementalStatistics,
            TimestepContactPairs.Length);
        incrementalStatistics.ContactActivationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - activationStart);
    }

    private void UpdatePersistentContactAfterScheduledCheck(
        StableEntityPairKey key,
        UnitCollisionPair pair,
        ushort nextCheckSubstep)
    {
        int contactIndex = FindPersistentPredictiveContactIndex(key);
        if (contactIndex < 0)
            return;

        PersistentPredictiveContact contact =
            PersistentPredictiveContacts[contactIndex];
        FlowMovementFrameState bodyA = States[pair.BodyA];
        FlowMovementFrameState bodyB = States[pair.BodyB];
        float3 delta = bodyA.PredictedPosition - bodyB.PredictedPosition;
        delta.y = 0f;
        float radiusSum = bodyA.Radius + bodyB.Radius;
        contact.Lifecycle = math.lengthsq(delta) <= radiusSum * radiusSum
            ? PersistentContactLifecycle.Actual
            : pair.ContactMode == UnitContactMode.Predictive
                ? PersistentContactLifecycle.Predictive
                : PersistentContactLifecycle.Approaching;
        contact.ContactMode = pair.ContactMode;
        contact.StableNormal = pair.PredictiveNormal;
        contact.NextCheckSubstep = nextCheckSubstep;
        PersistentPredictiveContacts[contactIndex] = contact;
        InvalidatePersistentContactViews();
    }

    private void UpdatePersistentContactNextCheck(
        StableEntityPairKey key,
        ushort nextCheckSubstep)
    {
        int contactIndex = FindPersistentPredictiveContactIndex(key);
        if (contactIndex < 0)
            return;
        PersistentPredictiveContact contact =
            PersistentPredictiveContacts[contactIndex];
        contact.NextCheckSubstep = nextCheckSubstep;
        PersistentPredictiveContacts[contactIndex] = contact;
        InvalidatePersistentContactViews();
    }

    private void MarkPersistentContactExpired(StableEntityPairKey key)
    {
        int contactIndex = FindPersistentPredictiveContactIndex(key);
        if (contactIndex < 0)
            return;
        PersistentPredictiveContact contact =
            PersistentPredictiveContacts[contactIndex];
        contact.Lifecycle = PersistentContactLifecycle.Expired;
        contact.NextCheckSubstep = ushort.MaxValue;
        PersistentPredictiveContacts[contactIndex] = contact;
        InvalidatePersistentContactViews();
    }

    private int FindPersistentPredictiveContactIndex(StableEntityPairKey key)
    {
        int low = 0;
        int high = PersistentPredictiveContacts.Length - 1;
        var comparer = new StableEntityPairKeyComparer();
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            int comparison = comparer.Compare(
                PersistentPredictiveContacts[middle].Key,
                key);
            if (comparison == 0)
                return middle;
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        return -1;
    }

    private void InvalidatePersistentContactViews()
    {
        if (!EnablePersistentContactCache)
            return;
        IncrementalContactCacheState state = IncrementalCacheState.Value;
        state.ContactViewsValid = 0;
        IncrementalCacheState.Value = state;
    }

    private void ClearPersistentClassificationCache()
    {
        PersistentPredictiveContacts.Clear();
        PersistentActiveContactKeys.Clear();
        PersistentSoftAvoidancePairKeys.Clear();
        PersistentDormantContactSchedule.Clear();
        IncrementalContactCacheState state = IncrementalCacheState.Value;
        state.ContactViewsValid = 0;
        IncrementalCacheState.Value = state;
    }

    private bool TryBuildCurrentScheduledPair(
        int firstBodyIndex,
        int secondBodyIndex,
        out UnitCollisionPair pair)
    {
        int bodyAIndex = math.min(firstBodyIndex, secondBodyIndex);
        int bodyBIndex = math.max(firstBodyIndex, secondBodyIndex);
        FlowMovementFrameState bodyA = States[bodyAIndex];
        FlowMovementFrameState bodyB = States[bodyBIndex];
        float radiusSum = bodyA.Radius + bodyB.Radius;
        float candidateDistance = radiusSum + math.max(0f, PredictiveSkin);

        float3 relativeStart = bodyB.PredictedPosition - bodyA.PredictedPosition;
        float3 relativeDisplacement =
            (bodyB.TimestepPredictedPosition - bodyB.PredictedPosition) -
            (bodyA.TimestepPredictedPosition - bodyA.PredictedPosition);
        relativeStart.y = 0f;
        relativeDisplacement.y = 0f;
        float relativeLengthSq = math.lengthsq(relativeDisplacement);
        float closestTime = relativeLengthSq > 0.0000001f
            ? math.clamp(
                -math.dot(relativeStart, relativeDisplacement) / relativeLengthSq,
                0f,
                1f)
            : 0f;
        float minDistanceSq = math.lengthsq(
            relativeStart + closestTime * relativeDisplacement);
        if (minDistanceSq > candidateDistance * candidateDistance)
        {
            pair = default;
            return false;
        }

        float startDistanceSq = math.lengthsq(relativeStart);
        float3 endDelta =
            bodyB.TimestepPredictedPosition - bodyA.TimestepPredictedPosition;
        endDelta.y = 0f;
        float endDistanceSq = math.lengthsq(endDelta);
        float radiusSumSq = radiusSum * radiusSum;
        bool isActual = startDistanceSq <= radiusSumSq;
        bool preventSideExchange =
            !isActual &&
            endDistanceSq >= radiusSumSq &&
            minDistanceSq <= radiusSumSq;

        pair = new UnitCollisionPair
        {
            BodyA = bodyAIndex,
            BodyB = bodyBIndex,
            ContactMode = preventSideExchange && EnablePredictiveContacts
                ? UnitContactMode.Predictive
                : UnitContactMode.Regular,
            PredictiveNormal = math.normalizesafe(
                bodyA.PredictedPosition - bodyB.PredictedPosition,
                DeterministicFallbackNormal(bodyAIndex, bodyBIndex)),
            FirstActivatedSubstep = -1
        };
        return true;
    }

    private void RefreshCurrentContactStateGauges(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int currentActiveConstraintCount)
    {
        incrementalStatistics.CurrentSweptContactCount =
            PredictiveContactScratch.Length;
        incrementalStatistics.CurrentDormantPairCount = 0;
        incrementalStatistics.CurrentApproachingPairCount = 0;
        incrementalStatistics.CurrentPredictivePairCount = 0;
        incrementalStatistics.CurrentActualPairCount = 0;

        for (int contactIndex = 0;
             contactIndex < PredictiveContactScratch.Length;
             contactIndex++)
        {
            switch (PredictiveContactScratch[contactIndex].Lifecycle)
            {
                case PersistentContactLifecycle.Dormant:
                    incrementalStatistics.CurrentDormantPairCount++;
                    break;
                case PersistentContactLifecycle.Approaching:
                    incrementalStatistics.CurrentApproachingPairCount++;
                    break;
                case PersistentContactLifecycle.Predictive:
                    incrementalStatistics.CurrentPredictivePairCount++;
                    break;
                case PersistentContactLifecycle.Actual:
                    incrementalStatistics.CurrentActualPairCount++;
                    break;
            }
        }

        UpdateActiveConstraintGauges(
            ref incrementalStatistics,
            currentActiveConstraintCount);
    }

    private static void UpdateActiveConstraintGauges(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int currentActiveConstraintCount)
    {
        incrementalStatistics.CurrentActiveConstraintCount =
            math.max(0, currentActiveConstraintCount);
        incrementalStatistics.PeakActiveConstraintCount = math.max(
            incrementalStatistics.PeakActiveConstraintCount,
            incrementalStatistics.CurrentActiveConstraintCount);
    }

    private bool ValidateAndClassifyIncrementalDirtyBodies(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        ClearIncrementalDirtyBodySet();
        IncrementalContactCacheState state = IncrementalCacheState.Value;
        if (state.IsValid == 0 ||
            state.BodyCount != States.Length ||
            PersistentSweptProxies.Length != CurrentIncrementalProxies.Length ||
            state.GuardMargin != math.max(0f, GuardEnvelopeMargin) ||
            state.PredictiveSkin != math.max(0f, PredictiveSkin) ||
            state.TimestepContactMargin != math.max(0f, TimestepContactMargin) ||
            state.SoftAvoidanceShell != math.max(0f, SoftAvoidanceShell) ||
            state.SoftAvoidanceResponseRate !=
                math.max(0f, SoftAvoidanceResponseRate) ||
            state.RvoTimeHorizon != math.max(0f, RvoTimeHorizon) ||
            state.SubstepCount != math.max(1, SubstepCount) ||
            state.PredictivePairGenerationEnabled !=
                (byte)(EnablePredictivePairGeneration ? 1 : 0) ||
            state.PredictiveContactsEnabled !=
                (byte)(EnablePredictiveContacts ? 1 : 0) ||
            state.SoftAvoidanceVelocitySolver !=
                (byte)SoftAvoidanceVelocitySolver)
            return false;

        int validProxyCount = 0;
        for (int proxyIndex = 0; proxyIndex < CurrentIncrementalProxies.Length; proxyIndex++)
        {
            PersistentSweptProxy current = CurrentIncrementalProxies[proxyIndex];
            PersistentSweptProxy previous = PersistentSweptProxies[proxyIndex];
            if (current.Entity != previous.Entity || current.IsValid != previous.IsValid)
                return false;
            AssignMotionVersion(ref current, previous);
            CurrentIncrementalProxies[proxyIndex] = current;

            IncrementalBodyDirtyFlags flags = IncrementalBodyDirtyFlags.None;
            if (current.IsValid != 0)
            {
                validProxyCount++;
                if (!AabbContains(
                        previous.GuardMin,
                        previous.GuardMax,
                        current.TightMin,
                        current.TightMax))
                {
                    flags |= IncrementalBodyDirtyFlags.Topology;
                    incrementalStatistics.TopologyDirtyBodyCount++;
                }
                else if (current.MotionVersion != previous.MotionVersion)
                {
                    flags |= IncrementalBodyDirtyFlags.Motion;
                    incrementalStatistics.MotionDirtyBodyCount++;
                }
            }

            if (flags != IncrementalBodyDirtyFlags.None)
                SetIncrementalDirtyFlags(current.BodyIndex, flags);
        }

        incrementalStatistics.ProxyCount = validProxyCount;
        return true;
    }

    private void UpdatePersistentProxyMetadata()
    {
        for (int proxyIndex = 0; proxyIndex < CurrentIncrementalProxies.Length; proxyIndex++)
        {
            PersistentSweptProxy current = CurrentIncrementalProxies[proxyIndex];
            PersistentSweptProxy previous = PersistentSweptProxies[proxyIndex];
            IncrementalBodyDirtyFlags flags = GetDirtyFlags(current.BodyIndex);
            if ((flags & IncrementalBodyDirtyFlags.Topology) != 0)
            {
                PersistentSweptProxies[proxyIndex] = current;
                continue;
            }

            // Motion-only changes keep the old guard envelope. This is the proof
            // that no new broad-phase neighbor can have appeared.
            previous.BodyIndex = current.BodyIndex;
            previous.TightMin = current.TightMin;
            previous.TightMax = current.TightMax;
            previous.TrajectoryStart = current.TrajectoryStart;
            previous.TrajectoryEnd = current.TrajectoryEnd;
            previous.AvoidanceHorizonEnd = current.AvoidanceHorizonEnd;
            previous.Radius = current.Radius;
            previous.MotionVersion = current.MotionVersion;
            previous.IsValid = current.IsValid;
            PersistentSweptProxies[proxyIndex] = previous;
        }
    }

    private void ClearIncrementalDirtyBodySet()
    {
        IncrementalDirtyBodies.Clear();
        for (int bodyIndex = 0;
             bodyIndex < IncrementalDirtyFlagsByBody.Length;
             bodyIndex++)
            IncrementalDirtyFlagsByBody[bodyIndex] = 0;
    }

    private void SetIncrementalDirtyFlags(
        int bodyIndex,
        IncrementalBodyDirtyFlags flags)
    {
        if ((uint)bodyIndex >= (uint)IncrementalDirtyFlagsByBody.Length)
            return;
        IncrementalBodyDirtyFlags previous =
            (IncrementalBodyDirtyFlags)IncrementalDirtyFlagsByBody[bodyIndex];
        IncrementalBodyDirtyFlags merged = previous | flags;
        IncrementalDirtyFlagsByBody[bodyIndex] = (byte)merged;
        if (previous == IncrementalBodyDirtyFlags.None)
        {
            IncrementalDirtyBodies.Add(new IncrementalDirtyBody
            {
                BodyIndex = bodyIndex,
                Flags = merged
            });
        }
    }

    private IncrementalBodyDirtyFlags GetDirtyFlags(int bodyIndex)
    {
        return (uint)bodyIndex < (uint)IncrementalDirtyFlagsByBody.Length
            ? (IncrementalBodyDirtyFlags)IncrementalDirtyFlagsByBody[bodyIndex]
            : IncrementalBodyDirtyFlags.EntitySet;
    }

    private bool IsTopologyDirtyEntity(Entity entity)
    {
        if (!TryFindCurrentBodyIndex(entity, out int bodyIndex))
            return true;
        return (GetDirtyFlags(bodyIndex) & IncrementalBodyDirtyFlags.Topology) != 0;
    }

    private int FindPersistentProxyIndex(Entity entity)
    {
        int low = 0;
        int high = PersistentSweptProxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            int comparison = StableEntityPairKey.CompareEntity(
                PersistentSweptProxies[middle].Entity,
                entity);
            if (comparison == 0)
                return middle;
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        return -1;
    }

    private void IncrementallyRepairPersistentNeighborTopology(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        bool advanceTimestep = true)
    {
        long queryStart = ProfilerUnsafeUtility.Timestamp;
        IncrementalNeighborPairScratch.Clear();
        int previousPairCount = PersistentNeighborPairs.Length;

        // Retain only edges whose endpoints both kept their guard envelope.
        for (int pairIndex = 0; pairIndex < PersistentNeighborPairs.Length; pairIndex++)
        {
            PersistentNeighborPair pair = PersistentNeighborPairs[pairIndex];
            if (IsTopologyDirtyEntity(pair.Key.EntityA) ||
                IsTopologyDirtyEntity(pair.Key.EntityB))
                continue;
            IncrementalNeighborPairScratch.Add(pair);
        }

        int retainedPairCount = IncrementalNeighborPairScratch.Length;
        uint nextTopologyEpoch = IncrementalCacheState.Value.TopologyEpoch + 1u;
        uint nextTimestep = IncrementalCacheState.Value.Timestep +
                            (advanceTimestep ? 1u : 0u);
        bool spatialMembershipReady =
            RebuildPersistentSpatialMembershipP1P6(nextTopologyEpoch);
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            IncrementalDirtyBody dirty = IncrementalDirtyBodies[dirtyIndex];
            if ((GetDirtyFlags(dirty.BodyIndex) &
                 IncrementalBodyDirtyFlags.Topology) == 0)
                continue;

            FlowMovementFrameState dirtyState = States[dirty.BodyIndex];
            if (!TryFindPersistentProxy(
                    dirtyState.Entity,
                    out PersistentSweptProxy dirtyProxy) ||
                dirtyProxy.IsValid == 0)
                continue;

            int dirtyProxyIndex = FindPersistentProxyIndex(dirtyState.Entity);
            if (spatialMembershipReady && dirtyProxyIndex >= 0 &&
                TryAppendPersistentSpatialNeighborsP1P6(
                    dirtyProxyIndex,
                    nextTopologyEpoch,
                    nextTimestep,
                    ref incrementalStatistics))
                continue;

            // Capacity failure or an invalid membership epoch takes the original
            // authoritative O(N) path. This is a correctness fallback, not a partial query.
            for (int proxyIndex = 0; proxyIndex < PersistentSweptProxies.Length; proxyIndex++)
            {
                PersistentSweptProxy other = PersistentSweptProxies[proxyIndex];
                if (other.IsValid == 0 || other.Entity == dirtyProxy.Entity)
                    continue;
                incrementalStatistics.LocalProxyQueryCount++;
                if (!AabbOverlaps(
                        dirtyProxy.GuardMin,
                        dirtyProxy.GuardMax,
                        other.GuardMin,
                        other.GuardMax))
                    continue;

                IncrementalNeighborPairScratch.Add(new PersistentNeighborPair
                {
                    Key = StableEntityPairKey.Create(dirtyProxy.Entity, other.Entity),
                    TopologyEpoch = nextTopologyEpoch,
                    LastValidatedTimestep = nextTimestep
                });
            }
        }

        SortAndDeduplicatePersistentNeighborPairs(IncrementalNeighborPairScratch);
        PersistentNeighborPairs.Clear();
        PersistentNeighborPairs.AddRange(IncrementalNeighborPairScratch.AsArray());

        IncrementalContactCacheState state = IncrementalCacheState.Value;
        state.IsValid = 1;
        state.LastUpdateWasFullRebuild = 0;
        state.Timestep = nextTimestep;
        state.TopologyEpoch = nextTopologyEpoch;
        state.BodyCount = States.Length;
        state.NeighborPairCount = PersistentNeighborPairs.Length;
        IncrementalCacheState.Value = state;

        incrementalStatistics.Timestep = state.Timestep;
        incrementalStatistics.NeighborPairRetainedCount = retainedPairCount;
        incrementalStatistics.NeighborPairRemovedCount =
            previousPairCount - retainedPairCount;
        incrementalStatistics.NeighborPairAddedCount =
            math.max(0, PersistentNeighborPairs.Length - retainedPairCount);
        incrementalStatistics.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
        incrementalStatistics.LocalBroadPhaseNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - queryStart);
    }

    private bool TryIncrementallyRepairEscapedContactSet(
        int substepIndex,
        int scheduleStartSubstep,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        PrepareCurrentBodyLookup();
        BuildCurrentIncrementalSweptProxies();
        ClearIncrementalDirtyBodySet();

        int escapedBodyCount = 0;
        int topologyDirtyCount = 0;
        for (int proxyIndex = 0; proxyIndex < CurrentIncrementalProxies.Length; proxyIndex++)
        {
            PersistentSweptProxy current = CurrentIncrementalProxies[proxyIndex];
            FlowMovementFrameState state = States[current.BodyIndex];
            if (state.TimestepEscaped == 0)
                continue;

            escapedBodyCount++;
            IncrementalBodyDirtyFlags flags =
                IncrementalBodyDirtyFlags.Motion |
                IncrementalBodyDirtyFlags.CorrectedEscape;
            if (!TryFindPersistentProxy(current.Entity, out PersistentSweptProxy previous) ||
                previous.IsValid != current.IsValid ||
                (current.IsValid != 0 && !AabbContains(
                    previous.GuardMin,
                    previous.GuardMax,
                    current.TightMin,
                    current.TightMax)))
            {
                flags |= IncrementalBodyDirtyFlags.Topology;
                topologyDirtyCount++;
            }
            else
            {
                AssignMotionVersion(ref current, previous);
                CurrentIncrementalProxies[proxyIndex] = current;
            }

            SetIncrementalDirtyFlags(current.BodyIndex, flags);
        }

        if (escapedBodyCount == 0)
        {
            incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - validationStart);
            return true;
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
        UpdatePersistentProxyMetadata();
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

    private bool MapDirtyIncidentNeighborPairsToCurrentBodies()
    {
        Pairs.Clear();
        if (EnableDiagnostics)
            PairDiagnostics.Clear();

        RebuildPersistentIncidentPairLookupIfNeededP1P6();
        if (!PersistentIncidentPairLookup.IsCreated ||
            !PersistentIncidentLookupEpoch.IsCreated ||
            PersistentIncidentLookupEpoch.Value !=
                IncrementalCacheState.Value.TopologyEpoch)
            return false;

        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            int dirtyBodyIndex = IncrementalDirtyBodies[dirtyIndex].BodyIndex;
            Entity entity = States[dirtyBodyIndex].Entity;
            NativeParallelMultiHashMapIterator<Entity> iterator;
            if (!PersistentIncidentPairLookup.TryGetFirstValue(
                    entity, out int persistentPairIndex, out iterator))
                continue;
            do
            {
                if ((uint)persistentPairIndex >= (uint)PersistentNeighborPairs.Length)
                    return false;
                StableEntityPairKey key =
                    PersistentNeighborPairs[persistentPairIndex].Key;
                if (!TryFindCurrentBodyIndex(key.EntityA, out int bodyA) ||
                    !TryFindCurrentBodyIndex(key.EntityB, out int bodyB))
                    return false;
                Pairs.Add(new UnitCollisionPair
                {
                    BodyA = math.min(bodyA, bodyB),
                    BodyB = math.max(bodyA, bodyB)
                });
            }
            while (PersistentIncidentPairLookup.TryGetNextValue(
                out persistentPairIndex, ref iterator));
        }

        SortAndDeduplicateBodyPairs(Pairs);
        return true;
    }

    private void ClassifyAndPatchDirtyIncidentContacts(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int scheduleStartSubstep)
    {
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

        int rawPairCount = Pairs.Length;
        int activeWriteIndex = 0;
        int retainedCount = 0;
        uint timestep = IncrementalCacheState.Value.Timestep;
        uint classificationEpoch = CalculateClassificationEpoch();
        statistics.CandidatePairCount += rawPairCount;

        for (int pairIndex = 0; pairIndex < rawPairCount; pairIndex++)
        {
            UnitCollisionPair rawPair = Pairs[pairIndex];
            FlowMovementFrameState bodyA = States[rawPair.BodyA];
            FlowMovementFrameState bodyB = States[rawPair.BodyB];
            StableEntityPairKey key = StableEntityPairKey.Create(
                bodyA.Entity,
                bodyB.Entity);
            TryFindPersistentProxy(
                key.EntityA,
                out PersistentSweptProxy proxyA);
            TryFindPersistentProxy(
                key.EntityB,
                out PersistentSweptProxy proxyB);
            PersistentPredictiveContact contact = ClassifyPersistentNeighborPair(
                key,
                rawPair,
                bodyA,
                bodyB,
                proxyA,
                proxyB,
                timestep,
                classificationEpoch,
                scheduleStartSubstep);
            PredictiveContactScratch.Add(contact);
            incrementalStatistics.ReclassifiedPairEvaluationCount++;
            incrementalStatistics.SweptClassificationEvaluationCount++;
            AccumulatePersistentClassificationStatistics(
                contact,
                ref statistics);

            if (contact.Lifecycle == PersistentContactLifecycle.Expired)
                continue;

            retainedCount++;
            if (contact.Lifecycle == PersistentContactLifecycle.Dormant)
            {
                PredictiveContactSchedule.Add(new PredictiveContactScheduleEntry
                {
                    Key = key,
                    Substep = contact.NextCheckSubstep
                });
                continue;
            }

            Pairs[activeWriteIndex++] = BuildUnitCollisionPairFromPersistentContact(
                rawPair.BodyA,
                rawPair.BodyB,
                contact);
        }

        Pairs.ResizeUninitialized(activeWriteIndex);
        if (Pairs.Length > 1)
            Pairs.AsArray().Sort(new UnitCollisionPairComparer());
        if (PredictiveContactScratch.Length > 1)
            PredictiveContactScratch.AsArray().Sort(
                new PersistentPredictiveContactComparer());
        if (PredictiveContactSchedule.Length > 1)
            PredictiveContactSchedule.AsArray().Sort(
                new PredictiveContactScheduleEntryComparer());
        PredictiveContactScheduleCursor.Value = 0;

        PersistentPredictiveContacts.Clear();
        PersistentPredictiveContacts.AddRange(PredictiveContactScratch.AsArray());
        RebuildPersistentContactViews();
        RebuildSoftAvoidancePairSetFromPersistentContacts();
        statistics.ContactPairCount += retainedCount;
        incrementalStatistics.CurrentInteractionPairCount =
            PersistentNeighborPairs.Length;
        incrementalStatistics.CurrentSoftAvoidancePairCount =
            SoftAvoidancePairs.Length;
        incrementalStatistics.PersistentViewRebuildCount++;

        IncrementalContactCacheState cacheState = IncrementalCacheState.Value;
        cacheState.ClassificationEpoch = classificationEpoch;
        IncrementalCacheState.Value = cacheState;
    }

    private void RebuildEscapedTimestepContactView(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        TimestepContactPairs.Clear();
        for (int previousIndex = 0;
             previousIndex < PreviousTimestepContactPairs.Length;
             previousIndex++)
        {
            UnitCollisionPair previous = PreviousTimestepContactPairs[previousIndex];
            if (IsDirtyBodyIndex(previous.BodyA) ||
                IsDirtyBodyIndex(previous.BodyB))
                continue;
            TimestepContactPairs.Add(previous);
        }

        for (int pairIndex = 0; pairIndex < Pairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = Pairs[pairIndex];
            int previousIndex = FindPairIndex(
                PreviousTimestepContactPairs,
                pair.BodyA,
                pair.BodyB);
            if (previousIndex >= 0)
            {
                UnitCollisionPair previous = PreviousTimestepContactPairs[previousIndex];
                pair.WasActivatedThisTimestep = previous.WasActivatedThisTimestep;
                pair.WasCorrectedThisTimestep = previous.WasCorrectedThisTimestep;
                pair.FirstActivatedSubstep = previous.FirstActivatedSubstep;
                pair.ActivatedSubstepCount = previous.ActivatedSubstepCount;
                pair.WasAddedByFallback = previous.WasAddedByFallback;
            }
            else
            {
                pair.WasAddedByFallback = 1;
                statistics.TimestepContactSetFallbackAddedPairCount++;
            }
            TimestepContactPairs.Add(pair);
        }
        SortAndDeduplicateBodyPairs(TimestepContactPairs);

        RefreshCurrentContactStateGauges(
            ref incrementalStatistics,
            TimestepContactPairs.Length);
        statistics.TimestepContactSetBuildCount++;
        statistics.TimestepContactSetClassificationPassCount++;
        statistics.TimestepContactSetUniquePairCount =
            TimestepContactPairs.Length;
        statistics.TimestepContactSetDormantPairCount =
            incrementalStatistics.CurrentDormantPairCount;
        ValidateSoftAvoidancePairViewAgainstQuadraticOracle(
            ref incrementalStatistics);
        ValidateIncrementalContactSetAgainstQuadraticOracle(
            ref incrementalStatistics);
    }

    private void RebuildSoftAvoidancePairSetFromPersistentContacts()
    {
        SoftAvoidancePairs.Clear();
        for (int contactIndex = 0;
             contactIndex < PersistentPredictiveContacts.Length;
             contactIndex++)
        {
            PersistentPredictiveContact contact =
                PersistentPredictiveContacts[contactIndex];
            if (contact.SoftAvoidanceCandidate == 0)
                continue;
            if (!TryFindCurrentBodyIndex(contact.Key.EntityA, out int bodyA) ||
                !TryFindCurrentBodyIndex(contact.Key.EntityB, out int bodyB))
                continue;
            SoftAvoidancePairs.Add(new UnitCollisionPair
            {
                BodyA = math.min(bodyA, bodyB),
                BodyB = math.max(bodyA, bodyB)
            });
        }
        if (SoftAvoidancePairs.Length > 1)
            SoftAvoidancePairs.AsArray().Sort(new UnitCollisionPairComparer());
    }

    private void RemoveDirtyPredictiveContactSchedules()
    {
        int writeIndex = 0;
        int readStart = math.clamp(
            PredictiveContactScheduleCursor.Value,
            0,
            PredictiveContactSchedule.Length);
        for (int scheduleIndex = readStart;
             scheduleIndex < PredictiveContactSchedule.Length;
             scheduleIndex++)
        {
            PredictiveContactScheduleEntry entry =
                PredictiveContactSchedule[scheduleIndex];
            if (IsDirtyEntity(entry.Key.EntityA) ||
                IsDirtyEntity(entry.Key.EntityB))
                continue;
            PredictiveContactSchedule[writeIndex++] = entry;
        }
        PredictiveContactSchedule.ResizeUninitialized(writeIndex);
        PredictiveContactScheduleCursor.Value = 0;
    }

    private bool IsDirtyBodyIndex(int bodyIndex)
    {
        return GetDirtyFlags(bodyIndex) != IncrementalBodyDirtyFlags.None;
    }

    private bool IsDirtyEntity(Entity entity)
    {
        if (!TryFindCurrentBodyIndex(entity, out int bodyIndex))
            return true;
        return IsDirtyBodyIndex(bodyIndex);
    }

    private void BuildCurrentIncrementalSweptProxies()
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
    }

    private void CalculateIncrementalTightSweptBounds(
        FlowMovementFrameState state,
        out float2 tightMin,
        out float2 tightMax)
    {
        float contactPadding = math.max(0f, PredictiveSkin) +
                               math.max(0f, TimestepContactMargin) * 2f;
        float avoidancePadding = math.max(0f, SoftAvoidanceShell) * 0.5f;
        float extent = math.max(0f, state.Radius) +
                       math.max(contactPadding, avoidancePadding);
        CalculateNeighborPathBounds(state, out float2 pathMin, out float2 pathMax);
        tightMin = pathMin - extent;
        tightMax = pathMax + extent;
    }

    private void CalculateIncrementalValidationBounds(
        FlowMovementFrameState state,
        out float2 validationMin,
        out float2 validationMax)
    {
        // The stored interaction envelope already includes the retained-contact
        // budget. Validation only needs the current contact/avoidance footprint;
        // using the retained padding again would make every unchanged proxy look
        // escaped at the envelope boundary.
        float contactPadding = math.max(0f, PredictiveSkin) +
                               math.max(0f, TimestepContactMargin);
        float avoidancePadding = math.max(0f, SoftAvoidanceShell) * 0.5f;
        float extent = math.max(0f, state.Radius) +
                       math.max(contactPadding, avoidancePadding);
        CalculateNeighborPathBounds(state, out float2 pathMin, out float2 pathMax);
        validationMin = pathMin - extent;
        validationMax = pathMax + extent;
    }

    private float2 CalculateAvoidanceHorizonEnd(FlowMovementFrameState state)
    {
        if (SoftAvoidanceVelocitySolver !=
                SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle ||
            SoftAvoidanceShell <= 0f || SoftAvoidanceResponseRate <= 0f)
            return state.TimestepPredictedPosition.xz;
        return state.TimestepStartPosition.xz +
               state.BasePredictedVelocity.xz * math.max(0f, RvoTimeHorizon);
    }

    private static void AssignMotionVersion(
        ref PersistentSweptProxy current,
        PersistentSweptProxy previous)
    {
        bool same = math.all(current.TrajectoryStart == previous.TrajectoryStart) &&
                    math.all(current.TrajectoryEnd == previous.TrajectoryEnd) &&
                    math.all(current.AvoidanceHorizonEnd ==
                             previous.AvoidanceHorizonEnd) &&
                    current.Radius == previous.Radius;
        current.MotionVersion = same
            ? previous.MotionVersion
            : previous.MotionVersion == uint.MaxValue
                ? 1u
                : previous.MotionVersion + 1u;
    }

    private void FullRebuildPersistentNeighborTopology(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long broadPhaseStart = ProfilerUnsafeUtility.Timestamp;
        SweptCellEntries.Clear();
        Pairs.Clear();
        PersistentSweptProxies.Clear();
        PersistentNeighborPairs.Clear();
        PersistentSweptProxies.AddRange(CurrentIncrementalProxies.AsArray());

        float cellSize = math.max(CellRadius * 2f, 0.0001f);
        int validProxyCount = 0;
        for (int proxyIndex = 0; proxyIndex < CurrentIncrementalProxies.Length; proxyIndex++)
        {
            PersistentSweptProxy proxy = CurrentIncrementalProxies[proxyIndex];
            if (proxy.IsValid == 0)
                continue;

            validProxyCount++;
            int2 minCell = (int2)math.floor((proxy.GuardMin - GridOrigin.xz) / cellSize);
            int2 maxCell = (int2)math.floor((proxy.GuardMax - GridOrigin.xz) / cellSize);
            if (maxCell.x < 0 || maxCell.y < 0 ||
                minCell.x >= GridDimensions.x || minCell.y >= GridDimensions.y)
                continue;

            minCell = math.clamp(minCell, int2.zero, GridDimensions - 1);
            maxCell = math.clamp(maxCell, int2.zero, GridDimensions - 1);
            for (int x = minCell.x; x <= maxCell.x; x++)
            {
                for (int y = minCell.y; y <= maxCell.y; y++)
                {
                    SweptCellEntries.Add(new SweptDiscCellEntry
                    {
                        CellIndex = FlowFieldUtils.GetFlatIndex(new int2(x, y), GridDimensions),
                        BodyIndex = proxy.BodyIndex
                    });
                }
            }
        }

        SweptCellEntries.AsArray().Sort(new SweptDiscCellEntryComparer());
        int cellStart = 0;
        while (cellStart < SweptCellEntries.Length)
        {
            int cellIndex = SweptCellEntries[cellStart].CellIndex;
            int cellEnd = cellStart + 1;
            while (cellEnd < SweptCellEntries.Length &&
                   SweptCellEntries[cellEnd].CellIndex == cellIndex)
                cellEnd++;

            for (int first = cellStart; first < cellEnd; first++)
            {
                int bodyA = SweptCellEntries[first].BodyIndex;
                for (int second = first + 1; second < cellEnd; second++)
                {
                    int bodyB = SweptCellEntries[second].BodyIndex;
                    if (bodyA == bodyB)
                        continue;
                    Pairs.Add(new UnitCollisionPair
                    {
                        BodyA = math.min(bodyA, bodyB),
                        BodyB = math.max(bodyA, bodyB)
                    });
                }
            }

            cellStart = cellEnd;
        }

        SortAndDeduplicateBodyPairs(Pairs);
        uint nextTopologyEpoch = IncrementalCacheState.Value.TopologyEpoch + 1u;
        for (int pairIndex = 0; pairIndex < Pairs.Length; pairIndex++)
        {
            UnitCollisionPair bodyPair = Pairs[pairIndex];
            FlowMovementFrameState stateA = States[bodyPair.BodyA];
            FlowMovementFrameState stateB = States[bodyPair.BodyB];
            if (!TryFindIncrementalProxy(stateA.Entity, out PersistentSweptProxy proxyA) ||
                !TryFindIncrementalProxy(stateB.Entity, out PersistentSweptProxy proxyB) ||
                proxyA.IsValid == 0 || proxyB.IsValid == 0 ||
                !AabbOverlaps(proxyA.GuardMin, proxyA.GuardMax, proxyB.GuardMin, proxyB.GuardMax))
                continue;

            PersistentNeighborPairs.Add(new PersistentNeighborPair
            {
                Key = StableEntityPairKey.Create(stateA.Entity, stateB.Entity),
                TopologyEpoch = nextTopologyEpoch,
                LastValidatedTimestep = IncrementalCacheState.Value.Timestep + 1u
            });
        }

        SortAndDeduplicatePersistentNeighborPairs();
        IncrementalContactCacheState state = IncrementalCacheState.Value;
        state.IsValid = 1;
        state.LastUpdateWasFullRebuild = 1;
        state.Timestep++;
        state.TopologyEpoch = nextTopologyEpoch;
        state.BodyCount = States.Length;
        state.NeighborPairCount = PersistentNeighborPairs.Length;
        state.GuardMargin = math.max(0f, GuardEnvelopeMargin);
        state.PredictiveSkin = math.max(0f, PredictiveSkin);
        state.TimestepContactMargin = math.max(0f, TimestepContactMargin);
        state.SoftAvoidanceShell = math.max(0f, SoftAvoidanceShell);
        state.SoftAvoidanceResponseRate = math.max(0f, SoftAvoidanceResponseRate);
        state.RvoTimeHorizon = math.max(0f, RvoTimeHorizon);
        state.SubstepCount = math.max(1, SubstepCount);
        state.PredictivePairGenerationEnabled =
            (byte)(EnablePredictivePairGeneration ? 1 : 0);
        state.PredictiveContactsEnabled =
            (byte)(EnablePredictiveContacts ? 1 : 0);
        state.SoftAvoidanceVelocitySolver =
            (byte)SoftAvoidanceVelocitySolver;
        IncrementalCacheState.Value = state;

        incrementalStatistics.Timestep = state.Timestep;
        incrementalStatistics.ProxyCount = validProxyCount;
        incrementalStatistics.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
        incrementalStatistics.NeighborPairAddedCount = PersistentNeighborPairs.Length;
        incrementalStatistics.LocalBroadPhaseNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - broadPhaseStart);
    }

    private bool RefreshTimestepInteractionPairs()
    {
        TimestepInteractionPairs.Clear();
        for (int pairIndex = 0; pairIndex < PersistentNeighborPairs.Length; pairIndex++)
        {
            StableEntityPairKey key = PersistentNeighborPairs[pairIndex].Key;
            if (!TryFindCurrentBodyIndex(key.EntityA, out int bodyA) ||
                !TryFindCurrentBodyIndex(key.EntityB, out int bodyB))
                return false;

            TimestepInteractionPairs.Add(new UnitCollisionPair
            {
                BodyA = math.min(bodyA, bodyB),
                BodyB = math.max(bodyA, bodyB)
            });
        }
        SortAndDeduplicateBodyPairs(TimestepInteractionPairs);
        return true;
    }

    private bool MapPersistentNeighborPairsToCurrentBodies()
    {
        if (!RefreshTimestepInteractionPairs())
            return false;

        Pairs.Clear();
        if (EnableDiagnostics)
            PairDiagnostics.Clear();
        Pairs.AddRange(TimestepInteractionPairs.AsArray());
        return true;
    }

    private bool TryFindPersistentProxy(Entity entity, out PersistentSweptProxy proxy)
    {
        int low = 0;
        int high = PersistentSweptProxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            PersistentSweptProxy candidate = PersistentSweptProxies[middle];
            int comparison = StableEntityPairKey.CompareEntity(candidate.Entity, entity);
            if (comparison == 0)
            {
                proxy = candidate;
                return true;
            }
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }

        proxy = default;
        return false;
    }

    private bool TryFindIncrementalProxy(Entity entity, out PersistentSweptProxy proxy)
    {
        int low = 0;
        int high = CurrentIncrementalProxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            PersistentSweptProxy candidate = CurrentIncrementalProxies[middle];
            int comparison = StableEntityPairKey.CompareEntity(candidate.Entity, entity);
            if (comparison == 0)
            {
                proxy = candidate;
                return true;
            }
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }

        proxy = default;
        return false;
    }

    private void SortAndDeduplicatePersistentNeighborPairs()
    {
        SortAndDeduplicatePersistentNeighborPairs(PersistentNeighborPairs);
    }

    private static void SortAndDeduplicatePersistentNeighborPairs(
        NativeList<PersistentNeighborPair> pairs)
    {
        if (pairs.Length <= 1)
            return;

        pairs.AsArray().Sort(new PersistentNeighborPairComparer());
        int writeIndex = 1;
        PersistentNeighborPair previous = pairs[0];
        for (int readIndex = 1; readIndex < pairs.Length; readIndex++)
        {
            PersistentNeighborPair current = pairs[readIndex];
            if (current.Key.Equals(previous.Key))
                continue;
            pairs[writeIndex++] = current;
            previous = current;
        }
        pairs.ResizeUninitialized(writeIndex);
    }
}
}
