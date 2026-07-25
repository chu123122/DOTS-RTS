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
        bool cacheCanBePatched = !forceFullRebuild && IsPersistentCacheStructurallyReusableP1P6();
        SummarizePreparedIncrementalDirtyBodiesP1P6(
            ref incrementalStatistics,
            out int topologyDirtyCount,
            out bool entitySetDirty);
        incrementalStatistics.ProxyValidationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        float dirtyRatio = Bodies.Length > 0 ? (float)topologyDirtyCount / Bodies.Length : 1f;
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
            Pairs.Add(BuildContactConstraintFromPersistentContact(
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
            SoftAvoidancePairs.Add(new BodyPair
            {
                BodyA = math.min(bodyA, bodyB),
                BodyB = math.max(bodyA, bodyB)
            });
        }
        PredictiveContactSchedule.AddRange(
            PersistentDormantContactSchedule.AsArray());
        PredictiveContactScheduleCursor.Value = 0;
        if (Pairs.Length > 1)
            Pairs.AsArray().Sort(new ContactConstraintComparer());
        if (SoftAvoidancePairs.Length > 1)
            SoftAvoidancePairs.AsArray().Sort(new BodyPairComparer());

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
            BodyPair rawPair = TimestepInteractionPairs[pairIndex];
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
                    bodyASnapshot,
                    bodyAEvidence,
                    bodyBSnapshot,
                    bodyBEvidence,
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
                    Key = key,
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
        ContactConstraint rawPair,
        CrowdBodySnapshot bodyASnapshot,
        CrowdMotionEvidence bodyAEvidence,
        CrowdBodySnapshot bodyBSnapshot,
        CrowdMotionEvidence bodyBEvidence,
        PersistentSweptProxy proxyA,
        PersistentSweptProxy proxyB,
        uint timestep,
        uint classificationEpoch,
        int scheduleStartSubstep)
    {
        float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;
        float3 relativeStart =
            bodyBEvidence.TrajectoryStart - bodyAEvidence.TrajectoryStart;
        float3 relativeDisplacement =
            (bodyBEvidence.BaselineEnd - bodyBEvidence.TrajectoryStart) -
            (bodyAEvidence.BaselineEnd - bodyAEvidence.TrajectoryStart);
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
            bodyBEvidence.BaselineEnd - bodyAEvidence.BaselineEnd;
        endDelta.y = 0f;
        float endDistanceSq = math.lengthsq(endDelta);
        float radiusSumSq = radiusSum * radiusSum;

        PersistentContactLifecycle lifecycle;
        ContactConstraintMode contactMode = ContactConstraintMode.Regular;
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
                ? ContactConstraintMode.Predictive
                : ContactConstraintMode.Regular;
        }

        // Keep A1 classification exactly equivalent to A0. Previous-frame
        // normals are not a correctness input and are therefore not blended.
        float3 stableNormal = bodyAEvidence.TrajectoryStart -
                              bodyBEvidence.TrajectoryStart;
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
            FixedSide = contactMode == ContactConstraintMode.Predictive
                ? (sbyte)1
                : (sbyte)0,
            SoftAvoidanceCandidate = (byte)(CouldEnterSoftAvoidanceRange(
                rawPair.BodyA,
                rawPair.BodyB) ? 1 : 0),
            FirstPossibleSubstep = firstPossibleSubstep,
            NextCheckSubstep = firstPossibleSubstep,
            ClosestTime = closestTime,
            LastSeenTimestep = timestep,
            MotionVersionA = proxyA.MotionVersion,
            MotionVersionB = proxyB.MotionVersion,
            ClassificationEpoch = classificationEpoch
        };
    }

    private static ContactConstraint BuildContactConstraintFromPersistentContact(
        int firstBodyIndex,
        int secondBodyIndex,
        PersistentPredictiveContact contact)
    {
        return new ContactConstraint
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
            ContactConstraint pair = Pairs[pairIndex];
            CrowdBodySnapshot bodyASnapshot = Bodies[pair.BodyA];
            CrowdNavigationState bodyANavigation = NavigationStates[pair.BodyA];
            CrowdMotionIntent bodyAIntent = MotionIntents[pair.BodyA];
            CrowdMotionEvidence bodyAEvidence = MotionEvidence[pair.BodyA];
            CrowdBodyStepState bodyAStep = StepStates[pair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = Bodies[pair.BodyB];
            CrowdNavigationState bodyBNavigation = NavigationStates[pair.BodyB];
            CrowdMotionIntent bodyBIntent = MotionIntents[pair.BodyB];
            CrowdMotionEvidence bodyBEvidence = MotionEvidence[pair.BodyB];
            CrowdBodyStepState bodyBStep = StepStates[pair.BodyB];
            StableEntityPairKey key = StableEntityPairKey.Create(bodyASnapshot.Entity, bodyBSnapshot.Entity);

            PersistentContactLifecycle lifecycle;
            float3 currentDelta = bodyAEvidence.TrajectoryStart - bodyBEvidence.TrajectoryStart;
            currentDelta.y = 0f;
            float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;
            if (math.lengthsq(currentDelta) <= radiusSum * radiusSum)
                lifecycle = PersistentContactLifecycle.Actual;
            else if (pair.IsDormant != 0)
                lifecycle = PersistentContactLifecycle.Dormant;
            else if (pair.ContactMode == ContactConstraintMode.Predictive)
                lifecycle = PersistentContactLifecycle.Predictive;
            else
                lifecycle = PersistentContactLifecycle.Approaching;

            // 调度与稳定法线属于中层 InteractionSet 的派生结果。
            // 不读取上一帧接触状态，保证 A0B1 与 A1B1 只有来源成本不同。
            float3 stableNormal = pair.PredictiveNormal;
            sbyte fixedSide = pair.ContactMode == ContactConstraintMode.Predictive
                ? (sbyte)1
                : (sbyte)0;

            PersistentSweptProxy proxyA = default;
            PersistentSweptProxy proxyB = default;
            if (EnablePersistentContactCache)
            {
                TryFindPersistentProxy(bodyASnapshot.Entity, out proxyA);
                TryFindPersistentProxy(bodyBSnapshot.Entity, out proxyB);
            }

            ushort firstPossibleSubstep = 0;
            if (lifecycle == PersistentContactLifecycle.Dormant)
            {
                if (!HasRelativeTimestepTrajectory(bodyAEvidence, bodyBEvidence))
                {
                    firstPossibleSubstep = ushort.MaxValue;
                }
                else
                {
                    float closestTime = CalculatePairClosestTime(bodyAEvidence, bodyBEvidence);
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
            ContactConstraint pair = Pairs[pairIndex];
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
        CrowdMotionEvidence bodyAEvidence,
        CrowdMotionEvidence bodyBEvidence)
    {
        float3 relativeStart =
            bodyBEvidence.TrajectoryStart - bodyAEvidence.TrajectoryStart;
        float3 relativeDisplacement =
            (bodyBEvidence.BaselineEnd - bodyBEvidence.TrajectoryStart) -
            (bodyAEvidence.BaselineEnd - bodyAEvidence.TrajectoryStart);
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
        CrowdMotionEvidence bodyAEvidence,
        CrowdMotionEvidence bodyBEvidence)
    {
        float3 relativeDisplacement =
            (bodyBEvidence.BaselineEnd - bodyBEvidence.TrajectoryStart) -
            (bodyAEvidence.BaselineEnd - bodyAEvidence.TrajectoryStart);
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

            if (TryBuildCurrentScheduledPair(bodyA, bodyB, out ContactConstraint pair))
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
            TimestepContactPairs.AsArray().Sort(new ContactConstraintComparer());
        UpdateActiveConstraintGauges(
            ref incrementalStatistics,
            TimestepContactPairs.Length);
        incrementalStatistics.ContactActivationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - activationStart);
    }

    private void UpdatePersistentContactAfterScheduledCheck(
        StableEntityPairKey key,
        ContactConstraint pair,
        ushort nextCheckSubstep)
    {
        int contactIndex = FindPersistentPredictiveContactIndex(key);
        if (contactIndex < 0)
            return;

        PersistentPredictiveContact contact =
            PersistentPredictiveContacts[contactIndex];
        CrowdBodySnapshot bodyASnapshot = Bodies[pair.BodyA];
        CrowdNavigationState bodyANavigation = NavigationStates[pair.BodyA];
        CrowdMotionIntent bodyAIntent = MotionIntents[pair.BodyA];
        CrowdMotionEvidence bodyAEvidence = MotionEvidence[pair.BodyA];
        CrowdBodyStepState bodyAStep = StepStates[pair.BodyA];
        CrowdBodySnapshot bodyBSnapshot = Bodies[pair.BodyB];
        CrowdNavigationState bodyBNavigation = NavigationStates[pair.BodyB];
        CrowdMotionIntent bodyBIntent = MotionIntents[pair.BodyB];
        CrowdMotionEvidence bodyBEvidence = MotionEvidence[pair.BodyB];
        CrowdBodyStepState bodyBStep = StepStates[pair.BodyB];
        float3 delta = bodyAStep.SolvedPosition - bodyBStep.SolvedPosition;
        delta.y = 0f;
        float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;
        contact.Lifecycle = math.lengthsq(delta) <= radiusSum * radiusSum
            ? PersistentContactLifecycle.Actual
            : pair.ContactMode == ContactConstraintMode.Predictive
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
        out ContactConstraint pair)
    {
        int bodyAIndex = math.min(firstBodyIndex, secondBodyIndex);
        int bodyBIndex = math.max(firstBodyIndex, secondBodyIndex);
        CrowdBodySnapshot bodyASnapshot = Bodies[bodyAIndex];
        CrowdNavigationState bodyANavigation = NavigationStates[bodyAIndex];
        CrowdMotionIntent bodyAIntent = MotionIntents[bodyAIndex];
        CrowdMotionEvidence bodyAEvidence = MotionEvidence[bodyAIndex];
        CrowdBodyStepState bodyAStep = StepStates[bodyAIndex];
        CrowdBodySnapshot bodyBSnapshot = Bodies[bodyBIndex];
        CrowdNavigationState bodyBNavigation = NavigationStates[bodyBIndex];
        CrowdMotionIntent bodyBIntent = MotionIntents[bodyBIndex];
        CrowdMotionEvidence bodyBEvidence = MotionEvidence[bodyBIndex];
        CrowdBodyStepState bodyBStep = StepStates[bodyBIndex];
        float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;
        float candidateDistance = radiusSum + math.max(0f, PredictiveSkin);

        float3 relativeStart = bodyBStep.SolvedPosition - bodyAStep.SolvedPosition;
        float3 relativeDisplacement =
            (bodyBEvidence.BaselineEnd - bodyBStep.SolvedPosition) -
            (bodyAEvidence.BaselineEnd - bodyAStep.SolvedPosition);
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
            bodyBEvidence.BaselineEnd - bodyAEvidence.BaselineEnd;
        endDelta.y = 0f;
        float endDistanceSq = math.lengthsq(endDelta);
        float radiusSumSq = radiusSum * radiusSum;
        bool isActual = startDistanceSq <= radiusSumSq;
        bool preventSideExchange =
            !isActual &&
            endDistanceSq >= radiusSumSq &&
            minDistanceSq <= radiusSumSq;

        pair = new ContactConstraint
        {
            BodyA = bodyAIndex,
            BodyB = bodyBIndex,
            ContactMode = preventSideExchange && EnablePredictiveContacts
                ? ContactConstraintMode.Predictive
                : ContactConstraintMode.Regular,
            PredictiveNormal = math.normalizesafe(
                bodyAStep.SolvedPosition - bodyBStep.SolvedPosition,
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

    private bool IsPersistentCacheStructurallyReusableP1P6()
    {
        IncrementalContactCacheState state = IncrementalCacheState.Value;
        return state.IsValid != 0 &&
               state.BodyCount == Bodies.Length &&
               PersistentSweptProxies.Length == Bodies.Length &&
               PersistentProxyIndexByBody.Length == Bodies.Length &&
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
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        float guardMargin,
        float softAvoidanceShell,
        float softAvoidanceResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon)
    {
        PersistentSweptProxy proxy = new PersistentSweptProxy
        {
            Entity = stateSnapshot.Entity,
            BodyIndex = bodyIndex,
            IsValid = (byte)((stateSnapshot.IsInsideSimulationDomain != 0) ? 1 : 0),
            Radius = math.max(0f, stateSnapshot.Radius)
        };
        if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            return proxy;
        proxy.TightMin = stateEvidence.InteractionEnvelopeMin;
        proxy.TightMax = stateEvidence.InteractionEnvelopeMax;
        proxy.GuardMin = proxy.TightMin - math.max(0f, guardMargin);
        proxy.GuardMax = proxy.TightMax + math.max(0f, guardMargin);
        proxy.TrajectoryStart = stateEvidence.TrajectoryStart.xz;
        proxy.TrajectoryEnd = stateEvidence.BaselineEnd.xz;
        proxy.AvoidanceHorizonEnd =
            softSolverMode == SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle &&
            softAvoidanceShell > 0f && softAvoidanceResponseRate > 0f
                ? stateEvidence.TrajectoryStart.xz +
                  stateStep.BaseVelocity.xz * math.max(0f, rvoTimeHorizon)
                : stateEvidence.BaselineEnd.xz;
        proxy.MotionVersion = 1u;
        return proxy;
    }

    private static IncrementalBodyDirtyFlags ClassifyAndUpdatePersistentProxyForBodyP1P6(
        int bodyIndex,
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
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
        if (previous.Entity != stateSnapshot.Entity)
            return IncrementalBodyDirtyFlags.EntitySet |
                   IncrementalBodyDirtyFlags.Topology |
                   IncrementalBodyDirtyFlags.Motion;

        PersistentSweptProxy current = BuildPersistentProxyFromStateP1P6(
            bodyIndex, stateSnapshot, stateEvidence, stateStep,
            guardMargin, softAvoidanceShell,
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
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            IncrementalBodyDirtyFlags flags = ClassifyAndUpdatePersistentProxyForBodyP1P6(
                bodyIndex, Bodies[bodyIndex], MotionEvidence[bodyIndex],
                StepStates[bodyIndex], PersistentSweptProxies.AsArray(),
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
            if ((uint)bodyIndex >= (uint)Bodies.Length)
                return false;
            IncrementalBodyDirtyFlags refreshed = ClassifyAndUpdatePersistentProxyForBodyP1P6(
                bodyIndex, Bodies[bodyIndex], MotionEvidence[bodyIndex],
                StepStates[bodyIndex], PersistentSweptProxies.AsArray(),
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
        state.BodyCount = Bodies.Length;
        state.NeighborPairCount = PersistentNeighborPairs.Length;
        IncrementalCacheState.Value = state;
        incrementalStatistics.Timestep = state.Timestep;
        incrementalStatistics.NeighborPairRetainedCount = PersistentNeighborPairs.Length;
    }

    private void RebuildPersistentProxyIndexByBodyP1P6()
    {
        PersistentProxyIndexByBody.ResizeUninitialized(Bodies.Length);
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
            PersistentProxyIndexByBody[bodyIndex] = -1;
        for (int proxyIndex = 0; proxyIndex < PersistentSweptProxies.Length; proxyIndex++)
        {
            int bodyIndex = PersistentSweptProxies[proxyIndex].BodyIndex;
            if ((uint)bodyIndex < (uint)PersistentProxyIndexByBody.Length)
                PersistentProxyIndexByBody[bodyIndex] = proxyIndex;
        }
    }
    private void ClearIncrementalDirtyBodySet()
    {
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            int bodyIndex = IncrementalDirtyBodies[dirtyIndex].BodyIndex;
            if ((uint)bodyIndex < (uint)IncrementalDirtyFlagsByBody.Length)
                IncrementalDirtyFlagsByBody[bodyIndex] = 0;
        }
        IncrementalDirtyBodies.Clear();
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

            CrowdBodySnapshot dirtyStateSnapshot = Bodies[dirty.BodyIndex];
            CrowdNavigationState dirtyStateNavigation = NavigationStates[dirty.BodyIndex];
            CrowdMotionIntent dirtyStateIntent = MotionIntents[dirty.BodyIndex];
            CrowdMotionEvidence dirtyStateEvidence = MotionEvidence[dirty.BodyIndex];
            CrowdBodyStepState dirtyStateStep = StepStates[dirty.BodyIndex];
            if (!TryFindPersistentProxy(
                    dirtyStateSnapshot.Entity,
                    out PersistentSweptProxy dirtyProxy) ||
                dirtyProxy.IsValid == 0)
                continue;

            int dirtyProxyIndex = FindPersistentProxyIndex(dirtyStateSnapshot.Entity);
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
        state.BodyCount = Bodies.Length;
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
        float dirtyRatio = Bodies.Length > 0 ? (float)escapedBodyCount / Bodies.Length : 1f;
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
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            stateEvidence.EnvelopeEscaped = 0;
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
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
            Entity entity = Bodies[dirtyBodyIndex].Entity;
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
                Pairs.Add(new ContactConstraint
                {
                    BodyA = math.min(bodyA, bodyB),
                    BodyB = math.max(bodyA, bodyB)
                });
            }
            while (PersistentIncidentPairLookup.TryGetNextValue(
                out persistentPairIndex, ref iterator));
        }

        SortAndDeduplicateConstraints(Pairs);
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
            ContactConstraint rawPair = Pairs[pairIndex];
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
            TryFindPersistentProxy(
                key.EntityA,
                out PersistentSweptProxy proxyA);
            TryFindPersistentProxy(
                key.EntityB,
                out PersistentSweptProxy proxyB);
            PersistentPredictiveContact contact = ClassifyPersistentNeighborPair(
                key,
                rawPair,
                bodyASnapshot,
                bodyAEvidence,
                bodyBSnapshot,
                bodyBEvidence,
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

            Pairs[activeWriteIndex++] = BuildContactConstraintFromPersistentContact(
                rawPair.BodyA,
                rawPair.BodyB,
                contact);
        }

        Pairs.ResizeUninitialized(activeWriteIndex);
        if (Pairs.Length > 1)
            Pairs.AsArray().Sort(new ContactConstraintComparer());
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
            ContactConstraint previous = PreviousTimestepContactPairs[previousIndex];
            if (IsDirtyBodyIndex(previous.BodyA) ||
                IsDirtyBodyIndex(previous.BodyB))
                continue;
            TimestepContactPairs.Add(previous);
        }

        for (int pairIndex = 0; pairIndex < Pairs.Length; pairIndex++)
        {
            ContactConstraint pair = Pairs[pairIndex];
            int previousIndex = FindPairIndex(
                PreviousTimestepContactPairs,
                pair.BodyA,
                pair.BodyB);
            if (previousIndex >= 0)
            {
                ContactConstraint previous = PreviousTimestepContactPairs[previousIndex];
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
        SortAndDeduplicateConstraints(TimestepContactPairs);

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
            SoftAvoidancePairs.Add(new BodyPair
            {
                BodyA = math.min(bodyA, bodyB),
                BodyB = math.max(bodyA, bodyB)
            });
        }
        if (SoftAvoidancePairs.Length > 1)
            SoftAvoidancePairs.AsArray().Sort(new BodyPairComparer());
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

        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CurrentIncrementalProxies.Add(BuildPersistentProxyFromStateP1P6(
                bodyIndex, Bodies[bodyIndex], MotionEvidence[bodyIndex],
                StepStates[bodyIndex], guardMargin, SoftAvoidanceShell,
                SoftAvoidanceResponseRate, SoftAvoidanceVelocitySolver,
                RvoTimeHorizon));
        }

        CurrentIncrementalProxies.AsArray().Sort(new PersistentSweptProxyComparer());
    }

    private void CalculateIncrementalTightSweptBounds(
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        out float2 tightMin,
        out float2 tightMax)
    {
        float contactPadding = math.max(0f, PredictiveSkin) +
                               math.max(0f, TimestepContactMargin) * 2f;
        float avoidancePadding = math.max(0f, SoftAvoidanceShell) * 0.5f;
        float extent = math.max(0f, stateSnapshot.Radius) +
                       math.max(contactPadding, avoidancePadding);
        CalculateNeighborPathBounds(stateEvidence, stateStep, out float2 pathMin, out float2 pathMax);
        tightMin = pathMin - extent;
        tightMax = pathMax + extent;
    }

    private void CalculateIncrementalValidationBounds(
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
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
        float extent = math.max(0f, stateSnapshot.Radius) +
                       math.max(contactPadding, avoidancePadding);
        CalculateNeighborPathBounds(stateEvidence, stateStep, out float2 pathMin, out float2 pathMax);
        validationMin = pathMin - extent;
        validationMax = pathMax + extent;
    }

    private float2 CalculateAvoidanceHorizonEnd(
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep)
    {
        if (SoftAvoidanceVelocitySolver !=
                SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle ||
            SoftAvoidanceShell <= 0f || SoftAvoidanceResponseRate <= 0f)
            return stateEvidence.BaselineEnd.xz;
        return stateEvidence.TrajectoryStart.xz +
               stateStep.BaseVelocity.xz * math.max(0f, RvoTimeHorizon);
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
        RebuildPersistentProxyIndexByBodyP1P6();

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
                    Pairs.Add(new ContactConstraint
                    {
                        BodyA = math.min(bodyA, bodyB),
                        BodyB = math.max(bodyA, bodyB)
                    });
                }
            }

            cellStart = cellEnd;
        }

        SortAndDeduplicateConstraints(Pairs);
        uint nextTopologyEpoch = IncrementalCacheState.Value.TopologyEpoch + 1u;
        for (int pairIndex = 0; pairIndex < Pairs.Length; pairIndex++)
        {
            ContactConstraint bodyPair = Pairs[pairIndex];
            CrowdBodySnapshot stateASnapshot = Bodies[bodyPair.BodyA];
            CrowdNavigationState stateANavigation = NavigationStates[bodyPair.BodyA];
            CrowdMotionIntent stateAIntent = MotionIntents[bodyPair.BodyA];
            CrowdMotionEvidence stateAEvidence = MotionEvidence[bodyPair.BodyA];
            CrowdBodyStepState stateAStep = StepStates[bodyPair.BodyA];
            CrowdBodySnapshot stateBSnapshot = Bodies[bodyPair.BodyB];
            CrowdNavigationState stateBNavigation = NavigationStates[bodyPair.BodyB];
            CrowdMotionIntent stateBIntent = MotionIntents[bodyPair.BodyB];
            CrowdMotionEvidence stateBEvidence = MotionEvidence[bodyPair.BodyB];
            CrowdBodyStepState stateBStep = StepStates[bodyPair.BodyB];
            if (!TryFindIncrementalProxy(stateASnapshot.Entity, out PersistentSweptProxy proxyA) ||
                !TryFindIncrementalProxy(stateBSnapshot.Entity, out PersistentSweptProxy proxyB) ||
                proxyA.IsValid == 0 || proxyB.IsValid == 0 ||
                !AabbOverlaps(proxyA.GuardMin, proxyA.GuardMax, proxyB.GuardMin, proxyB.GuardMax))
                continue;

            PersistentNeighborPairs.Add(new PersistentNeighborPair
            {
                Key = StableEntityPairKey.Create(stateASnapshot.Entity, stateBSnapshot.Entity),
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
        state.BodyCount = Bodies.Length;
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

            TimestepInteractionPairs.Add(new BodyPair
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
        AppendBodyPairsAsConstraints(TimestepInteractionPairs.AsArray(), Pairs);
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
