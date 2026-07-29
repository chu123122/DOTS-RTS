using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct InteractionCertificationJob
{
    // Threshold above which a full broad-phase rebuild is cheaper than per-body
    // incremental repair. Kept high on purpose: a dense local collision cluster
    // (e.g. two formations colliding head-on) can dirty many bodies in one region
    // while the rest of the crowd stays clean. Repair is spatial-hash-scoped to
    // the dirty list, so even a 50-60% dirty ratio of locally-clustered bodies
    // is still cheaper than a global O(N) rebuild. Only treat a near-total dirty
    // set (>70%) as a genuinely global change worth a full rebuild.
    private const float IncrementalDirtyBodyRatioThreshold = 0.7f;

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
        bool cacheCanBePatched = !forceFullRebuild && IsPersistentCacheStructurallyReusable();
        SummarizePreparedIncrementalDirtyBodies(
            ref incrementalStatistics,
            out int topologyDirtyCount,
            out bool entitySetDirty);
        incrementalStatistics.ProxyValidationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
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
            RebuildPersistentSpatialMembership(IncrementalCacheState.Value.TopologyEpoch);
            long elapsed = ContactPipelineMath.TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - buildStart);
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
                AdvancePersistentCacheTimestep(ref incrementalStatistics);
            }
            long elapsed = ContactPipelineMath.TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - repairStart);
            long localElapsed = incrementalStatistics.LocalBroadPhaseNanoseconds - localBefore;
            long exclusive = elapsed - localElapsed;
            incrementalStatistics.PairDiffNanoseconds += exclusive > 0L ? exclusive : 0L;
            incrementalStatistics.UsedIncrementalTopology = 1;
        }

        long classificationStart = ProfilerUnsafeUtility.Timestamp;
        if (TryReusePersistentContactViews(ref statistics, ref incrementalStatistics))
        {
            incrementalStatistics.SweptClassificationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - classificationStart);
            incrementalStatistics.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
            persistentViewReady = true;
            return true;
        }
        if (!useFullRebuild &&
            TryPatchDirtyIncidentPersistentContactViews(
                ref statistics,
                ref incrementalStatistics,
                scheduleStartSubstep))
        {
            incrementalStatistics.SweptClassificationNanoseconds +=
                ContactPipelineMath.TimestampToNanoseconds(
                    ProfilerUnsafeUtility.Timestamp -
                    classificationStart);
            incrementalStatistics.PersistentNeighborPairCount =
                PersistentNeighborPairs.Length;
            persistentViewReady = true;
            return true;
        }

        long mappingStart = ProfilerUnsafeUtility.Timestamp;
        bool mapped = MapPersistentNeighborPairsToCurrentBodies();
        incrementalStatistics.PersistentPairMappingNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - mappingStart);
        if (!mapped)
        {
            IncrementalContactCacheState invalidState = IncrementalCacheState.Value;
            invalidState.IsValid = 0;
            IncrementalCacheState.Value = invalidState;
            TimestepInteractionPairs.Clear();
            long fullSweepStart = ProfilerUnsafeUtility.Timestamp;
            BuildSweptInteractionPairs(ref statistics);
            incrementalStatistics.FullSweepSourceNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - fullSweepStart);
            incrementalStatistics.UsedFullRebuild = 1;
            return false;
        }

        classificationStart = ProfilerUnsafeUtility.Timestamp;
        ClassifyOrReusePersistentNeighborPairs(
            ref statistics,
            ref incrementalStatistics,
            scheduleStartSubstep);
        incrementalStatistics.SweptClassificationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - classificationStart);
        incrementalStatistics.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
        persistentViewReady = true;
        return true;
    }

    private bool TryPatchDirtyIncidentPersistentContactViews(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int scheduleStartSubstep)
    {
        IncrementalContactCacheState cacheState =
            IncrementalCacheState.Value;
        if (IncrementalDirtyBodies.Length == 0 ||
            cacheState.ContactViewsValid == 0 ||
            cacheState.ClassificationEpoch !=
                CalculateClassificationEpoch())
            return false;

        RebuildPersistentIncidentPairLookupIfNeededParallel();
        long mappingStart = ProfilerUnsafeUtility.Timestamp;
        bool mapped = MapDirtyIncidentNeighborPairsToCurrentBodies(
            out int _,
            out int eligibilitySkipped);
        incrementalStatistics.PersistentPairMappingNanoseconds +=
            ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - mappingStart);
        if (!mapped)
            return false;

        // eligibility filter 跳过的对是真省下的分类工作，计入 Skipped（仅这些）。
        incrementalStatistics.ClassificationSkippedCount += eligibilitySkipped;
        int incidentPairCount = Pairs.Length;
        PredictiveDiscContactStatistics statisticsBeforePatch =
            statistics;
        ClassifyAndPatchDirtyIncidentContacts(
            ref statistics,
            ref incrementalStatistics,
            scheduleStartSubstep);
        statistics = statisticsBeforePatch;

        if (!TryBuildCurrentContactViewsFromPersistentState())
            return false;

        int skippedCount = math.max(
            0,
            PersistentNeighborPairs.Length - incidentPairCount);
        RestorePersistentContactViewStatistics(
            ref statistics,
            ref incrementalStatistics,
            skippedCount,
            true);
        return true;
    }

    private bool TryReusePersistentContactViews(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        IncrementalContactCacheState cacheState = IncrementalCacheState.Value;
        uint classificationEpoch = CalculateClassificationEpoch();
        // The topology can remain reusable while endpoint motion changes a
        // contact's Actual/Approaching/Predictive/Dormant classification. The
        // whole-view fast path bypasses the per-pair MotionVersion checks below,
        // so it is valid only when no endpoint changed this timestep.
        if (IncrementalDirtyBodies.Length != 0 ||
            cacheState.ContactViewsValid == 0 ||
            cacheState.ClassificationEpoch != classificationEpoch)
            return false;

        if (!TryBuildCurrentContactViewsFromPersistentState())
            return false;

        RestorePersistentContactViewStatistics(
            ref statistics,
            ref incrementalStatistics,
            PersistentNeighborPairs.Length,
            true);
        incrementalStatistics.PersistentViewReuseCount++;
        return true;
    }

    private bool TryBuildCurrentContactViewsFromPersistentState()
    {
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
        return true;
    }

    private void RestorePersistentContactViewStatistics(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int classificationSkippedCount,
        bool countAsClassificationReuse)
    {
        IncrementalContactCacheState cacheState = IncrementalCacheState.Value;
        int sweptCount = cacheState.DormantContactCount +
                         cacheState.ApproachingContactCount +
                         cacheState.PredictiveContactCount +
                         cacheState.ActualContactCount;
        statistics.CandidatePairCount += PersistentNeighborPairs.Length;
        statistics.ContactPairCount += sweptCount;
        // Reuse skips per-contact classification, so restore the same
        // solver-facing lifecycle counters that classification would emit.
        statistics.ActualGeneratedPairCount += cacheState.ActualContactCount;
        statistics.PredictiveGeneratedPairCount +=
            cacheState.ApproachingContactCount +
            cacheState.PredictiveContactCount;
        statistics.PotentialPredictivePairCount +=
            cacheState.PredictiveContactCount;
        statistics.PredictivePairCount += cacheState.PredictiveContactCount;
        statistics.TimestepContactSetDormantPairCount +=
            cacheState.DormantContactCount;
        if (countAsClassificationReuse)
            incrementalStatistics.ClassificationReuseCount +=
                classificationSkippedCount;
        incrementalStatistics.ClassificationSkippedCount +=
            classificationSkippedCount;
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
        UpdateActiveConstraintGauges(
            ref incrementalStatistics,
            Pairs.Length);
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
        RebuildPersistentContactIndex();
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
        BodyPair rawPair,
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
            ContactPipelineMath.DeterministicFallbackNormal(rawPair.BodyA, rawPair.BodyB));

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
        PersistentPredictiveContact contact) =>
        PersistentContactMath.BuildConstraintFromPersistentContact(
            firstBodyIndex, secondBodyIndex, contact);

    private static void AccumulatePersistentClassificationStatistics(
        PersistentPredictiveContact contact,
        ref PredictiveDiscContactStatistics statistics) =>
        PersistentContactMath.AccumulateClassificationStatistics(
            contact, ref statistics);

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
        if (PersistentContactIndex.IsCreated &&
            PersistentContactIndex.TryGetValue(key, out contact))
            return true;
        contact = default;
        return false;
    }

    private void BuildTimestepPredictiveSchedule(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int scheduleStartSubstep)
    {
        PrepareCurrentBodyLookup();
        // 调度器向 scratch/schedule 追加条目而不清零；
        // fallback/per-substep 路径下 scratch 仍保有上次持久化分类的全量数据，
        // 必须在此处先清除，否则产生 O(N_total) 重复条目并导致 hashmap 重建爆涨。
        PredictiveContactScratch.Clear();
        PredictiveContactSchedule.Clear();
        PredictiveContactScheduler.BuildTimestepSchedule(
            Pairs,
            Bodies,
            MotionEvidence,
            PersistentSweptProxies,
            PredictiveContactScratch,
            PersistentPredictiveContacts,
            PredictiveContactSchedule,
            PredictiveContactScheduleCursor,
            IncrementalCacheState.Value.Timestep,
            SubstepCount,
            scheduleStartSubstep,
            EnableTimestepContactSetCache,
            EnablePersistentContactCache,
            EnablePredictiveContacts,
            ref incrementalStatistics);
        // 调度器覆写 PersistentPredictiveContacts，仅含本次 Pairs（O(active_pairs)）；同步重建索引
        RebuildPersistentContactIndex();
    }

    private static float CalculatePairClosestTime(
        CrowdMotionEvidence bodyAEvidence,
        CrowdMotionEvidence bodyBEvidence) =>
        PersistentContactMath.CalculatePairClosestTime(bodyAEvidence, bodyBEvidence);

    private static bool HasRelativeTimestepTrajectory(
        CrowdMotionEvidence bodyAEvidence,
        CrowdMotionEvidence bodyBEvidence) =>
        PersistentContactMath.HasRelativeTimestepTrajectory(bodyAEvidence, bodyBEvidence);

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
        incrementalStatistics.ContactActivationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - activationStart);
        // Activation can add constraints or rewrite the dormant schedule after
        // the previous commit, so certify the final consumer-visible views.
        IssueCertificateForCommittedViews(
            incrementalStatistics,
            substepIndex);
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
        if (!PersistentContactIndex.IsCreated ||
            !PersistentContactIndex.TryGetValue(key, out PersistentPredictiveContact contact))
            return;
        contact.NextCheckSubstep = nextCheckSubstep;
        PersistentContactIndex[key] = contact;
        // 同步列表（线性扫描，调用频率低）
        for (int i = 0; i < PersistentPredictiveContacts.Length; i++)
        {
            if (PersistentPredictiveContacts[i].Key.Equals(key))
            {
                PersistentPredictiveContacts[i] = contact;
                break;
            }
        }
        InvalidatePersistentContactViews();
    }

    private void MarkPersistentContactExpired(StableEntityPairKey key)
    {
        if (!PersistentContactIndex.IsCreated ||
            !PersistentContactIndex.TryGetValue(key, out PersistentPredictiveContact contact))
            return;
        contact.Lifecycle = PersistentContactLifecycle.Expired;
        contact.NextCheckSubstep = ushort.MaxValue;
        PersistentContactIndex[key] = contact;
        // 同步列表（线性扫描，调用频率低）
        for (int i = 0; i < PersistentPredictiveContacts.Length; i++)
        {
            if (PersistentPredictiveContacts[i].Key.Equals(key))
            {
                PersistentPredictiveContacts[i] = contact;
                break;
            }
        }
        InvalidatePersistentContactViews();
    }

    private int FindPersistentPredictiveContactIndex(StableEntityPairKey key) =>
        PersistentStoreLookup.FindPredictiveContactIndex(PersistentPredictiveContacts, key);

    private void InvalidatePersistentContactViews()
    {
        if (!EnablePersistentContactCache)
            return;
        IncrementalContactCacheState state = IncrementalCacheState.Value;
        state.ContactViewsValid = 0;
        IncrementalCacheState.Value = state;
    }

    /// <summary>
    /// 从 <see cref="PersistentPredictiveContacts"/> 全量重建 <see cref="PersistentContactIndex"/>。
    /// 在所有覆写列表的路径（全量重建、调度器写回）之后调用。O(N)，无排序。
    /// </summary>
    private void RebuildPersistentContactIndex()
    {
        if (!PersistentContactIndex.IsCreated) return;
        PersistentContactIndex.Clear();
        for (int i = 0; i < PersistentPredictiveContacts.Length; i++)
        {
            PersistentPredictiveContact c = PersistentPredictiveContacts[i];
            PersistentContactIndex[c.Key] = c;
        }
    }

    private void ClearPersistentClassificationCache()
    {
        PersistentPredictiveContacts.Clear();
        if (PersistentContactIndex.IsCreated) PersistentContactIndex.Clear();
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
                ContactPipelineMath.DeterministicFallbackNormal(bodyAIndex, bodyBIndex)),
            FirstActivatedSubstep = -1
        };
        return true;
    }

    private void RefreshCurrentContactStateGauges(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int currentActiveConstraintCount) =>
        PersistentContactMath.RefreshCurrentContactStateGauges(
            PredictiveContactScratch,
            ref incrementalStatistics,
            currentActiveConstraintCount);

    private static void UpdateActiveConstraintGauges(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int currentActiveConstraintCount) =>
        PersistentContactMath.UpdateActiveConstraintGauges(
            ref incrementalStatistics, currentActiveConstraintCount);

    private bool IsPersistentCacheStructurallyReusable() =>
        PersistentCacheReusability.IsStructurallyReusable(
            IncrementalCacheState.Value,
            Bodies.Length,
            PersistentSweptProxies.Length,
            PersistentProxyIndexByBody.Length,
            new PersistentCacheReusability.ConfigurationFingerprint
            {
                GuardMargin = GuardEnvelopeMargin,
                PredictiveSkin = PredictiveSkin,
                TimestepContactMargin = TimestepContactMargin,
                SoftAvoidanceShell = SoftAvoidanceShell,
                SoftAvoidanceResponseRate = SoftAvoidanceResponseRate,
                RvoTimeHorizon = RvoTimeHorizon,
                SubstepCount = SubstepCount,
                PredictivePairGenerationEnabled = EnablePredictivePairGeneration,
                PredictiveContactsEnabled = EnablePredictiveContacts,
                SoftAvoidanceVelocitySolver = SoftAvoidanceVelocitySolver
            });

    private static PersistentSweptProxy BuildPersistentProxyFromState(
        int bodyIndex,
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        float guardMargin,
        float softAvoidanceShell,
        float softAvoidanceResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon) =>
        PersistentProxyBuilder.BuildFromState(
            bodyIndex, stateSnapshot, stateEvidence, stateStep,
            guardMargin, softAvoidanceShell,
            softAvoidanceResponseRate, softSolverMode, rvoTimeHorizon);

    internal static IncrementalBodyDirtyFlags ClassifyAndUpdatePersistentProxyForBody(
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
        float rvoTimeHorizon) =>
        PersistentProxyBuilder.ClassifyAndUpdateForBody(
            bodyIndex, stateSnapshot, stateEvidence, stateStep,
            persistentProxies, proxyIndexByBody, cacheState,
            guardMargin, softAvoidanceShell,
            softAvoidanceResponseRate, softSolverMode, rvoTimeHorizon);

    private void PrepareInitialPersistentDirtyBodySet()
    {
        ClearIncrementalDirtyBodySet();
        if (!IsPersistentCacheStructurallyReusable())
            return;
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            IncrementalBodyDirtyFlags flags = ClassifyAndUpdatePersistentProxyForBody(
                bodyIndex, Bodies[bodyIndex], MotionEvidence[bodyIndex],
                StepStates[bodyIndex], PersistentSweptProxies.AsArray(),
                PersistentProxyIndexByBody.AsArray(), IncrementalCacheState.Value,
                GuardEnvelopeMargin, SoftAvoidanceShell, SoftAvoidanceResponseRate,
                SoftAvoidanceVelocitySolver, RvoTimeHorizon);
            if (flags != IncrementalBodyDirtyFlags.None)
                SetIncrementalDirtyFlags(bodyIndex, flags);
        }
    }

    private bool RefreshPreparedIncrementalDirtyBodies(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        out int topologyDirtyCount)
    {
        topologyDirtyCount = 0;
        if (!IsPersistentCacheStructurallyReusable())
            return false;
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            IncrementalDirtyBody dirty = IncrementalDirtyBodies[dirtyIndex];
            int bodyIndex = dirty.BodyIndex;
            if ((uint)bodyIndex >= (uint)Bodies.Length)
                return false;
            IncrementalBodyDirtyFlags refreshed = ClassifyAndUpdatePersistentProxyForBody(
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
        SummarizePreparedIncrementalDirtyBodies(
            ref incrementalStatistics, out topologyDirtyCount, out _);
        return true;
    }

    private void SummarizePreparedIncrementalDirtyBodies(
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

    private void AdvancePersistentCacheTimestep(
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

    private void RebuildPersistentProxyIndexByBody()
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
    private void ClearIncrementalDirtyBodySet() =>
        IncrementalDirtyBodyStore.Clear(IncrementalDirtyFlagsByBody, IncrementalDirtyBodies);

    private void SetIncrementalDirtyFlags(
        int bodyIndex,
        IncrementalBodyDirtyFlags flags) =>
        IncrementalDirtyBodyStore.SetFlags(
            bodyIndex, flags, IncrementalDirtyFlagsByBody, IncrementalDirtyBodies);

    private IncrementalBodyDirtyFlags GetDirtyFlags(int bodyIndex) =>
        IncrementalDirtyBodyStore.GetFlags(IncrementalDirtyFlagsByBody, bodyIndex);

    private bool IsTopologyDirtyEntity(Entity entity) =>
        IncrementalDirtyBodyStore.IsTopologyDirtyEntity(
            entity, CurrentBodyIndexByEntity, IncrementalDirtyFlagsByBody);

    private int FindPersistentProxyIndex(Entity entity) =>
        PersistentStoreLookup.FindProxyIndex(PersistentSweptProxies, entity);

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
            RebuildPersistentSpatialMembership(nextTopologyEpoch);
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
                TryAppendPersistentSpatialNeighbors(
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
                if (!ContactPipelineShared.AabbOverlaps(
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
        incrementalStatistics.LocalBroadPhaseNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
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
            incrementalStatistics.ProxyValidationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - validationStart);
            return true;
        }
        if (!RefreshPreparedIncrementalDirtyBodies(
                ref incrementalStatistics, out int topologyDirtyCount))
        {
            incrementalStatistics.ProxyValidationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - validationStart);
            return false;
        }
        float dirtyRatio = Bodies.Length > 0 ? (float)escapedBodyCount / Bodies.Length : 1f;
        if (dirtyRatio > IncrementalDirtyBodyRatioThreshold ||
            IncrementalCacheState.Value.IsValid == 0)
        {
            incrementalStatistics.ProxyValidationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - validationStart);
            return false;
        }
        incrementalStatistics.ProxyValidationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        long pairDiffStart = ProfilerUnsafeUtility.Timestamp;
        long localBefore = incrementalStatistics.LocalBroadPhaseNanoseconds;
        if (topologyDirtyCount > 0)
            IncrementallyRepairPersistentNeighborTopology(ref incrementalStatistics, false);
        long pairDiffElapsed = ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - pairDiffStart);
        long localElapsed = incrementalStatistics.LocalBroadPhaseNanoseconds - localBefore;
        long pairDiffExclusive = pairDiffElapsed - localElapsed;
        incrementalStatistics.PairDiffNanoseconds += pairDiffExclusive > 0L
            ? pairDiffExclusive
            : 0L;

        PreviousTimestepContactPairs.Clear();
        PreviousTimestepContactPairs.AddRange(TimestepContactPairs.AsArray());
        long mappingStart = ProfilerUnsafeUtility.Timestamp;
        if (!MapDirtyIncidentNeighborPairsToCurrentBodies(
                out int _,
                out int eligibilitySkipped))
        {
            incrementalStatistics.PersistentPairMappingNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - mappingStart);
            return false;
        }
        incrementalStatistics.PersistentPairMappingNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - mappingStart);
        incrementalStatistics.ClassificationSkippedCount += eligibilitySkipped;

        long contactViewStart = ProfilerUnsafeUtility.Timestamp;
        long classificationStart = contactViewStart;
        ClassifyAndPatchDirtyIncidentContacts(
            ref statistics, ref incrementalStatistics, scheduleStartSubstep);
        incrementalStatistics.SweptClassificationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - classificationStart);
        RebuildEscapedTimestepContactView(ref statistics, ref incrementalStatistics);
        statistics.TimestepContactSetBuildNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
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
        return MapDirtyIncidentNeighborPairsToCurrentBodies(
            out _,
            out _);
    }

    private bool MapDirtyIncidentNeighborPairsToCurrentBodies(
        out int dirtyIncidentPairCount,
        out int eligibilitySkippedCount)
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (EnableDiagnostics)
            PairDiagnostics.Clear();
#endif

        RebuildPersistentIncidentPairLookupIfNeededParallel();
        return DirtyIncidentPairMapper.TryMap(
            Pairs,
            IncrementalDirtyBodies,
            IncrementalDirtyFlagsByBody,
            Bodies,
            CurrentBodyIndexByEntity,
            PersistentIncidentPairLookup,
            PersistentIncidentLookupEpoch,
            PersistentNeighborPairs,
            PersistentContactIndex,
            PersistentSweptProxies,
            PersistentProxyIndexByBody,
            IncrementalCacheState.Value,
            out dirtyIncidentPairCount,
            out eligibilitySkippedCount);
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
            {
                // 从 O(1) 索引中移除 dirty 条目，后续会用新分类结果补回
                if (PersistentContactIndex.IsCreated)
                    PersistentContactIndex.Remove(contact.Key);
                continue;
            }
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
                new BodyPair(rawPair.BodyA, rawPair.BodyB),
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
            // 新分类结果写入 O(1) 索引
            if (PersistentContactIndex.IsCreated)
                PersistentContactIndex[key] = contact;
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
        // 无需对 PredictiveContactScratch 排序：hashmap 已提供 O(1) 查找，
        // 列表仅用于迭代（视图重建、下帧 dirty 过滤），顺序无关。
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
        ContactPipelineShared.SortAndDeduplicateConstraints(TimestepContactPairs);

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

    private bool IsDirtyBodyIndex(int bodyIndex) =>
        IncrementalDirtyBodyStore.IsDirtyBodyIndex(IncrementalDirtyFlagsByBody, bodyIndex);

    private bool IsDirtyEntity(Entity entity) =>
        IncrementalDirtyBodyStore.IsDirtyEntity(
            entity, CurrentBodyIndexByEntity, IncrementalDirtyFlagsByBody);

    private void BuildCurrentIncrementalSweptProxies()
    {
        CurrentIncrementalProxies.Clear();
        float guardMargin = math.max(0f, GuardEnvelopeMargin);

        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CurrentIncrementalProxies.Add(BuildPersistentProxyFromState(
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
        out float2 tightMax) =>
        PersistentContactMath.CalculateIncrementalTightSweptBounds(
            stateSnapshot, stateEvidence, stateStep,
            PredictiveSkin, TimestepContactMargin, SoftAvoidanceShell,
            RvoTimeHorizon, SoftAvoidanceVelocitySolver,
            out tightMin, out tightMax);

    private void CalculateIncrementalValidationBounds(
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        out float2 validationMin,
        out float2 validationMax) =>
        PersistentContactMath.CalculateIncrementalValidationBounds(
            stateSnapshot, stateEvidence, stateStep,
            PredictiveSkin, TimestepContactMargin, SoftAvoidanceShell,
            out validationMin, out validationMax);

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
        PersistentSweptProxy previous) =>
        PersistentProxyBuilder.AssignMotionVersion(ref current, previous);

    private void FullRebuildPersistentNeighborTopology(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long broadPhaseStart = ProfilerUnsafeUtility.Timestamp;
        SweptCellEntries.Clear();
        Pairs.Clear();
        PersistentSweptProxies.Clear();
        PersistentNeighborPairs.Clear();
        PersistentSweptProxies.AddRange(CurrentIncrementalProxies.AsArray());
        RebuildPersistentProxyIndexByBody();

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

        ContactPipelineShared.SortAndDeduplicateConstraints(Pairs);
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
                !ContactPipelineShared.AabbOverlaps(proxyA.GuardMin, proxyA.GuardMax, proxyB.GuardMin, proxyB.GuardMax))
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
        incrementalStatistics.LocalBroadPhaseNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
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
        ContactPipelineShared.SortAndDeduplicateBodyPairs(TimestepInteractionPairs);
        return true;
    }

    private bool MapPersistentNeighborPairsToCurrentBodies()
    {
        if (!RefreshTimestepInteractionPairs())
            return false;

        Pairs.Clear();
#if RTS_CONTACT_DIAGNOSTICS
        if (EnableDiagnostics)
            PairDiagnostics.Clear();
#endif
        ContactPipelineShared.AppendBodyPairsAsConstraints(TimestepInteractionPairs.AsArray(), Pairs);
        return true;
    }

    private bool TryFindPersistentProxy(Entity entity, out PersistentSweptProxy proxy) =>
        PersistentStoreLookup.TryFindPersistentProxy(PersistentSweptProxies, entity, out proxy);

    private bool TryFindIncrementalProxy(Entity entity, out PersistentSweptProxy proxy) =>
        PersistentStoreLookup.TryFindIncrementalProxy(CurrentIncrementalProxies, entity, out proxy);

    private void SortAndDeduplicatePersistentNeighborPairs()
    {
        SortAndDeduplicatePersistentNeighborPairs(PersistentNeighborPairs);
    }

    private static void SortAndDeduplicatePersistentNeighborPairs(
        NativeList<PersistentNeighborPair> pairs) =>
        PersistentContactMath.SortAndDeduplicatePersistentNeighborPairs(pairs);
}
}
