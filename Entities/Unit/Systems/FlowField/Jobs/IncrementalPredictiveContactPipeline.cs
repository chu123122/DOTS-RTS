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
        bool forceFullRebuild)
    {
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
            long buildStart = ProfilerUnsafeUtility.Timestamp;
            FullRebuildPersistentNeighborTopology(ref incrementalStatistics);
            incrementalStatistics.FallbackNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - buildStart);
            incrementalStatistics.FullRebuildCount++;
            incrementalStatistics.UsedFullRebuild = 1;
        }
        else
        {
            long repairStart = ProfilerUnsafeUtility.Timestamp;
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

            incrementalStatistics.PairDiffNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - repairStart);
            incrementalStatistics.UsedIncrementalTopology = 1;
        }

        long classificationStart = ProfilerUnsafeUtility.Timestamp;
        bool mapped = MapPersistentNeighborPairsToCurrentBodies();
        if (!mapped)
        {
            IncrementalContactCacheState invalidState = IncrementalCacheState.Value;
            invalidState.IsValid = 0;
            IncrementalCacheState.Value = invalidState;
            BuildSweptContactPairs(ref statistics);
            incrementalStatistics.UsedFullRebuild = 1;
            return false;
        }

        statistics.CandidatePairCount += Pairs.Length;
        incrementalStatistics.ReclassifiedPairCount += Pairs.Length;
        FilterAndClassifyPairs(ref statistics, math.max(0f, PredictiveSkin));
        incrementalStatistics.SweptHitCount += Pairs.Length;
        SynchronizePersistentPredictiveContacts(ref incrementalStatistics);
        incrementalStatistics.SweptClassificationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - classificationStart);
        incrementalStatistics.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
        return true;
    }

    private void SynchronizePersistentPredictiveContacts(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        PredictiveContactScratch.Clear();
        PredictiveContactSchedule.Clear();
        uint timestep = IncrementalCacheState.Value.Timestep;
        int substepCount = math.max(1, SubstepCount);

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

            float3 stableNormal = pair.PredictiveNormal;
            sbyte fixedSide = pair.ContactMode == UnitContactMode.Predictive
                ? (sbyte)1
                : (sbyte)0;
            if (TryFindPersistentPredictiveContact(
                    key,
                    out PersistentPredictiveContact previous) &&
                math.dot(previous.StableNormal, stableNormal) > 0.5f)
            {
                stableNormal = previous.StableNormal;
                fixedSide = previous.FixedSide != 0 ? previous.FixedSide : fixedSide;
            }

            PersistentSweptProxy proxyA = default;
            PersistentSweptProxy proxyB = default;
            TryFindPersistentProxy(bodyA.Entity, out proxyA);
            TryFindPersistentProxy(bodyB.Entity, out proxyB);

            ushort firstPossibleSubstep = 0;
            if (lifecycle == PersistentContactLifecycle.Dormant)
            {
                float closestTime = CalculatePairClosestTime(bodyA, bodyB);
                int closestSubstep = math.clamp(
                    (int)math.floor(closestTime * substepCount),
                    0,
                    substepCount - 1);
                // Wake one substep early. The retained contact margin is the safety
                // budget for solver/RVO deviations; any larger deviation triggers
                // the envelope-escape repair path.
                firstPossibleSubstep = (ushort)math.max(0, closestSubstep - 1);
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

            switch (lifecycle)
            {
                case PersistentContactLifecycle.Dormant:
                    incrementalStatistics.DormantPairCount++;
                    PredictiveContactSchedule.Add(new PredictiveContactScheduleEntry
                    {
                        Key = key,
                        Substep = firstPossibleSubstep
                    });
                    break;
                case PersistentContactLifecycle.Predictive:
                case PersistentContactLifecycle.Approaching:
                    incrementalStatistics.PredictivePairCount++;
                    break;
                case PersistentContactLifecycle.Actual:
                    incrementalStatistics.ActualPairCount++;
                    break;
            }
        }

        if (PredictiveContactScratch.Length > 1)
            PredictiveContactScratch.AsArray().Sort(new PersistentPredictiveContactComparer());
        if (PredictiveContactSchedule.Length > 1)
            PredictiveContactSchedule.AsArray().Sort(new PredictiveContactScheduleEntryComparer());
        PersistentPredictiveContacts.Clear();
        PersistentPredictiveContacts.AddRange(PredictiveContactScratch.AsArray());

        // Dormant contacts live in the persistent cache/schedule, not in the
        // per-substep solver set. This is the main candidate-utilization win.
        int activeWriteIndex = 0;
        for (int pairIndex = 0; pairIndex < Pairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = Pairs[pairIndex];
            if (pair.IsDormant != 0)
                continue;
            Pairs[activeWriteIndex++] = pair;
        }
        Pairs.ResizeUninitialized(activeWriteIndex);
        incrementalStatistics.ActiveConstraintCount = activeWriteIndex;
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

    private void ActivateScheduledPredictiveContactsForSubstep(
        int substepIndex,
        int substepCount,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        if (PredictiveContactSchedule.Length == 0)
            return;

        long activationStart = ProfilerUnsafeUtility.Timestamp;
        bool addedPair = false;
        for (int scheduleIndex = 0;
             scheduleIndex < PredictiveContactSchedule.Length;
             scheduleIndex++)
        {
            PredictiveContactScheduleEntry entry = PredictiveContactSchedule[scheduleIndex];
            if (entry.Substep != substepIndex)
                continue;

            incrementalStatistics.ScheduledWakeupCount++;
            if (!TryFindCurrentBodyIndex(entry.Key.EntityA, out int bodyA) ||
                !TryFindCurrentBodyIndex(entry.Key.EntityB, out int bodyB))
            {
                entry.Substep = ushort.MaxValue;
                PredictiveContactSchedule[scheduleIndex] = entry;
                continue;
            }

            if (TryBuildCurrentScheduledPair(bodyA, bodyB, out UnitCollisionPair pair))
            {
                if (FindPairIndex(TimestepContactPairs, pair.BodyA, pair.BodyB) < 0)
                {
                    TimestepContactPairs.Add(pair);
                    addedPair = true;
                    incrementalStatistics.ActiveConstraintCount++;
                }
                entry.Substep = ushort.MaxValue;
            }
            else if (substepIndex + 1 < substepCount)
            {
                entry.Substep = (ushort)(substepIndex + 1);
            }
            else
            {
                entry.Substep = ushort.MaxValue;
            }
            PredictiveContactSchedule[scheduleIndex] = entry;
        }

        if (addedPair)
            TimestepContactPairs.AsArray().Sort(new UnitCollisionPairComparer());
        incrementalStatistics.ContactActivationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - activationStart);
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

    private bool TryFindPersistentPredictiveContact(
        StableEntityPairKey key,
        out PersistentPredictiveContact contact)
    {
        int low = 0;
        int high = PersistentPredictiveContacts.Length - 1;
        var comparer = new StableEntityPairKeyComparer();
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            PersistentPredictiveContact candidate = PersistentPredictiveContacts[middle];
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

    private static PersistentContactLifecycle ClassifyPersistentLifecycle(
        UnitCollisionPair pair,
        FlowMovementFrameState bodyA,
        FlowMovementFrameState bodyB)
    {
        float3 currentDelta = bodyA.TimestepStartPosition - bodyB.TimestepStartPosition;
        currentDelta.y = 0f;
        float radiusSum = bodyA.Radius + bodyB.Radius;
        if (math.lengthsq(currentDelta) <= radiusSum * radiusSum)
            return PersistentContactLifecycle.Actual;
        if (pair.IsDormant != 0)
            return PersistentContactLifecycle.Dormant;
        if (pair.ContactMode == UnitContactMode.Predictive)
            return PersistentContactLifecycle.Predictive;
        return PersistentContactLifecycle.Approaching;
    }

    private PersistentPredictiveContact CreatePersistentPredictiveContact(
        StableEntityPairKey key,
        UnitCollisionPair pair,
        FlowMovementFrameState bodyA,
        FlowMovementFrameState bodyB,
        PersistentContactLifecycle lifecycle,
        uint timestep)
    {
        float3 stableNormal = pair.PredictiveNormal;
        sbyte fixedSide = pair.ContactMode == UnitContactMode.Predictive
            ? (sbyte)1
            : (sbyte)0;
        if (TryFindPersistentPredictiveContact(
                key,
                out PersistentPredictiveContact previous) &&
            math.dot(previous.StableNormal, stableNormal) > 0.5f)
        {
            stableNormal = previous.StableNormal;
            fixedSide = previous.FixedSide != 0 ? previous.FixedSide : fixedSide;
        }

        PersistentSweptProxy proxyA = default;
        PersistentSweptProxy proxyB = default;
        TryFindPersistentProxy(bodyA.Entity, out proxyA);
        TryFindPersistentProxy(bodyB.Entity, out proxyB);
        return new PersistentPredictiveContact
        {
            Key = key,
            StableNormal = stableNormal,
            Lifecycle = lifecycle,
            FixedSide = fixedSide,
            FirstPossibleSubstep = lifecycle == PersistentContactLifecycle.Dormant
                ? ushort.MaxValue
                : (ushort)0,
            NextCheckSubstep = lifecycle == PersistentContactLifecycle.Dormant
                ? ushort.MaxValue
                : (ushort)0,
            LastSeenTimestep = timestep,
            MotionVersionA = proxyA.MotionVersion,
            MotionVersionB = proxyB.MotionVersion
        };
    }

    private static void AccumulatePersistentLifecycleStatistics(
        PersistentContactLifecycle lifecycle,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        switch (lifecycle)
        {
            case PersistentContactLifecycle.Dormant:
                incrementalStatistics.DormantPairCount++;
                break;
            case PersistentContactLifecycle.Predictive:
            case PersistentContactLifecycle.Approaching:
                incrementalStatistics.PredictivePairCount++;
                break;
            case PersistentContactLifecycle.Actual:
                incrementalStatistics.ActualPairCount++;
                break;
        }
    }

    private bool ValidateAndClassifyIncrementalDirtyBodies(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        IncrementalDirtyBodies.Clear();
        IncrementalContactCacheState state = IncrementalCacheState.Value;
        if (state.IsValid == 0 ||
            state.BodyCount != States.Length ||
            PersistentSweptProxies.Length != CurrentIncrementalProxies.Length ||
            math.abs(state.GuardMargin - math.max(0f, FatAabbCacheMargin)) > 0.000001f ||
            math.abs(state.PredictiveSkin - math.max(0f, PredictiveSkin)) > 0.000001f ||
            math.abs(state.TimestepContactMargin - math.max(0f, TimestepContactMargin)) > 0.000001f)
            return false;

        int validProxyCount = 0;
        for (int proxyIndex = 0; proxyIndex < CurrentIncrementalProxies.Length; proxyIndex++)
        {
            PersistentSweptProxy current = CurrentIncrementalProxies[proxyIndex];
            PersistentSweptProxy previous = PersistentSweptProxies[proxyIndex];
            if (current.Entity != previous.Entity || current.IsValid != previous.IsValid)
                return false;

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
            {
                IncrementalDirtyBodies.Add(new IncrementalDirtyBody
                {
                    BodyIndex = current.BodyIndex,
                    Flags = flags
                });
            }
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
            previous.MotionVersion = current.MotionVersion;
            previous.IsValid = current.IsValid;
            PersistentSweptProxies[proxyIndex] = previous;
        }
    }

    private IncrementalBodyDirtyFlags GetDirtyFlags(int bodyIndex)
    {
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            IncrementalDirtyBody dirty = IncrementalDirtyBodies[dirtyIndex];
            if (dirty.BodyIndex == bodyIndex)
                return dirty.Flags;
        }
        return IncrementalBodyDirtyFlags.None;
    }

    private bool IsTopologyDirtyEntity(Entity entity)
    {
        if (!TryFindCurrentBodyIndex(entity, out int bodyIndex))
            return true;
        return (GetDirtyFlags(bodyIndex) & IncrementalBodyDirtyFlags.Topology) != 0;
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
        for (int dirtyIndex = 0; dirtyIndex < IncrementalDirtyBodies.Length; dirtyIndex++)
        {
            IncrementalDirtyBody dirty = IncrementalDirtyBodies[dirtyIndex];
            if ((dirty.Flags & IncrementalBodyDirtyFlags.Topology) == 0)
                continue;

            FlowMovementFrameState dirtyState = States[dirty.BodyIndex];
            if (!TryFindPersistentProxy(
                    dirtyState.Entity,
                    out PersistentSweptProxy dirtyProxy) ||
                dirtyProxy.IsValid == 0)
                continue;

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
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long repairStart = ProfilerUnsafeUtility.Timestamp;
        PrepareCurrentBodyLookup();
        BuildCurrentIncrementalSweptProxies();
        IncrementalDirtyBodies.Clear();

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

            IncrementalDirtyBodies.Add(new IncrementalDirtyBody
            {
                BodyIndex = current.BodyIndex,
                Flags = flags
            });
        }

        if (escapedBodyCount == 0)
            return true;

        float dirtyRatio = States.Length > 0
            ? (float)escapedBodyCount / States.Length
            : 1f;
        if (dirtyRatio > IncrementalDirtyBodyRatioThreshold ||
            IncrementalCacheState.Value.IsValid == 0)
            return false;

        UpdatePersistentProxyMetadata();
        if (topologyDirtyCount > 0)
        {
            IncrementallyRepairPersistentNeighborTopology(
                ref incrementalStatistics,
                false);
        }

        MappedFatCachePairs.Clear();
        MappedFatCachePairs.AddRange(TimestepContactPairs.AsArray());
        if (!MapDirtyIncidentNeighborPairsToCurrentBodies())
            return false;

        statistics.CandidatePairCount += Pairs.Length;
        incrementalStatistics.ReclassifiedPairCount += Pairs.Length;
        long classificationStart = ProfilerUnsafeUtility.Timestamp;
        FilterAndClassifyPairs(ref statistics, math.max(0f, PredictiveSkin));
        incrementalStatistics.SweptHitCount += Pairs.Length;
        PatchPersistentPredictiveContactsForDirtyBodies(ref incrementalStatistics);
        incrementalStatistics.SweptClassificationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - classificationStart);

        TimestepContactPairs.Clear();
        for (int previousIndex = 0; previousIndex < MappedFatCachePairs.Length; previousIndex++)
        {
            UnitCollisionPair previous = MappedFatCachePairs[previousIndex];
            if (IsDirtyBodyIndex(previous.BodyA) || IsDirtyBodyIndex(previous.BodyB))
                continue;
            TimestepContactPairs.Add(previous);
        }

        for (int pairIndex = 0; pairIndex < Pairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = Pairs[pairIndex];
            int previousIndex = FindPairIndex(
                MappedFatCachePairs,
                pair.BodyA,
                pair.BodyB);
            if (previousIndex >= 0)
            {
                UnitCollisionPair previous = MappedFatCachePairs[previousIndex];
                pair.WasActivatedThisTimestep = previous.WasActivatedThisTimestep;
                pair.FirstActivatedSubstep = previous.FirstActivatedSubstep;
                pair.ActivatedSubstepCount = previous.ActivatedSubstepCount;
            }
            else
            {
                pair.WasAddedByFallback = 1;
                statistics.TimestepContactSetFallbackAddedPairCount++;
            }
            TimestepContactPairs.Add(pair);
        }
        SortAndDeduplicateBodyPairs(TimestepContactPairs);
        statistics.TimestepContactSetDormantPairCount = 0;
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            if (TimestepContactPairs[pairIndex].IsDormant != 0)
                statistics.TimestepContactSetDormantPairCount++;
        }

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
        incrementalStatistics.PairDiffNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - repairStart);
        return true;
    }

    private bool MapDirtyIncidentNeighborPairsToCurrentBodies()
    {
        Pairs.Clear();
        if (EnableDiagnostics)
            PairDiagnostics.Clear();

        for (int pairIndex = 0; pairIndex < PersistentNeighborPairs.Length; pairIndex++)
        {
            StableEntityPairKey key = PersistentNeighborPairs[pairIndex].Key;
            if (!IsDirtyEntity(key.EntityA) && !IsDirtyEntity(key.EntityB))
                continue;
            if (!TryFindCurrentBodyIndex(key.EntityA, out int bodyA) ||
                !TryFindCurrentBodyIndex(key.EntityB, out int bodyB))
                return false;
            Pairs.Add(new UnitCollisionPair
            {
                BodyA = math.min(bodyA, bodyB),
                BodyB = math.max(bodyA, bodyB)
            });
        }

        SortAndDeduplicatePairs();
        return true;
    }

    private void PatchPersistentPredictiveContactsForDirtyBodies(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        PredictiveContactScratch.Clear();
        for (int contactIndex = 0;
             contactIndex < PersistentPredictiveContacts.Length;
             contactIndex++)
        {
            PersistentPredictiveContact contact = PersistentPredictiveContacts[contactIndex];
            if (IsDirtyEntity(contact.Key.EntityA) || IsDirtyEntity(contact.Key.EntityB))
                continue;
            PredictiveContactScratch.Add(contact);
        }

        uint timestep = IncrementalCacheState.Value.Timestep;
        for (int pairIndex = 0; pairIndex < Pairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = Pairs[pairIndex];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            StableEntityPairKey key = StableEntityPairKey.Create(bodyA.Entity, bodyB.Entity);
            PersistentContactLifecycle lifecycle = ClassifyPersistentLifecycle(
                pair,
                bodyA,
                bodyB);
            PredictiveContactScratch.Add(CreatePersistentPredictiveContact(
                key,
                pair,
                bodyA,
                bodyB,
                lifecycle,
                timestep));
            AccumulatePersistentLifecycleStatistics(
                lifecycle,
                ref incrementalStatistics);
        }

        if (PredictiveContactScratch.Length > 1)
            PredictiveContactScratch.AsArray().Sort(new PersistentPredictiveContactComparer());
        PersistentPredictiveContacts.Clear();
        PersistentPredictiveContacts.AddRange(PredictiveContactScratch.AsArray());
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
        float guardMargin = math.max(0f, FatAabbCacheMargin);

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
                proxy.MotionVersion = CalculateMotionVersion(state);
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
        float retainedPadding = math.max(0f, PredictiveSkin) +
                                math.max(0f, TimestepContactMargin) * 2f;
        float extent = math.max(0f, state.Radius) + retainedPadding;
        CalculateNeighborPathBounds(state, out float2 pathMin, out float2 pathMax);
        tightMin = pathMin - extent;
        tightMax = pathMax + extent;
    }

    private static uint CalculateMotionVersion(FlowMovementFrameState state)
    {
        // Deterministic, allocation-free trajectory fingerprint. It is only used
        // to detect likely reclassification work; correctness never depends on a
        // hash collision because topology remains guarded by AABB containment.
        uint hash = math.hash(new float4(
            state.TimestepStartPosition.x,
            state.TimestepStartPosition.z,
            state.TimestepPredictedPosition.x,
            state.TimestepPredictedPosition.z));
        hash = math.hash(new uint2(hash, math.asuint(state.Radius)));
        return hash;
    }

    private void FullRebuildPersistentNeighborTopology(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long broadPhaseStart = ProfilerUnsafeUtility.Timestamp;
        ShadowCellEntries.Clear();
        ShadowBodyPairs.Clear();
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
                    ShadowCellEntries.Add(new SweptDiscCellEntry
                    {
                        CellIndex = FlowFieldUtils.GetFlatIndex(new int2(x, y), GridDimensions),
                        BodyIndex = proxy.BodyIndex
                    });
                }
            }
        }

        ShadowCellEntries.AsArray().Sort(new SweptDiscCellEntryComparer());
        int cellStart = 0;
        while (cellStart < ShadowCellEntries.Length)
        {
            int cellIndex = ShadowCellEntries[cellStart].CellIndex;
            int cellEnd = cellStart + 1;
            while (cellEnd < ShadowCellEntries.Length &&
                   ShadowCellEntries[cellEnd].CellIndex == cellIndex)
                cellEnd++;

            for (int first = cellStart; first < cellEnd; first++)
            {
                int bodyA = ShadowCellEntries[first].BodyIndex;
                for (int second = first + 1; second < cellEnd; second++)
                {
                    int bodyB = ShadowCellEntries[second].BodyIndex;
                    if (bodyA == bodyB)
                        continue;
                    ShadowBodyPairs.Add(new UnitCollisionPair
                    {
                        BodyA = math.min(bodyA, bodyB),
                        BodyB = math.max(bodyA, bodyB)
                    });
                }
            }

            cellStart = cellEnd;
        }

        SortAndDeduplicateBodyPairs(ShadowBodyPairs);
        uint nextTopologyEpoch = IncrementalCacheState.Value.TopologyEpoch + 1u;
        for (int pairIndex = 0; pairIndex < ShadowBodyPairs.Length; pairIndex++)
        {
            UnitCollisionPair bodyPair = ShadowBodyPairs[pairIndex];
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
        state.GuardMargin = math.max(0f, FatAabbCacheMargin);
        state.PredictiveSkin = math.max(0f, PredictiveSkin);
        state.TimestepContactMargin = math.max(0f, TimestepContactMargin);
        IncrementalCacheState.Value = state;

        incrementalStatistics.Timestep = state.Timestep;
        incrementalStatistics.ProxyCount = validProxyCount;
        incrementalStatistics.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
        incrementalStatistics.NeighborPairAddedCount = PersistentNeighborPairs.Length;
        incrementalStatistics.LocalBroadPhaseNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - broadPhaseStart);
    }

    private bool MapPersistentNeighborPairsToCurrentBodies()
    {
        Pairs.Clear();
        if (EnableDiagnostics)
            PairDiagnostics.Clear();

        for (int pairIndex = 0; pairIndex < PersistentNeighborPairs.Length; pairIndex++)
        {
            StableEntityPairKey key = PersistentNeighborPairs[pairIndex].Key;
            if (!TryFindCurrentBodyIndex(key.EntityA, out int bodyA) ||
                !TryFindCurrentBodyIndex(key.EntityB, out int bodyB))
                return false;

            Pairs.Add(new UnitCollisionPair
            {
                BodyA = math.min(bodyA, bodyB),
                BodyB = math.max(bodyA, bodyB)
            });
        }

        SortAndDeduplicatePairs();
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
