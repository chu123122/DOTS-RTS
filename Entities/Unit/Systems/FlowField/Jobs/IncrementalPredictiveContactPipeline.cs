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
            long localBroadPhaseBefore = incrementalStatistics.LocalBroadPhaseNanoseconds;
            FullRebuildPersistentNeighborTopology(ref incrementalStatistics);
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

        incrementalStatistics.PersistentNeighborPairCount = PersistentNeighborPairs.Length;
        return true;
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
        UpdateActiveConstraintGauges(
            ref incrementalStatistics,
            TimestepContactPairs.Length);
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
        IncrementalDirtyBodies.Clear();
        IncrementalContactCacheState state = IncrementalCacheState.Value;
        if (state.IsValid == 0 ||
            state.BodyCount != States.Length ||
            PersistentSweptProxies.Length != CurrentIncrementalProxies.Length ||
            math.abs(state.GuardMargin - math.max(0f, FatAabbCacheMargin)) > 0.000001f ||
            math.abs(state.PredictiveSkin - math.max(0f, PredictiveSkin)) > 0.000001f ||
            math.abs(state.TimestepContactMargin - math.max(0f, TimestepContactMargin)) > 0.000001f ||
            math.abs(state.SoftAvoidanceShell - math.max(0f, SoftAvoidanceShell)) > 0.000001f)
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
        int substepIndex,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long validationStart = ProfilerUnsafeUtility.Timestamp;
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

        long mappingStart = ProfilerUnsafeUtility.Timestamp;
        if (!RefreshTimestepInteractionPairs())
        {
            incrementalStatistics.PersistentPairMappingNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - mappingStart);
            return false;
        }
        incrementalStatistics.CurrentInteractionPairCount =
            TimestepInteractionPairs.Length;

        MappedFatCachePairs.Clear();
        MappedFatCachePairs.AddRange(TimestepContactPairs.AsArray());
        incrementalStatistics.PersistentPairMappingNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - mappingStart);

        long contactViewStart = ProfilerUnsafeUtility.Timestamp;
        FinalizeTimestepContactView(
            ref statistics,
            ref incrementalStatistics,
            true,
            substepIndex + 1);
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
        float contactPadding = math.max(0f, PredictiveSkin) +
                               math.max(0f, TimestepContactMargin) * 2f;
        float avoidancePadding = math.max(0f, SoftAvoidanceShell) * 0.5f;
        float extent = math.max(0f, state.Radius) +
                       math.max(contactPadding, avoidancePadding);
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
        state.SoftAvoidanceShell = math.max(0f, SoftAvoidanceShell);
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
