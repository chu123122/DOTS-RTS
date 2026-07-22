using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct SolveXpbdUnitContactsJob
{
    private void PrepareCurrentBodyLookup()
    {
        ShadowCurrentProxies.Clear();
        CurrentBodyIndexByEntity.Clear();
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            CurrentBodyIndexByEntity.TryAdd(state.Entity, bodyIndex);
            ShadowCurrentProxies.Add(new ShadowFatBodyProxy
            {
                Entity = state.Entity,
                BodyIndex = bodyIndex,
                IsValid = (byte)(state.IsInsideGrid ? 1 : 0)
            });
        }
        ShadowCurrentProxies.AsArray().Sort(new ShadowFatBodyProxyComparer());
    }

    private bool BuildContactPairsFromFatAabbCache(
        ref PredictiveDiscContactStatistics statistics,
        ref ShadowNeighborCacheStatistics cacheStatistics,
        ref bool fatCachePairsMappedThisFrame)
    {
        if (!EnsureFatAabbRawCandidates(
                ref cacheStatistics,
                ref fatCachePairsMappedThisFrame,
                true))
        {
            BuildSweptInteractionPairs(ref statistics);
            cacheStatistics.FullBroadPhaseFallbackCount++;
            return false;
        }

        Pairs.Clear();
        if (EnableDiagnostics)
            PairDiagnostics.Clear();
        Pairs.AddRange(MappedFatCachePairs.AsArray());

        cacheStatistics.CacheUseCount++;
        cacheStatistics.CachedNarrowPhasePairCheckCount += Pairs.Length;
        statistics.CandidatePairCount += Pairs.Length;
        FilterAndClassifyPairs(ref statistics, math.max(0f, PredictiveSkin));
        return true;
    }

    private bool EnsureFatAabbRawCandidates(
        ref ShadowNeighborCacheStatistics cacheStatistics,
        ref bool fatCachePairsMappedThisFrame,
        bool countContactReuse)
    {
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        bool cacheValid = IsFatAabbCacheValid(
            out bool entitySetInvalid,
            out bool boundsInvalid);
        cacheStatistics.CacheValidationCount++;
        cacheStatistics.ValidationNanoseconds += TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - validationStart);

        if (!cacheValid)
        {
            FatAabbCacheState previousState = FatAabbCacheState.Value;
            if (previousState.IsValid != 0)
            {
                cacheStatistics.CacheInvalidationCount++;
                if (entitySetInvalid)
                    cacheStatistics.EntitySetInvalidationCount++;
                if (boundsInvalid)
                    cacheStatistics.BoundsInvalidationCount++;
            }

            long buildStart = ProfilerUnsafeUtility.Timestamp;
            BuildCurrentShadowCache(ref cacheStatistics);
            PromoteCurrentShadowCache();
            cacheStatistics.CacheBuildNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - buildStart);
            cacheStatistics.CacheRebuildCount++;

            var rebuiltState = new FatAabbCacheState
            {
                IsValid = 1,
                Mode = FatCacheMode.Global,
                AgeFrames = 0,
                PredictiveSkin = math.max(0f, PredictiveSkin),
                SoftAvoidanceShell = math.max(0f, SoftAvoidanceShell),
                SoftAvoidanceVelocitySolver = SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = math.max(0f, RvoTimeHorizon),
                Margin = math.max(0f, FatAabbCacheMargin)
            };
            FatAabbCacheState.Value = rebuiltState;
            fatCachePairsMappedThisFrame = false;
        }
        else if (countContactReuse)
        {
            cacheStatistics.CacheReuseCount++;
        }

        if (!fatCachePairsMappedThisFrame)
        {
            long mappingStart = ProfilerUnsafeUtility.Timestamp;
            bool mapped = MapFatCacheCandidatesToCurrentBodies();
            cacheStatistics.CachePairMappingNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - mappingStart);
            cacheStatistics.CachePairMappingBuildCount++;
            if (!mapped)
            {
                InvalidateFatAabbCache(ref cacheStatistics, false);
                return false;
            }

            fatCachePairsMappedThisFrame = true;
        }
        else
        {
            cacheStatistics.CachePairMappingReuseCount++;
        }

        return true;
    }

    private bool IsFatAabbCacheValid(
        out bool entitySetInvalid,
        out bool boundsInvalid)
    {
        entitySetInvalid = false;
        boundsInvalid = false;
        FatAabbCacheState cacheState = FatAabbCacheState.Value;
        if (cacheState.IsValid == 0)
            return false;

        if (math.abs(cacheState.PredictiveSkin - math.max(0f, PredictiveSkin)) > 0.000001f ||
            math.abs(cacheState.SoftAvoidanceShell -
                     math.max(0f, SoftAvoidanceShell)) > 0.000001f ||
            cacheState.SoftAvoidanceVelocitySolver != SoftAvoidanceVelocitySolver ||
            math.abs(cacheState.RvoTimeHorizon -
                     math.max(0f, RvoTimeHorizon)) > 0.000001f ||
            math.abs(cacheState.Margin - math.max(0f, FatAabbCacheMargin)) > 0.000001f)
        {
            boundsInvalid = true;
            return false;
        }

        if (ShadowPreviousProxies.Length != States.Length)
        {
            entitySetInvalid = true;
            return false;
        }

        float neighborPadding = CalculateFatAabbNeighborPadding();
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!TryFindProxyEntry(
                    ShadowPreviousProxies,
                    state.Entity,
                    out ShadowFatBodyProxy proxy))
            {
                entitySetInvalid = true;
                return false;
            }

            byte expectedValid = (byte)(state.IsInsideGrid ? 1 : 0);
            if (proxy.IsValid != expectedValid)
            {
                entitySetInvalid = true;
                return false;
            }
            if (!state.IsInsideGrid)
                continue;

            CalculateCoreSweptBounds(
                state,
                neighborPadding,
                out float2 coreMin,
                out float2 coreMax);
            if (AabbContains(proxy.FatMin, proxy.FatMax, coreMin, coreMax))
                continue;

            boundsInvalid = true;
            return false;
        }

        return true;
    }

    private bool MapFatCacheCandidatesToCurrentBodies()
    {
        MappedFatCachePairs.Clear();

        for (int i = 0; i < ShadowPreviousPairs.Length; i++)
        {
            ShadowEntityPair entityPair = ShadowPreviousPairs[i];
            if (!TryFindCurrentBodyIndex(entityPair.EntityA, out int bodyA) ||
                !TryFindCurrentBodyIndex(entityPair.EntityB, out int bodyB))
                return false;

            MappedFatCachePairs.Add(new UnitCollisionPair
            {
                BodyA = math.min(bodyA, bodyB),
                BodyB = math.max(bodyA, bodyB)
            });
        }

        return true;
    }

    private bool TryFindCurrentBodyIndex(Entity entity, out int bodyIndex)
    {
        return CurrentBodyIndexByEntity.TryGetValue(entity, out bodyIndex) &&
               bodyIndex >= 0 && bodyIndex < States.Length;
    }

    private void CalculateCoreSweptBounds(
        FlowMovementFrameState state,
        float neighborPadding,
        out float2 coreMin,
        out float2 coreMax)
    {
        float extent = math.max(0f, state.Radius) + math.max(0f, neighborPadding);
        CalculateNeighborPathBounds(state, out float2 pathMin, out float2 pathMax);
        coreMin = pathMin - extent;
        coreMax = pathMax + extent;
    }

    private void CalculateNeighborPathBounds(
        FlowMovementFrameState state,
        out float2 pathMin,
        out float2 pathMax)
    {
        pathMin = math.min(
            state.TimestepStartPosition.xz,
            math.min(
                state.TimestepPredictedPosition.xz,
                math.min(state.UnconstrainedPredictedPosition.xz, state.PredictedPosition.xz)));
        pathMax = math.max(
            state.TimestepStartPosition.xz,
            math.max(
                state.TimestepPredictedPosition.xz,
                math.max(state.UnconstrainedPredictedPosition.xz, state.PredictedPosition.xz)));
        if (SoftAvoidanceVelocitySolver !=
                SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle ||
            SoftAvoidanceShell <= 0f || SoftAvoidanceResponseRate <= 0f)
            return;

        float2 horizonEnd = state.PredictedPosition.xz +
                            state.BasePredictedVelocity.xz * math.max(0f, RvoTimeHorizon);
        pathMin = math.min(pathMin, horizonEnd);
        pathMax = math.max(pathMax, horizonEnd);
    }

    private bool AreCorrectedDiscsInsideFatCache(
        ref ShadowNeighborCacheStatistics statistics)
    {
        if (FatAabbCacheState.Value.IsValid == 0)
            return false;

        float neighborPadding = CalculateFatAabbNeighborPadding();
        statistics.CorrectedBodyValidationCount += CorrectedBodyIndices.Length;
        for (int i = 0; i < CorrectedBodyIndices.Length; i++)
        {
            int bodyIndex = CorrectedBodyIndices[i];
            if (AdaptiveFatAabbRequested &&
                AdaptiveBodyRouting[bodyIndex].IsFatParticipant == 0)
                continue;
            FlowMovementFrameState state = States[bodyIndex];
            if (!TryFindProxy(
                    ShadowPreviousProxies,
                    state.Entity,
                    out ShadowFatBodyProxy proxy))
                return false;

            float extent = math.max(0f, state.Radius) + neighborPadding;
            float2 currentMin = state.PredictedPosition.xz - extent;
            float2 currentMax = state.PredictedPosition.xz + extent;
            if (!AabbContains(proxy.FatMin, proxy.FatMax, currentMin, currentMax))
                return false;
        }

        return true;
    }

    private float CalculateFatAabbNeighborPadding()
    {
        // Global 路径已移除，始终使用 Adaptive 的 PredictiveSkin 作为 padding。
        return math.max(0f, PredictiveSkin);
    }

    private void ResetCorrectedBodyTracking()
    {
        for (int i = 0; i < CorrectedBodyIndices.Length; i++)
            CorrectedBodyFlags[CorrectedBodyIndices[i]] = 0;
        CorrectedBodyIndices.Clear();
    }

    private void MarkCorrectedBody(int bodyIndex)
    {
        if (CorrectedBodyFlags[bodyIndex] != 0)
            return;

        CorrectedBodyFlags[bodyIndex] = 1;
        CorrectedBodyIndices.Add(bodyIndex);
    }

    private void InvalidateFatAabbCache(
        ref ShadowNeighborCacheStatistics statistics,
        bool postSolve)
    {
        FatAabbCacheState cacheState = FatAabbCacheState.Value;
        if (cacheState.IsValid == 0)
            return;

        cacheState.IsValid = 0;
        cacheState.AgeFrames = 0;
        FatAabbCacheState.Value = cacheState;
        statistics.CacheInvalidationCount++;
        if (postSolve)
        {
            statistics.BoundsInvalidationCount++;
            statistics.PostSolveInvalidationCount++;
        }
    }

    private void BuildCurrentShadowCache(ref ShadowNeighborCacheStatistics statistics)
    {
        ShadowCellEntries.Clear();
        ShadowBodyPairs.Clear();
        ShadowCurrentProxies.Clear();
        ShadowCurrentPairs.Clear();

        float cellSize = math.max(CellRadius * 2f, 0.0001f);
        float extentMargin = CalculateFatAabbNeighborPadding() +
                             math.max(0f, FatAabbCacheMargin);
        int validBodyCount = 0;

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            var proxy = new ShadowFatBodyProxy
            {
                Entity = state.Entity,
                BodyIndex = bodyIndex
            };
            if (!state.IsInsideGrid)
            {
                ShadowCurrentProxies.Add(proxy);
                continue;
            }

            float extent = math.max(0f, state.Radius) + extentMargin;
            CalculateNeighborPathBounds(state, out float2 pathMin, out float2 pathMax);
            proxy.FatMin = pathMin - extent;
            proxy.FatMax = pathMax + extent;
            proxy.IsValid = 1;
            ShadowCurrentProxies.Add(proxy);
            validBodyCount++;

            int2 minCell = (int2)math.floor((proxy.FatMin - GridOrigin.xz) / cellSize);
            int2 maxCell = (int2)math.floor((proxy.FatMax - GridOrigin.xz) / cellSize);
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
                        BodyIndex = bodyIndex
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
        for (int i = 0; i < ShadowBodyPairs.Length; i++)
        {
            UnitCollisionPair bodyPair = ShadowBodyPairs[i];
            ShadowFatBodyProxy proxyA = ShadowCurrentProxies[bodyPair.BodyA];
            ShadowFatBodyProxy proxyB = ShadowCurrentProxies[bodyPair.BodyB];
            if (proxyA.IsValid == 0 || proxyB.IsValid == 0 ||
                !AabbOverlaps(proxyA.FatMin, proxyA.FatMax, proxyB.FatMin, proxyB.FatMax))
                continue;

            ShadowCurrentPairs.Add(ShadowEntityOrdering.CreatePair(proxyA.Entity, proxyB.Entity));
        }

        SortAndDeduplicateEntityPairs(ShadowCurrentPairs);
        ShadowCurrentProxies.AsArray().Sort(new ShadowFatBodyProxyComparer());
        statistics.CurrentFrameCacheBodyCount = validBodyCount;
        statistics.CurrentFrameCachePairCount = ShadowCurrentPairs.Length;
    }

    private static void SortAndDeduplicateBodyPairs(NativeList<UnitCollisionPair> pairs)
    {
        if (pairs.Length <= 1)
            return;

        pairs.AsArray().Sort(new UnitCollisionPairComparer());
        int writeIndex = 1;
        UnitCollisionPair previous = pairs[0];
        for (int readIndex = 1; readIndex < pairs.Length; readIndex++)
        {
            UnitCollisionPair current = pairs[readIndex];
            if (current.BodyA == previous.BodyA && current.BodyB == previous.BodyB)
                continue;
            pairs[writeIndex++] = current;
            previous = current;
        }
        pairs.ResizeUninitialized(writeIndex);
    }

    private static void SortAndDeduplicateEntityPairs(NativeList<ShadowEntityPair> pairs)
    {
        if (pairs.Length <= 1)
            return;

        pairs.AsArray().Sort(new ShadowEntityPairComparer());
        int writeIndex = 1;
        ShadowEntityPair previous = pairs[0];
        for (int readIndex = 1; readIndex < pairs.Length; readIndex++)
        {
            ShadowEntityPair current = pairs[readIndex];
            if (current.EntityA == previous.EntityA && current.EntityB == previous.EntityB)
                continue;
            pairs[writeIndex++] = current;
            previous = current;
        }
        pairs.ResizeUninitialized(writeIndex);
    }

    private void PromoteCurrentShadowCache()
    {
        ShadowPreviousProxies.Clear();
        ShadowPreviousPairs.Clear();
        ShadowPreviousProxies.AddRange(ShadowCurrentProxies.AsArray());
        ShadowPreviousPairs.AddRange(ShadowCurrentPairs.AsArray());
    }

    private static bool TryFindProxy(
        NativeList<ShadowFatBodyProxy> proxies,
        Entity entity,
        out ShadowFatBodyProxy proxy)
    {
        return TryFindProxyEntry(proxies, entity, out proxy) && proxy.IsValid != 0;
    }

    private static bool TryFindProxyEntry(
        NativeList<ShadowFatBodyProxy> proxies,
        Entity entity,
        out ShadowFatBodyProxy proxy)
    {
        int low = 0;
        int high = proxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            ShadowFatBodyProxy candidate = proxies[middle];
            int comparison = ShadowEntityOrdering.Compare(candidate.Entity, entity);
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

    private static bool ContainsPair(
        NativeList<ShadowEntityPair> pairs,
        ShadowEntityPair target)
    {
        int low = 0;
        int high = pairs.Length - 1;
        var comparer = new ShadowEntityPairComparer();
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            int comparison = comparer.Compare(pairs[middle], target);
            if (comparison == 0)
                return true;
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        return false;
    }

    private static bool AabbContains(
        float2 outerMin,
        float2 outerMax,
        float2 innerMin,
        float2 innerMax)
    {
        const float tolerance = 0.00001f;
        return math.all(innerMin >= outerMin - tolerance) &&
               math.all(innerMax <= outerMax + tolerance);
    }

    private static bool AabbOverlaps(
        float2 minA,
        float2 maxA,
        float2 minB,
        float2 maxB)
    {
        return math.all(maxA >= minB) && math.all(maxB >= minA);
    }
}
}
