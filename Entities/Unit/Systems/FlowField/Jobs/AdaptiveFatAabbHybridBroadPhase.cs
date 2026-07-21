using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct SolveXpbdUnitContactsJob
{
    /// <summary>
    /// 热点内部由跨 timestep Fat AABB 候选缓存负责；热点外部、Halo 和 Core 边界
    /// 仍进入普通 swept broad phase。最终只向 Narrow Phase 暴露一份去重后的 Pair 列表。
    /// </summary>
    private bool BuildAdaptiveHybridContactPairs(
        ref PredictiveDiscContactStatistics statistics,
        ref ShadowNeighborCacheStatistics cacheStatistics,
        ref bool fatCachePairsMappedThisFrame)
    {
        if (!EnsureAdaptiveFatAabbRawCandidates(
                ref cacheStatistics,
                ref fatCachePairsMappedThisFrame))
        {
            BuildSweptContactPairs(ref statistics);
            cacheStatistics.FullBroadPhaseFallbackCount++;
            return false;
        }

        BuildAdaptiveNormalRawPairs();
        Pairs.AddRange(MappedFatCachePairs.AsArray());
        SortAndDeduplicateBodyPairs(Pairs);

        if (EnableDiagnostics)
            PairDiagnostics.Clear();
        cacheStatistics.CacheUseCount++;
        cacheStatistics.CachedNarrowPhasePairCheckCount += MappedFatCachePairs.Length;
        statistics.CandidatePairCount += Pairs.Length;
        FilterAndClassifyPairs(ref statistics, math.max(0f, PredictiveSkin));
        return true;
    }

    private bool EnsureAdaptiveFatAabbRawCandidates(
        ref ShadowNeighborCacheStatistics cacheStatistics,
        ref bool fatCachePairsMappedThisFrame)
    {
        long validationStart = ProfilerUnsafeUtility.Timestamp;
        bool cacheValid = IsAdaptiveFatAabbCacheValid(
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
            BuildCurrentAdaptiveShadowCache(ref cacheStatistics);
            PromoteCurrentShadowCache();
            cacheStatistics.CacheBuildNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - buildStart);
            cacheStatistics.CacheRebuildCount++;

            FatAabbCacheState.Value = new FatAabbCacheState
            {
                IsValid = 1,
                AgeFrames = 0,
                PredictiveSkin = math.max(0f, PredictiveSkin),
                SoftAvoidanceShell = math.max(0f, SoftAvoidanceShell),
                SoftAvoidanceVelocitySolver = SoftAvoidanceVelocitySolver,
                RvoTimeHorizon = math.max(0f, RvoTimeHorizon),
                Margin = math.max(0f, FatAabbCacheMargin)
            };
            fatCachePairsMappedThisFrame = false;
        }
        else
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

        RefreshAdaptiveDebugProxiesFromCache();
        return true;
    }

    private void RefreshAdaptiveDebugProxiesFromCache()
    {
        AdaptiveDebugProxies.Clear();
        float neighborPadding = CalculateFatAabbNeighborPadding();
        for (int i = 0; i < ShadowPreviousProxies.Length; i++)
        {
            ShadowFatBodyProxy proxy = ShadowPreviousProxies[i];
            if (proxy.IsValid == 0 ||
                !TryFindCurrentBodyIndex(proxy.Entity, out int bodyIndex))
                continue;
            FlowMovementFrameState state = States[bodyIndex];
            CalculateCoreSweptBounds(
                state,
                neighborPadding,
                out float2 coreMin,
                out float2 coreMax);
            float minimumSlack = CalculateMinimumBoundsSlack(
                coreMin,
                coreMax,
                proxy.FatMin,
                proxy.FatMax);
            AdaptiveDebugProxies.Add(new AdaptiveFatAabbDebugProxy
            {
                Entity = proxy.Entity,
                CoreMin = coreMin,
                CoreMax = coreMax,
                FatMin = proxy.FatMin,
                FatMax = proxy.FatMax,
                MinimumSlack = minimumSlack,
                RegionIndex = AdaptiveBodyRouting[bodyIndex].FatRegionIndex,
                Escaped = (byte)(minimumSlack < 0f ? 1 : 0)
            });
        }
    }

    private bool IsAdaptiveFatAabbCacheValid(
        out bool entitySetInvalid,
        out bool boundsInvalid)
    {
        entitySetInvalid = false;
        boundsInvalid = false;
        FatAabbCacheState cacheState = FatAabbCacheState.Value;
        if (cacheState.IsValid == 0)
            return false;

        if (math.abs(cacheState.PredictiveSkin - math.max(0f, PredictiveSkin)) > 0.000001f ||
            math.abs(cacheState.Margin - math.max(0f, FatAabbCacheMargin)) > 0.000001f)
        {
            boundsInvalid = true;
            return false;
        }

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            if (AdaptiveBodyRouting[bodyIndex].IsFatParticipant == 0)
                continue;

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
        }

        // 反向校验：缓存中的每个 proxy 必须对应当前帧的 participant body，
        // 只用计数比较会因 participant set 变化频繁导致不必要的缓存失效。
        for (int i = 0; i < ShadowPreviousProxies.Length; i++)
        {
            ShadowFatBodyProxy proxy = ShadowPreviousProxies[i];
            if (!TryFindCurrentBodyIndex(proxy.Entity, out int cachedBodyIndex) ||
                AdaptiveBodyRouting[cachedBodyIndex].IsFatParticipant == 0)
            {
                entitySetInvalid = true;
                return false;
            }
        }
        return true;
    }

    private void BuildCurrentAdaptiveShadowCache(
        ref ShadowNeighborCacheStatistics statistics)
    {
        ShadowCellEntries.Clear();
        ShadowBodyPairs.Clear();
        ShadowCurrentProxies.Clear();
        ShadowCurrentPairs.Clear();
        AdaptiveDebugProxies.Clear();

        float cellSize = math.max(CellRadius * 2f, 0.0001f);
        float neighborPadding = CalculateFatAabbNeighborPadding();
        float extentMargin = neighborPadding + math.max(0f, FatAabbCacheMargin);
        int validBodyCount = 0;

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            AdaptiveFatAabbBodyRouting routing = AdaptiveBodyRouting[bodyIndex];
            if (routing.IsFatParticipant == 0)
                continue;

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

            CalculateNeighborPathBounds(state, out float2 pathMin, out float2 pathMax);
            float coreExtent = math.max(0f, state.Radius) + neighborPadding;
            float fatExtent = math.max(0f, state.Radius) + extentMargin;
            float2 coreMin = pathMin - coreExtent;
            float2 coreMax = pathMax + coreExtent;
            proxy.FatMin = pathMin - fatExtent;
            proxy.FatMax = pathMax + fatExtent;
            proxy.IsValid = 1;
            ShadowCurrentProxies.Add(proxy);
            validBodyCount++;

            float minimumSlack = CalculateMinimumBoundsSlack(
                coreMin,
                coreMax,
                proxy.FatMin,
                proxy.FatMax);
            AdaptiveDebugProxies.Add(new AdaptiveFatAabbDebugProxy
            {
                Entity = state.Entity,
                CoreMin = coreMin,
                CoreMax = coreMax,
                FatMin = proxy.FatMin,
                FatMax = proxy.FatMax,
                MinimumSlack = minimumSlack,
                RegionIndex = routing.FatRegionIndex,
                Escaped = (byte)(minimumSlack < 0f ? 1 : 0)
            });

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
                        CellIndex = FlowFieldUtils.GetFlatIndex(
                            new int2(x, y),
                            GridDimensions),
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
                    AdaptiveFatAabbBodyRouting routeA = AdaptiveBodyRouting[bodyA];
                    AdaptiveFatAabbBodyRouting routeB = AdaptiveBodyRouting[bodyB];
                    if (routeA.FatRegionIndex != routeB.FatRegionIndex)
                        continue;
                    if (routeA.IsCore == 0 && routeB.IsCore == 0)
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
            if (!TryFindCurrentProxy(bodyPair.BodyA, out ShadowFatBodyProxy proxyA) ||
                !TryFindCurrentProxy(bodyPair.BodyB, out ShadowFatBodyProxy proxyB) ||
                !AabbOverlaps(proxyA.FatMin, proxyA.FatMax, proxyB.FatMin, proxyB.FatMax))
                continue;
            ShadowCurrentPairs.Add(ShadowEntityOrdering.CreatePair(
                proxyA.Entity,
                proxyB.Entity));
        }

        SortAndDeduplicateEntityPairs(ShadowCurrentPairs);
        ShadowCurrentProxies.AsArray().Sort(new ShadowFatBodyProxyComparer());
        statistics.CurrentFrameCacheBodyCount = validBodyCount;
        statistics.CurrentFrameCachePairCount = ShadowCurrentPairs.Length;
    }

    private bool TryFindCurrentProxy(int bodyIndex, out ShadowFatBodyProxy proxy)
    {
        Entity entity = States[bodyIndex].Entity;
        for (int i = 0; i < ShadowCurrentProxies.Length; i++)
        {
            if (ShadowCurrentProxies[i].Entity != entity)
                continue;
            proxy = ShadowCurrentProxies[i];
            return proxy.IsValid != 0;
        }
        proxy = default;
        return false;
    }

    private void BuildAdaptiveNormalRawPairs()
    {
        SweptCellEntries.Clear();
        Pairs.Clear();
        float cellSize = math.max(CellRadius * 2f, 0.0001f);
        float skin = math.max(0f, PredictiveSkin);

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid || AdaptiveBodyRouting[bodyIndex].UseNormalBroadPhase == 0)
                continue;

            float extent = math.max(0f, state.Radius) + skin;
            float2 start = state.StartPosition.xz;
            float2 end = state.PredictedPosition.xz;
            float2 min = math.min(start, end) - extent;
            float2 max = math.max(start, end) + extent;
            int2 minCell = (int2)math.floor((min - GridOrigin.xz) / cellSize);
            int2 maxCell = (int2)math.floor((max - GridOrigin.xz) / cellSize);
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
                        CellIndex = FlowFieldUtils.GetFlatIndex(
                            new int2(x, y),
                            GridDimensions),
                        BodyIndex = bodyIndex
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
    }

    private void ResetAdaptiveFatAabbCacheWhenInactive()
    {
        if (HasActiveAdaptiveFatRegions)
            return;
        ShadowPreviousProxies.Clear();
        ShadowPreviousPairs.Clear();
        ShadowCurrentProxies.Clear();
        ShadowCurrentPairs.Clear();
        MappedFatCachePairs.Clear();
        AdaptiveDebugProxies.Clear();
        FatAabbCacheState.Value = default;
    }
    private static float CalculateMinimumBoundsSlack(
        float2 coreMin,
        float2 coreMax,
        float2 fatMin,
        float2 fatMax)
    {
        float2 lowerSlack = coreMin - fatMin;
        float2 upperSlack = fatMax - coreMax;
        return math.cmin(math.min(lowerSlack, upperSlack));
    }

}
}
