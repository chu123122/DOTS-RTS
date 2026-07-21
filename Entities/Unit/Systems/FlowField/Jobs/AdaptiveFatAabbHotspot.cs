using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct SolveXpbdUnitContactsJob
{
    private bool AdaptiveFatAabbRequested =>
        EnableFatAabbCache && AdaptiveSettings.Enabled != 0;

    private bool HasActiveAdaptiveFatRegions =>
        AdaptiveFatAabbRequested && AdaptiveRegions.Length > 0;

    private void BuildAdaptiveFatAabbHotspots()
    {
        AdaptiveRegions.Clear();
        AdaptiveFloodQueue.Clear();
        AdaptiveFloodCells.Clear();
        AdaptiveDebugCells.Clear();
        AdaptiveDebugRegions.Clear();
        AdaptiveDebugProxies.Clear();

        for (int i = 0; i < AdaptiveBodyRouting.Length; i++)
        {
            AdaptiveBodyRouting[i] = new AdaptiveFatAabbBodyRouting
            {
                CoreRegionIndex = -1,
                FatRegionIndex = -1,
                UseNormalBroadPhase = 1
            };
        }

        for (int i = 0; i < AdaptiveCellMetrics.Length; i++)
        {
            AdaptiveCellMetrics[i] = new AdaptiveFatAabbCellMetric
            {
                RegionIndex = -1
            };
        }

        if (!AdaptiveFatAabbRequested ||
            AdaptiveCellDimensions.x <= 0 || AdaptiveCellDimensions.y <= 0)
            return;

        int span = math.max(1, AdaptiveSettings.DetectionCellSpan);
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;

            int2 adaptiveCell = GetAdaptiveCell(state.CellPosition, span);
            int cellIndex = GetAdaptiveCellIndex(adaptiveCell);
            AdaptiveFatAabbCellMetric metric = AdaptiveCellMetrics[cellIndex];
            metric.UnitCount++;
            metric.SpeedSum += math.length(state.CurrentVelocity.xz);
            metric.OccupancyBloom |= EntityBloomBit(state.Entity);
            AdaptiveCellMetrics[cellIndex] = metric;
        }

        for (int cellIndex = 0; cellIndex < AdaptiveCellMetrics.Length; cellIndex++)
        {
            AdaptiveFatAabbCellMetric metric = AdaptiveCellMetrics[cellIndex];
            AdaptiveFatAabbCellHistory history = AdaptiveCellHistory[cellIndex];

            metric.DensityScore = math.saturate(
                metric.UnitCount / (float)math.max(1, AdaptiveSettings.MinimumUnitsPerCell));
            metric.PersistenceScore = BloomSimilarity(
                history.OccupancyBloom,
                metric.OccupancyBloom);
            metric.PressureScore = math.saturate(
                history.SmoothedCorrection /
                math.max(0.0001f, AdaptiveSettings.CorrectionReference));

            float averageSpeed = metric.UnitCount > 0
                ? metric.SpeedSum / metric.UnitCount
                : 0f;
            metric.EscapeRiskScore = math.saturate(
                averageSpeed /
                math.max(0.0001f, AdaptiveSettings.MaximumCacheableSpeed));

            float positiveWeight = math.max(
                0.0001f,
                AdaptiveSettings.DensityWeight +
                AdaptiveSettings.PersistenceWeight +
                AdaptiveSettings.PressureWeight);
            float positiveScore =
                AdaptiveSettings.DensityWeight * metric.DensityScore +
                AdaptiveSettings.PersistenceWeight * metric.PersistenceScore +
                AdaptiveSettings.PressureWeight * metric.PressureScore;
            float rawScore = positiveScore / positiveWeight -
                             AdaptiveSettings.EscapeRiskWeight * metric.EscapeRiskScore;
            rawScore = math.saturate(rawScore);
            metric.Score = math.lerp(
                history.SmoothedScore,
                rawScore,
                AdaptiveSettings.ScoreSmoothing);

            if (history.Active == 0)
            {
                history.EnableStreak = metric.Score >= AdaptiveSettings.EnableScore
                    ? SaturatingIncrement(history.EnableStreak)
                    : (ushort)0;
                history.DisableStreak = 0;
                if (history.EnableStreak >= AdaptiveSettings.EnableFrames)
                {
                    history.Active = 1;
                    history.EnableStreak = 0;
                }
            }
            else
            {
                history.DisableStreak = metric.Score <= AdaptiveSettings.DisableScore
                    ? SaturatingIncrement(history.DisableStreak)
                    : (ushort)0;
                history.EnableStreak = 0;
                if (history.DisableStreak >= AdaptiveSettings.DisableFrames)
                {
                    history.Active = 0;
                    history.DisableStreak = 0;
                }
            }

            metric.Active = history.Active;
            history.SmoothedScore = metric.Score;
            AdaptiveCellMetrics[cellIndex] = metric;
            AdaptiveCellHistory[cellIndex] = history;
        }

        BuildConnectedAdaptiveRegions();
        BuildAdaptiveBodyRouting();
        BuildAdaptiveDebugSnapshot();
    }

    private void BuildConnectedAdaptiveRegions()
    {
        int cellCount = AdaptiveCellMetrics.Length;
        for (int startIndex = 0; startIndex < cellCount; startIndex++)
        {
            AdaptiveFatAabbCellMetric startMetric = AdaptiveCellMetrics[startIndex];
            if (startMetric.Active == 0 || startMetric.RegionIndex >= 0)
                continue;

            int provisionalRegionIndex = AdaptiveRegions.Length;
            AdaptiveFloodQueue.Clear();
            AdaptiveFloodCells.Clear();
            AdaptiveFloodQueue.Add(startIndex);
            startMetric.RegionIndex = provisionalRegionIndex;
            AdaptiveCellMetrics[startIndex] = startMetric;

            int queueHead = 0;
            int unitCount = 0;
            float scoreSum = 0f;
            int2 firstCell = GetAdaptiveCellCoordinate(startIndex);
            int2 minCell = firstCell;
            int2 maxCell = firstCell;
            int stableId = startIndex;

            while (queueHead < AdaptiveFloodQueue.Length)
            {
                int cellIndex = AdaptiveFloodQueue[queueHead++];
                AdaptiveFloodCells.Add(cellIndex);
                AdaptiveFatAabbCellMetric metric = AdaptiveCellMetrics[cellIndex];
                int2 cell = GetAdaptiveCellCoordinate(cellIndex);
                unitCount += metric.UnitCount;
                scoreSum += metric.Score;
                minCell = math.min(minCell, cell);
                maxCell = math.max(maxCell, cell);
                stableId = math.min(stableId, cellIndex);

                TryQueueAdaptiveNeighbor(cell + new int2(-1, 0), provisionalRegionIndex);
                TryQueueAdaptiveNeighbor(cell + new int2(1, 0), provisionalRegionIndex);
                TryQueueAdaptiveNeighbor(cell + new int2(0, -1), provisionalRegionIndex);
                TryQueueAdaptiveNeighbor(cell + new int2(0, 1), provisionalRegionIndex);
            }

            if (unitCount < AdaptiveSettings.MinimumUnitsPerRegion)
            {
                for (int i = 0; i < AdaptiveFloodCells.Length; i++)
                {
                    int cellIndex = AdaptiveFloodCells[i];
                    AdaptiveFatAabbCellMetric metric = AdaptiveCellMetrics[cellIndex];
                    metric.RegionIndex = -1;
                    metric.Active = 0;
                    AdaptiveCellMetrics[cellIndex] = metric;
                }
                continue;
            }

            AdaptiveRegions.Add(new AdaptiveFatAabbRegion
            {
                StableId = stableId,
                MinCell = minCell,
                MaxCell = maxCell,
                UnitCount = unitCount,
                AverageScore = scoreSum / math.max(1, AdaptiveFloodCells.Length),
                Active = 1
            });
        }
    }

    private void TryQueueAdaptiveNeighbor(int2 cell, int regionIndex)
    {
        if (cell.x < 0 || cell.y < 0 ||
            cell.x >= AdaptiveCellDimensions.x ||
            cell.y >= AdaptiveCellDimensions.y)
            return;

        int index = GetAdaptiveCellIndex(cell);
        AdaptiveFatAabbCellMetric metric = AdaptiveCellMetrics[index];
        if (metric.Active == 0 || metric.RegionIndex >= 0)
            return;

        metric.RegionIndex = regionIndex;
        AdaptiveCellMetrics[index] = metric;
        AdaptiveFloodQueue.Add(index);
    }

    private void BuildAdaptiveBodyRouting()
    {
        int span = math.max(1, AdaptiveSettings.DetectionCellSpan);
        int halo = math.max(0, AdaptiveSettings.HaloCellCount);

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;

            int2 cell = GetAdaptiveCell(state.CellPosition, span);
            int cellIndex = GetAdaptiveCellIndex(cell);
            AdaptiveFatAabbCellMetric metric = AdaptiveCellMetrics[cellIndex];
            var routing = new AdaptiveFatAabbBodyRouting
            {
                CoreRegionIndex = metric.RegionIndex,
                FatRegionIndex = -1,
                IsCore = (byte)(metric.RegionIndex >= 0 ? 1 : 0),
                UseNormalBroadPhase = 1
            };

            if (metric.RegionIndex >= 0)
            {
                routing.FatRegionIndex = metric.RegionIndex;
                routing.IsFatParticipant = 1;
                routing.IsBoundary = (byte)(IsAdaptiveBoundaryCell(cell, metric.RegionIndex) ? 1 : 0);
            }
            else
            {
                for (int regionIndex = 0; regionIndex < AdaptiveRegions.Length; regionIndex++)
                {
                    AdaptiveFatAabbRegion region = AdaptiveRegions[regionIndex];
                    int2 haloMin = region.MinCell - halo;
                    int2 haloMax = region.MaxCell + halo;
                    if (math.any(cell < haloMin) || math.any(cell > haloMax))
                        continue;

                    routing.FatRegionIndex = regionIndex;
                    routing.IsFatParticipant = 1;
                    break;
                }
            }

            AdaptiveBodyRouting[bodyIndex] = routing;
        }
    }

    private bool IsAdaptiveBoundaryCell(int2 cell, int regionIndex)
    {
        return GetAdaptiveRegionIndex(cell + new int2(-1, 0)) != regionIndex ||
               GetAdaptiveRegionIndex(cell + new int2(1, 0)) != regionIndex ||
               GetAdaptiveRegionIndex(cell + new int2(0, -1)) != regionIndex ||
               GetAdaptiveRegionIndex(cell + new int2(0, 1)) != regionIndex;
    }

    private int GetAdaptiveRegionIndex(int2 cell)
    {
        if (cell.x < 0 || cell.y < 0 ||
            cell.x >= AdaptiveCellDimensions.x ||
            cell.y >= AdaptiveCellDimensions.y)
            return -1;
        return AdaptiveCellMetrics[GetAdaptiveCellIndex(cell)].RegionIndex;
    }

    private void BuildAdaptiveDebugSnapshot()
    {
        float worldCellSize = CellRadius * 2f * math.max(1, AdaptiveSettings.DetectionCellSpan);
        float2 origin = GridOrigin.xz;

        for (int cellIndex = 0; cellIndex < AdaptiveCellMetrics.Length; cellIndex++)
        {
            AdaptiveFatAabbCellMetric metric = AdaptiveCellMetrics[cellIndex];
            if (metric.UnitCount == 0 && metric.Active == 0)
                continue;

            int2 cell = GetAdaptiveCellCoordinate(cellIndex);
            float2 min = origin + (float2)cell * worldCellSize;
            AdaptiveDebugCells.Add(new AdaptiveFatAabbDebugCell
            {
                Min = min,
                Max = min + worldCellSize,
                Score = metric.Score,
                UnitCount = metric.UnitCount,
                Active = metric.Active
            });
        }

        int halo = math.max(0, AdaptiveSettings.HaloCellCount);
        for (int regionIndex = 0; regionIndex < AdaptiveRegions.Length; regionIndex++)
        {
            AdaptiveFatAabbRegion region = AdaptiveRegions[regionIndex];
            float2 coreMin = origin + (float2)region.MinCell * worldCellSize;
            float2 coreMax = origin + (float2)(region.MaxCell + 1) * worldCellSize;
            float2 haloMin = origin + (float2)(region.MinCell - halo) * worldCellSize;
            float2 haloMax = origin + (float2)(region.MaxCell + 1 + halo) * worldCellSize;
            AdaptiveDebugRegions.Add(new AdaptiveFatAabbDebugRegion
            {
                CoreMin = coreMin,
                CoreMax = coreMax,
                HaloMin = haloMin,
                HaloMax = haloMax,
                Score = region.AverageScore,
                StableId = region.StableId,
                Active = region.Active
            });
        }
    }

    private void UpdateAdaptiveFatAabbHistoryAfterSolve(
        ref ShadowNeighborCacheStatistics shadowStatistics)
    {
        if (!AdaptiveFatAabbRequested)
            return;

        for (int cellIndex = 0; cellIndex < AdaptiveCellMetrics.Length; cellIndex++)
        {
            AdaptiveFatAabbCellMetric metric = AdaptiveCellMetrics[cellIndex];
            metric.CorrectionSum = 0f;
            AdaptiveCellMetrics[cellIndex] = metric;
        }

        int span = math.max(1, AdaptiveSettings.DetectionCellSpan);
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;
            int cellIndex = GetAdaptiveCellIndex(GetAdaptiveCell(state.CellPosition, span));
            AdaptiveFatAabbCellMetric metric = AdaptiveCellMetrics[cellIndex];
            metric.CorrectionSum += math.length(
                state.ContactPositionCorrection.xz + state.WallPositionCorrection.xz);
            AdaptiveCellMetrics[cellIndex] = metric;
        }

        float escapePenalty = math.saturate(
            shadowStatistics.FullBroadPhaseFallbackCount +
            shadowStatistics.PostSolveInvalidationCount);
        for (int cellIndex = 0; cellIndex < AdaptiveCellMetrics.Length; cellIndex++)
        {
            AdaptiveFatAabbCellMetric metric = AdaptiveCellMetrics[cellIndex];
            AdaptiveFatAabbCellHistory history = AdaptiveCellHistory[cellIndex];
            float averageCorrection = metric.UnitCount > 0
                ? metric.CorrectionSum / metric.UnitCount
                : 0f;
            history.SmoothedCorrection = math.lerp(
                history.SmoothedCorrection,
                averageCorrection,
                AdaptiveSettings.ScoreSmoothing);
            history.SmoothedEscapePenalty = math.lerp(
                history.SmoothedEscapePenalty,
                metric.Active != 0 ? escapePenalty : 0f,
                AdaptiveSettings.ScoreSmoothing);
            history.OccupancyBloom = metric.OccupancyBloom;
            AdaptiveCellHistory[cellIndex] = history;
        }
    }

    private int2 GetAdaptiveCell(int2 flowCell, int span)
    {
        return math.clamp(
            flowCell / math.max(1, span),
            int2.zero,
            AdaptiveCellDimensions - 1);
    }

    private int GetAdaptiveCellIndex(int2 cell)
    {
        return cell.x * AdaptiveCellDimensions.y + cell.y;
    }

    private int2 GetAdaptiveCellCoordinate(int index)
    {
        return new int2(index / AdaptiveCellDimensions.y, index % AdaptiveCellDimensions.y);
    }

    private static ulong EntityBloomBit(Entity entity)
    {
        uint hash = math.hash(new uint2((uint)entity.Index, (uint)entity.Version));
        return 1ul << (int)(hash & 63u);
    }

    private static float BloomSimilarity(ulong previous, ulong current)
    {
        ulong union = previous | current;
        if (union == 0ul)
            return 0f;
        ulong intersection = previous & current;
        return CountBits(intersection) / (float)math.max(1, CountBits(union));
    }

    private static int CountBits(ulong value)
    {
        return math.countbits((uint)value) + math.countbits((uint)(value >> 32));
    }

    private static ushort SaturatingIncrement(ushort value)
    {
        return value == ushort.MaxValue ? value : (ushort)(value + 1);
    }
}
}
