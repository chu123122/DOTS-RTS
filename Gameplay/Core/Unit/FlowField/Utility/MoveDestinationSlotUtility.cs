using System;
using System.Collections.Generic;
using Unity.Mathematics;

namespace RTS.Unit.FlowField
{

/// <summary>
/// 为一次本地移动订单生成固定槽位，并按单位原有相对编队分配槽位。
/// 所有方法只在订单到来时调用，不进入逐帧移动热路径。
/// </summary>
public static class MoveDestinationSlotUtility
{
    public static List<float3> GenerateWalkableSlots(
        float3 requestedTarget,
        int requiredCount,
        float spacing,
        in FlowFieldGrid grid)
    {
        var slots = new List<float3>(math.max(0, requiredCount));
        if (requiredCount <= 0)
            return slots;

        float safeSpacing = math.max(0.05f, spacing);
        int maxRing = CalculateMaximumRing(requiredCount, safeSpacing, grid);
        for (int ring = 0; ring <= maxRing && slots.Count < requiredCount; ring++)
        {
            for (int z = -ring; z <= ring && slots.Count < requiredCount; z++)
            {
                for (int x = -ring; x <= ring && slots.Count < requiredCount; x++)
                {
                    if (ring > 0 && math.max(math.abs(x), math.abs(z)) != ring)
                        continue;

                    float3 candidate = requestedTarget +
                                       new float3(x * safeSpacing, 0f, z * safeSpacing);
                    if (IsWalkable(candidate, grid))
                        slots.Add(candidate);
                }
            }
        }

        return slots;
    }

    public static int[] AssignSlotsPreservingFormation(
        IReadOnlyList<float3> unitPositions,
        IReadOnlyList<float3> slots,
        float3 formationTarget)
    {
        if (unitPositions.Count > slots.Count)
            throw new ArgumentException("可用槽位少于需要分配的单位数量。");

        var assignments = new int[unitPositions.Count];
        if (unitPositions.Count == 0)
            return assignments;

        float3 centroid = float3.zero;
        for (int i = 0; i < unitPositions.Count; i++)
            centroid += unitPositions[i];
        centroid /= unitPositions.Count;

        var unitOrder = new List<int>(unitPositions.Count);
        for (int i = 0; i < unitPositions.Count; i++)
            unitOrder.Add(i);
        unitOrder.Sort((a, b) =>
        {
            float distanceA = math.lengthsq(unitPositions[a].xz - centroid.xz);
            float distanceB = math.lengthsq(unitPositions[b].xz - centroid.xz);
            int distanceComparison = distanceB.CompareTo(distanceA);
            return distanceComparison != 0 ? distanceComparison : a.CompareTo(b);
        });

        var slotUsed = new bool[slots.Count];
        foreach (int unitIndex in unitOrder)
        {
            float2 preferredPosition =
                formationTarget.xz + (unitPositions[unitIndex].xz - centroid.xz);
            int bestSlot = -1;
            float bestDistanceSq = float.MaxValue;
            for (int slotIndex = 0; slotIndex < slots.Count; slotIndex++)
            {
                if (slotUsed[slotIndex])
                    continue;

                float distanceSq = math.lengthsq(slots[slotIndex].xz - preferredPosition);
                if (distanceSq < bestDistanceSq - 0.000001f ||
                    math.abs(distanceSq - bestDistanceSq) <= 0.000001f &&
                    slotIndex < bestSlot)
                {
                    bestDistanceSq = distanceSq;
                    bestSlot = slotIndex;
                }
            }

            assignments[unitIndex] = bestSlot;
            slotUsed[bestSlot] = true;
        }

        return assignments;
    }

    private static int CalculateMaximumRing(
        int requiredCount,
        float spacing,
        in FlowFieldGrid grid)
    {
        if (!grid.Grid.IsCreated)
            return math.max(1, (int)math.ceil(math.sqrt(requiredCount)) + 1);

        float cellSize = math.max(0.0001f, grid.CellRadius * 2f);
        float maximumWorldSpan = math.max(grid.GridDimensions.x, grid.GridDimensions.y) *
                                 cellSize;
        return math.max(1, (int)math.ceil(maximumWorldSpan / spacing) + 1);
    }

    private static bool IsWalkable(float3 position, in FlowFieldGrid grid)
    {
        if (!grid.Grid.IsCreated)
            return true;

        float cellSize = math.max(0.0001f, grid.CellRadius * 2f);
        float2 localPosition = position.xz - grid.GridOrigin.xz;
        int2 cell = (int2)math.floor(localPosition / cellSize);
        if (cell.x < 0 || cell.x >= grid.GridDimensions.x ||
            cell.y < 0 || cell.y >= grid.GridDimensions.y)
            return false;

        int index = FlowFieldUtils.GetFlatIndex(cell, grid.GridDimensions);
        return grid.Grid[index].Cost != 0;
    }
}
}
