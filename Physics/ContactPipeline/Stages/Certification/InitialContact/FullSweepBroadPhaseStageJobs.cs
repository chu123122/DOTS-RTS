using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
[BurstCompile]
internal struct CountBodyCellsJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    public NativeArray<int> BodyCellCounts;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    public void Execute(int bodyIndex)
    {
        CrowdBodySnapshot body = Bodies[bodyIndex];
        if (body.IsInsideSimulationDomain == 0)
        {
            BodyCellCounts[bodyIndex] = 0;
            return;
        }
        float cellSize = math.max(CellRadius * 2f, 0.0001f);
        CrowdMotionEvidence evidence = MotionEvidence[bodyIndex];
        int2 minCell = (int2)math.floor(
            (evidence.InteractionEnvelopeMin - GridOrigin.xz) / cellSize);
        int2 maxCell = (int2)math.floor(
            (evidence.InteractionEnvelopeMax - GridOrigin.xz) / cellSize);
        if (maxCell.x < 0 || maxCell.y < 0 ||
            minCell.x >= GridDimensions.x || minCell.y >= GridDimensions.y)
        {
            BodyCellCounts[bodyIndex] = 0;
            return;
        }
        minCell = math.clamp(minCell, int2.zero, GridDimensions - 1);
        maxCell = math.clamp(maxCell, int2.zero, GridDimensions - 1);
        BodyCellCounts[bodyIndex] =
            (maxCell.x - minCell.x + 1) * (maxCell.y - minCell.y + 1);
    }
}

[BurstCompile]
internal struct PrefixBodyCellsJob : IJob
{
    [ReadOnly] public NativeArray<int> BodyCellCounts;
    public NativeArray<int> BodyCellOffsets;
    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    public NativeList<int> CellPairCounts;
    public NativeList<int> CellPairOffsets;

    public void Execute()
    {
        int offset = 0;
        for (int bodyIndex = 0; bodyIndex < BodyCellCounts.Length; bodyIndex++)
        {
            BodyCellOffsets[bodyIndex] = offset;
            offset += BodyCellCounts[bodyIndex];
        }
        SweptCellEntries.ResizeUninitialized(offset);
        CellPairCounts.ResizeUninitialized(offset);
        CellPairOffsets.ResizeUninitialized(offset);
    }
}

[BurstCompile]
internal struct ScatterBodyCellsJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<int> BodyCellCounts;
    [ReadOnly] public NativeArray<int> BodyCellOffsets;
    [NativeDisableParallelForRestriction]
    public NativeArray<SweptDiscCellEntry> SweptCellEntries;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    public void Execute(int bodyIndex)
    {
        if (BodyCellCounts[bodyIndex] == 0)
            return;
        float cellSize = math.max(CellRadius * 2f, 0.0001f);
        CrowdMotionEvidence evidence = MotionEvidence[bodyIndex];
        int2 minCell = (int2)math.floor(
            (evidence.InteractionEnvelopeMin - GridOrigin.xz) / cellSize);
        int2 maxCell = (int2)math.floor(
            (evidence.InteractionEnvelopeMax - GridOrigin.xz) / cellSize);
        minCell = math.clamp(minCell, int2.zero, GridDimensions - 1);
        maxCell = math.clamp(maxCell, int2.zero, GridDimensions - 1);
        int writeIndex = BodyCellOffsets[bodyIndex];
        for (int x = minCell.x; x <= maxCell.x; x++)
        {
            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                SweptCellEntries[writeIndex++] = new SweptDiscCellEntry
                {
                    CellIndex = FlowFieldUtils.GetFlatIndex(
                        new int2(x, y), GridDimensions),
                    BodyIndex = bodyIndex
                };
            }
        }
    }
}

[BurstCompile]
internal struct SortBodyCellsJob : IJob
{
    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    public void Execute()
    {
        if (SweptCellEntries.Length > 1)
            SweptCellEntries.AsArray().Sort(new SweptDiscCellEntryComparer());
    }
}

[BurstCompile]
internal struct CountCellPairsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<SweptDiscCellEntry> SweptCellEntries;
    public NativeArray<int> CellPairCounts;

    public void Execute(int entryIndex)
    {
        int cellIndex = SweptCellEntries[entryIndex].CellIndex;
        if (entryIndex > 0 &&
            SweptCellEntries[entryIndex - 1].CellIndex == cellIndex)
        {
            CellPairCounts[entryIndex] = 0;
            return;
        }
        int cellEnd = entryIndex + 1;
        while (cellEnd < SweptCellEntries.Length &&
               SweptCellEntries[cellEnd].CellIndex == cellIndex)
            cellEnd++;
        int bodyCount = cellEnd - entryIndex;
        CellPairCounts[entryIndex] = bodyCount * (bodyCount - 1) / 2;
    }
}

[BurstCompile]
internal struct PrefixCellPairsJob : IJob
{
    [ReadOnly] public NativeArray<int> CellPairCounts;
    public NativeArray<int> CellPairOffsets;
    public NativeList<ContactConstraint> Pairs;

    public void Execute()
    {
        int offset = 0;
        for (int entryIndex = 0; entryIndex < CellPairCounts.Length; entryIndex++)
        {
            CellPairOffsets[entryIndex] = offset;
            offset += CellPairCounts[entryIndex];
        }
        Pairs.ResizeUninitialized(offset);
    }
}

[BurstCompile]
internal struct ScatterCellPairsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<SweptDiscCellEntry> SweptCellEntries;
    [ReadOnly] public NativeArray<int> CellPairCounts;
    [ReadOnly] public NativeArray<int> CellPairOffsets;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> Pairs;

    public void Execute(int entryIndex)
    {
        if (CellPairCounts[entryIndex] == 0)
            return;
        int cellIndex = SweptCellEntries[entryIndex].CellIndex;
        int cellEnd = entryIndex + 1;
        while (cellEnd < SweptCellEntries.Length &&
               SweptCellEntries[cellEnd].CellIndex == cellIndex)
            cellEnd++;
        int writeIndex = CellPairOffsets[entryIndex];
        for (int first = entryIndex; first < cellEnd; first++)
        {
            int bodyA = SweptCellEntries[first].BodyIndex;
            for (int second = first + 1; second < cellEnd; second++)
            {
                int bodyB = SweptCellEntries[second].BodyIndex;
                Pairs[writeIndex++] = new ContactConstraint
                {
                    BodyA = math.min(bodyA, bodyB),
                    BodyB = math.max(bodyA, bodyB)
                };
            }
        }
    }
}

[BurstCompile]
internal struct SortAndDeduplicateBroadPhasePairsJob : IJob
{
    public NativeList<ContactConstraint> Pairs;
    public NativeList<BodyPair> TimestepInteractionPairs;
    public NativeReference<byte> FullSweepPrepared;

    public void Execute()
    {
        if (Pairs.Length > 1)
        {
            Pairs.AsArray().Sort(new ContactConstraintComparer());
            int writeIndex = 1;
            ContactConstraint previous = Pairs[0];
            for (int readIndex = 1; readIndex < Pairs.Length; readIndex++)
            {
                ContactConstraint current = Pairs[readIndex];
                if (current.BodyA == previous.BodyA &&
                    current.BodyB == previous.BodyB)
                    continue;
                Pairs[writeIndex++] = current;
                previous = current;
            }
            Pairs.ResizeUninitialized(writeIndex);
        }
        TimestepInteractionPairs.Clear();
        ContactPipelineShared.CopyConstraintsToBodyPairs(
            Pairs.AsArray(), TimestepInteractionPairs);
        FullSweepPrepared.Value = 1;
    }
}
}
