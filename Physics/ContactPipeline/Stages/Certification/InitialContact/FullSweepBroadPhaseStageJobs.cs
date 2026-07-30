using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
[BurstCompile]
internal struct PrepareFullSweepBroadPhaseJob : IJob
{
    [ReadOnly] public NativeList<IncrementalDirtyBody> DirtyBodies;
    public NativeReference<byte> FullSweepPrepared;
    public NativeList<byte> BodyWorkset;
    [ReadOnly] public NativeReference<ContactPipelineExecutionState> RuntimeState;
    [ReadOnly] public NativeReference<IncrementalContactCacheState> CacheState;
    [ReadOnly] public NativeList<PersistentSweptProxy> PersistentProxies;
    [ReadOnly] public NativeList<int> PersistentProxyIndexByBody;
    public ContactPipelineConfiguration Configuration;
    public int BodyCount;
    public byte RequireDirtyBodies;
    public byte RequireValidPersistentCache;

    public void Execute()
    {
        IncrementalContactCacheState cacheState = CacheState.Value;
        bool persistentCacheInvalid =
            RequireValidPersistentCache != 0 &&
            (cacheState.ContactViewsValid == 0 ||
             !PersistentCacheReusability.IsStructurallyReusable(
                 cacheState,
                 BodyCount,
                 PersistentProxies.Length,
                 PersistentProxyIndexByBody.Length,
                 Configuration));
        byte prepared = (byte)(
            RuntimeState.Value.IsValid != 0 &&
            (RequireDirtyBodies == 0 ||
             DirtyBodies.Length != 0 ||
             persistentCacheInvalid)
                ? 1
                : 0);
        FullSweepPrepared.Value = prepared;
        BodyWorkset.ResizeUninitialized(prepared != 0 ? BodyCount : 0);
    }
}

[BurstCompile]
internal struct CountBodyCellsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
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
    [ReadOnly] public NativeReference<byte> FullSweepPrepared;
    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    public NativeList<int> CellPairCounts;
    public NativeList<int> CellPairOffsets;

    public void Execute()
    {
        if (FullSweepPrepared.Value == 0)
        {
            SweptCellEntries.Clear();
            CellPairCounts.Clear();
            CellPairOffsets.Clear();
            return;
        }
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
internal struct ScatterBodyCellsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
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
                    CellIndex = FlowGridGeometry.FlatIndex(
                        new int2(x, y), GridDimensions),
                    BodyIndex = bodyIndex
                };
            }
        }
    }
}

[BurstCompile]
internal struct PrepareBodyCellSortJob : IJob
{
    [ReadOnly] public NativeList<SweptDiscCellEntry> Entries;
    public NativeList<byte> BlockWorkset;
    public NativeList<SweptDiscCellEntry> Scratch;
    public int BlockSize;

    public void Execute()
    {
        int blockCount = (Entries.Length + BlockSize - 1) / BlockSize;
        BlockWorkset.ResizeUninitialized(blockCount);
        Scratch.ResizeUninitialized(Entries.Length);
    }
}

[BurstCompile]
internal struct SortBodyCellBlocksJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [NativeDisableParallelForRestriction]
    public NativeArray<SweptDiscCellEntry> Entries;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        int start = blockIndex * BlockSize;
        int length = math.min(BlockSize, Entries.Length - start);
        if (length > 1)
        {
            Entries.GetSubArray(start, length).Sort(
                new SweptDiscCellEntryComparer());
        }
    }
}

[BurstCompile]
internal struct MergeBodyCellBlocksJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<SweptDiscCellEntry> Source;
    [NativeDisableParallelForRestriction]
    public NativeArray<SweptDiscCellEntry> Destination;
    public int BlockSize;
    public int MergePass;

    public void Execute(int blockIndex)
    {
        int sourceBlocksPerGroup = 1 << MergePass;
        int destinationBlocksPerGroup = sourceBlocksPerGroup << 1;
        if (blockIndex % destinationBlocksPerGroup != 0)
            return;
        int start = blockIndex * BlockSize;
        int middle = math.min(
            start + sourceBlocksPerGroup * BlockSize,
            Source.Length);
        int end = math.min(
            start + destinationBlocksPerGroup * BlockSize,
            Source.Length);
        int left = start;
        int right = middle;
        int write = start;
        var comparer = new SweptDiscCellEntryComparer();
        while (left < middle && right < end)
        {
            SweptDiscCellEntry a = Source[left];
            SweptDiscCellEntry b = Source[right];
            if (comparer.Compare(a, b) <= 0)
            {
                Destination[write++] = a;
                left++;
            }
            else
            {
                Destination[write++] = b;
                right++;
            }
        }
        while (left < middle)
            Destination[write++] = Source[left++];
        while (right < end)
            Destination[write++] = Source[right++];
    }
}

[BurstCompile]
internal struct CopyBodyCellSortResultJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<SweptDiscCellEntry> Source;
    [NativeDisableParallelForRestriction]
    public NativeArray<SweptDiscCellEntry> Destination;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        int start = blockIndex * BlockSize;
        int end = math.min(start + BlockSize, Source.Length);
        for (int index = start; index < end; index++)
            Destination[index] = Source[index];
    }
}

[BurstCompile]
internal struct CountCellPairsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<SweptDiscCellEntry> SweptCellEntries;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    public NativeArray<int> CellPairCounts;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

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
        int pairCount = 0;
        for (int first = entryIndex; first < cellEnd; first++)
        {
            int bodyA = SweptCellEntries[first].BodyIndex;
            for (int second = first + 1; second < cellEnd; second++)
            {
                int bodyB = SweptCellEntries[second].BodyIndex;
                if (FullSweepBroadPhaseMath.IsCanonicalSharedCell(
                        cellIndex,
                        MotionEvidence[bodyA],
                        MotionEvidence[bodyB],
                        GridOrigin,
                        GridDimensions,
                        CellRadius))
                    pairCount++;
            }
        }
        CellPairCounts[entryIndex] = pairCount;
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
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<int> CellPairCounts;
    [ReadOnly] public NativeArray<int> CellPairOffsets;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> Pairs;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

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
                if (!FullSweepBroadPhaseMath.IsCanonicalSharedCell(
                        cellIndex,
                        MotionEvidence[bodyA],
                        MotionEvidence[bodyB],
                        GridOrigin,
                        GridDimensions,
                        CellRadius))
                    continue;
                Pairs[writeIndex++] = new ContactConstraint
                {
                    Definition = new ContactConstraintDefinition
                    {
                        BodyA = math.min(bodyA, bodyB),
                        BodyB = math.max(bodyA, bodyB)
                    }
                };
            }
        }
    }
}

[BurstCompile]
internal struct DeduplicateAndPublishBroadPhasePairsJob : IJob
{
    public NativeList<ContactConstraint> Pairs;
    public NativeList<BodyPair> TimestepInteractionPairs;
    public NativeReference<byte> FullSweepPrepared;
    public NativeReference<ContactPipelineExecutionState> RuntimeState;

    public void Execute()
    {
        if (FullSweepPrepared.Value == 0)
            return;
        for (int pairIndex = 0; pairIndex < Pairs.Length; pairIndex++)
        {
            ContactConstraint pair = Pairs[pairIndex];
            if (pair.BodyA >= 0 && pair.BodyB >= 0)
                continue;

            ContactPipelineExecutionState runtime = RuntimeState.Value;
            runtime.IsValid = 0;
            runtime.RecoveryRequired = 1;
            RuntimeState.Value = runtime;
            Pairs.Clear();
            TimestepInteractionPairs.Clear();
            FullSweepPrepared.Value = 0;
            return;
        }
        if (Pairs.Length > 1)
        {
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

internal static class FullSweepBroadPhaseMath
{
    internal static bool IsCanonicalSharedCell(
        int cellIndex,
        CrowdMotionEvidence evidenceA,
        CrowdMotionEvidence evidenceB,
        float3 gridOrigin,
        int2 gridDimensions,
        float cellRadius)
    {
        float cellSize = math.max(cellRadius * 2f, 0.0001f);
        int2 minA = (int2)math.floor(
            (evidenceA.InteractionEnvelopeMin - gridOrigin.xz) / cellSize);
        int2 minB = (int2)math.floor(
            (evidenceB.InteractionEnvelopeMin - gridOrigin.xz) / cellSize);
        minA = math.clamp(minA, int2.zero, gridDimensions - 1);
        minB = math.clamp(minB, int2.zero, gridDimensions - 1);
        int2 canonicalCell = math.max(minA, minB);
        return cellIndex ==
               FlowGridGeometry.FlatIndex(canonicalCell, gridDimensions);
    }
}

[BurstCompile]
internal struct PrepareBroadPhasePairSortJob : IJob
{
    [ReadOnly] public NativeList<ContactConstraint> Pairs;
    public NativeList<byte> BlockWorkset;
    public NativeList<ContactConstraint> Scratch;
    public int BlockSize;

    public void Execute()
    {
        int blockCount =
            (Pairs.Length + BlockSize - 1) / BlockSize;
        BlockWorkset.ResizeUninitialized(blockCount);
        Scratch.ResizeUninitialized(Pairs.Length);
    }
}

[BurstCompile]
internal struct SortBroadPhasePairBlocksJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> Pairs;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        int start = blockIndex * BlockSize;
        int length = math.min(BlockSize, Pairs.Length - start);
        if (length > 1)
        {
            Pairs.GetSubArray(start, length).Sort(
                new ContactConstraintComparer());
        }
    }
}

[BurstCompile]
internal struct MergeBroadPhasePairBlocksJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactConstraint> Source;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> Destination;
    public int BlockSize;
    public int MergePass;

    public void Execute(int blockIndex)
    {
        int sourceBlocksPerGroup = 1 << MergePass;
        int destinationBlocksPerGroup = sourceBlocksPerGroup << 1;
        if (blockIndex % destinationBlocksPerGroup != 0)
            return;

        int start = blockIndex * BlockSize;
        int middle = math.min(
            start + sourceBlocksPerGroup * BlockSize,
            Source.Length);
        int end = math.min(
            start + destinationBlocksPerGroup * BlockSize,
            Source.Length);
        int left = start;
        int right = middle;
        int write = start;
        var comparer = new ContactConstraintComparer();
        while (left < middle && right < end)
        {
            ContactConstraint a = Source[left];
            ContactConstraint b = Source[right];
            if (comparer.Compare(a, b) <= 0)
            {
                Destination[write++] = a;
                left++;
            }
            else
            {
                Destination[write++] = b;
                right++;
            }
        }
        while (left < middle)
            Destination[write++] = Source[left++];
        while (right < end)
            Destination[write++] = Source[right++];
    }
}

[BurstCompile]
internal struct CopyBroadPhasePairSortResultJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactConstraint> Source;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> Destination;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        int start = blockIndex * BlockSize;
        int end = math.min(start + BlockSize, Source.Length);
        for (int index = start; index < end; index++)
            Destination[index] = Source[index];
    }
}
}
