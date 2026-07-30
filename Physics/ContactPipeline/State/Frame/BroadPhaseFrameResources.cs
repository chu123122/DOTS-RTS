using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>Full-sweep and persistent-reuse broad-phase worksets only.</summary>
internal struct BroadPhaseFrameResources
{
    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    public NativeList<byte> FullSweepBodyWorkset;
    public NativeList<byte> PersistentReusePairWorkset;
    public NativeArray<int> BodyCellCounts;
    public NativeArray<int> BodyCellOffsets;
    public NativeList<int> CellPairCounts;
    public NativeList<int> CellPairOffsets;
    public NativeList<byte> CellSortBlockWorkset;
    public NativeList<SweptDiscCellEntry> CellSortScratch;
    public NativeList<byte> PairSortBlockWorkset;
    public NativeList<ContactConstraint> PairSortScratch;
    public NativeList<PersistentSweptProxy> PreviousProxies;
    public NativeReference<byte> FullSweepPrepared;
    public NativeList<ContactConstraint> CollisionPairs;

    public static BroadPhaseFrameResources Create(int bodyCount)
    {
        int one = math.max(bodyCount, 1);
        return new BroadPhaseFrameResources
        {
            SweptCellEntries = new NativeList<SweptDiscCellEntry>(
                math.max(bodyCount * 4, 1), Allocator.TempJob),
            FullSweepBodyWorkset =
                new NativeList<byte>(one, Allocator.TempJob),
            PersistentReusePairWorkset =
                new NativeList<byte>(one, Allocator.TempJob),
            BodyCellCounts = new NativeArray<int>(
                bodyCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            BodyCellOffsets = new NativeArray<int>(
                bodyCount,
                Allocator.TempJob,
                NativeArrayOptions.UninitializedMemory),
            CellPairCounts = new NativeList<int>(
                math.max(bodyCount * 4, 1), Allocator.TempJob),
            CellPairOffsets = new NativeList<int>(
                math.max(bodyCount * 4, 1), Allocator.TempJob),
            CellSortBlockWorkset =
                new NativeList<byte>(one, Allocator.TempJob),
            CellSortScratch =
                new NativeList<SweptDiscCellEntry>(
                    math.max(bodyCount * 4, 1), Allocator.TempJob),
            PairSortBlockWorkset =
                new NativeList<byte>(one, Allocator.TempJob),
            PairSortScratch = new NativeList<ContactConstraint>(
                math.max(bodyCount * 4, 1), Allocator.TempJob),
            PreviousProxies =
                new NativeList<PersistentSweptProxy>(
                    one, Allocator.TempJob),
            FullSweepPrepared =
                new NativeReference<byte>(Allocator.TempJob),
            CollisionPairs = new NativeList<ContactConstraint>(
                math.max(bodyCount * 4, 1), Allocator.TempJob)
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = finalReader;
        combined = Combine(combined, SweptCellEntries.Dispose(finalReader));
        combined = Combine(
            combined, FullSweepBodyWorkset.Dispose(finalReader));
        combined = Combine(
            combined, PersistentReusePairWorkset.Dispose(finalReader));
        combined = Combine(combined, BodyCellCounts.Dispose(finalReader));
        combined = Combine(combined, BodyCellOffsets.Dispose(finalReader));
        combined = Combine(combined, CellPairCounts.Dispose(finalReader));
        combined = Combine(combined, CellPairOffsets.Dispose(finalReader));
        combined = Combine(
            combined, CellSortBlockWorkset.Dispose(finalReader));
        combined = Combine(combined, CellSortScratch.Dispose(finalReader));
        combined = Combine(
            combined, PairSortBlockWorkset.Dispose(finalReader));
        combined = Combine(combined, PairSortScratch.Dispose(finalReader));
        combined = Combine(combined, PreviousProxies.Dispose(finalReader));
        combined = Combine(combined, FullSweepPrepared.Dispose(finalReader));
        combined = Combine(combined, CollisionPairs.Dispose(finalReader));
        return combined;
    }

    private static JobHandle Combine(JobHandle a, JobHandle b) =>
        JobHandle.CombineDependencies(a, b);
}
}
