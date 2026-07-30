using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>Dirty detection, refresh and repair worksets only.</summary>
internal struct ContactRepairFrameResources
{
    public NativeList<IncrementalDirtyBody> DirtyBodies;
    public NativeList<PersistentPredictiveContact>
        PersistentContactCompactionScratch;
    public NativeArray<byte> DirtyFlagsByBody;
    public NativeList<PersistentNeighborPair> NeighborPairScratch;
    public NativeArray<DirtyBodyRefreshResult> BodyRefreshResults;
    public NativeReference<DirtyBodyRefreshSummary> BodyRefreshSummary;
    public NativeList<DirtyContactScheduleBlock> ScheduleBlockCounts;
    public NativeList<DirtyContactScheduleBlock> ScheduleBlockOffsets;
    public NativeList<byte> PersistentIncidentPairWorkset;
    public NativeReference<int> PersistentIncidentRebuildPairCount;

    public static ContactRepairFrameResources Create(int bodyCount)
    {
        int one = math.max(bodyCount, 1);
        return new ContactRepairFrameResources
        {
            DirtyBodies =
                new NativeList<IncrementalDirtyBody>(
                    one, Allocator.TempJob),
            PersistentContactCompactionScratch =
                new NativeList<PersistentPredictiveContact>(
                    math.max(bodyCount * 4, 1), Allocator.TempJob),
            DirtyFlagsByBody = new NativeArray<byte>(
                bodyCount,
                Allocator.TempJob,
                NativeArrayOptions.ClearMemory),
            NeighborPairScratch =
                new NativeList<PersistentNeighborPair>(
                    math.max(bodyCount * 8, 1), Allocator.TempJob),
            BodyRefreshResults = new NativeArray<DirtyBodyRefreshResult>(
                bodyCount,
                Allocator.TempJob,
                NativeArrayOptions.ClearMemory),
            BodyRefreshSummary =
                new NativeReference<DirtyBodyRefreshSummary>(
                    Allocator.TempJob),
            ScheduleBlockCounts =
                new NativeList<DirtyContactScheduleBlock>(
                    one, Allocator.TempJob),
            ScheduleBlockOffsets =
                new NativeList<DirtyContactScheduleBlock>(
                    one, Allocator.TempJob),
            PersistentIncidentPairWorkset =
                new NativeList<byte>(one, Allocator.TempJob),
            PersistentIncidentRebuildPairCount =
                new NativeReference<int>(Allocator.TempJob)
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = DirtyBodies.Dispose(finalReader);
        combined = JobHandle.CombineDependencies(
            combined,
            PersistentContactCompactionScratch.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, DirtyFlagsByBody.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, NeighborPairScratch.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, BodyRefreshResults.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, BodyRefreshSummary.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ScheduleBlockCounts.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ScheduleBlockOffsets.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, PersistentIncidentPairWorkset.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, PersistentIncidentRebuildPairCount.Dispose(finalReader));
        return combined;
    }
}
}
