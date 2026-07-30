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
    public NativeArray<byte> DirtyFlagsByBody;
    public NativeList<PersistentNeighborPair> NeighborPairScratch;
    public NativeArray<DirtyBodyRefreshResult> BodyRefreshResults;
    public NativeReference<DirtyBodyRefreshSummary> BodyRefreshSummary;
    public NativeList<byte> PersistentIncidentPairWorkset;
    public NativeReference<int> PersistentIncidentRebuildPairCount;
    public NativeList<ContactViewCandidate> ContactViewCandidates;
    public NativeList<ContactViewCandidate> ContactViewSortScratch;
    public NativeList<byte> ContactViewCandidateWorkset;
    public NativeList<ContactViewPublicationBlock>
        ContactViewPublicationBlocks;
    public NativeList<byte> ContactViewBlockWorkset;
    public NativeReference<int> ContactViewRequiredMergePassCount;

    public static ContactRepairFrameResources Create(int bodyCount)
    {
        int one = math.max(bodyCount, 1);
        int contactCapacity = math.max(bodyCount * 8, 1);
        return new ContactRepairFrameResources
        {
            DirtyBodies =
                new NativeList<IncrementalDirtyBody>(
                    one, Allocator.TempJob),
            DirtyFlagsByBody = new NativeArray<byte>(
                bodyCount,
                Allocator.TempJob,
                NativeArrayOptions.ClearMemory),
            NeighborPairScratch =
                new NativeList<PersistentNeighborPair>(
                    contactCapacity, Allocator.TempJob),
            BodyRefreshResults = new NativeArray<DirtyBodyRefreshResult>(
                bodyCount,
                Allocator.TempJob,
                NativeArrayOptions.ClearMemory),
            BodyRefreshSummary =
                new NativeReference<DirtyBodyRefreshSummary>(
                    Allocator.TempJob),
            PersistentIncidentPairWorkset =
                new NativeList<byte>(one, Allocator.TempJob),
            PersistentIncidentRebuildPairCount =
                new NativeReference<int>(Allocator.TempJob),
            ContactViewCandidates =
                new NativeList<ContactViewCandidate>(
                    contactCapacity, Allocator.TempJob),
            ContactViewSortScratch =
                new NativeList<ContactViewCandidate>(
                    contactCapacity, Allocator.TempJob),
            ContactViewCandidateWorkset =
                new NativeList<byte>(
                    contactCapacity, Allocator.TempJob),
            ContactViewPublicationBlocks =
                new NativeList<ContactViewPublicationBlock>(
                    one, Allocator.TempJob),
            ContactViewBlockWorkset =
                new NativeList<byte>(one, Allocator.TempJob),
            ContactViewRequiredMergePassCount =
                new NativeReference<int>(Allocator.TempJob)
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = DirtyBodies.Dispose(finalReader);
        combined = JobHandle.CombineDependencies(
            combined, DirtyFlagsByBody.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, NeighborPairScratch.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, BodyRefreshResults.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, BodyRefreshSummary.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, PersistentIncidentPairWorkset.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, PersistentIncidentRebuildPairCount.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ContactViewCandidates.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ContactViewSortScratch.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ContactViewCandidateWorkset.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ContactViewPublicationBlocks.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, ContactViewBlockWorkset.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined,
            ContactViewRequiredMergePassCount.Dispose(finalReader));
        return combined;
    }
}
}
