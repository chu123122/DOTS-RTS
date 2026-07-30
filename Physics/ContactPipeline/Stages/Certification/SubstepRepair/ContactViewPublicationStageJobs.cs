using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
[BurstCompile]
internal struct PrepareRepairContactViewPublicationJob : IJob
{
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;
    [ReadOnly] public NativeReference<PersistentClassificationPhaseState>
        PhaseState;
    [ReadOnly] public NativeList<ContactConstraint> PreviousContacts;
    [ReadOnly] public NativeList<ContactConstraint> NewContacts;
    public NativeList<ContactViewCandidate> Candidates;
    public NativeList<ContactViewCandidate> SortScratch;
    public NativeList<byte> CandidateWorkset;
    public NativeList<ContactViewPublicationBlock> PublicationBlocks;
    public NativeList<byte> BlockWorkset;
    public NativeList<ContactConstraint> OutputContacts;
    public NativeReference<int> RequiredMergePassCount;
    public int BlockSize;

    public void Execute()
    {
        Candidates.Clear();
        SortScratch.Clear();
        CandidateWorkset.Clear();
        PublicationBlocks.Clear();
        BlockWorkset.Clear();
        RequiredMergePassCount.Value = 0;
        if (RuntimeState.Value.IsValid == 0 ||
            PhaseState.Value.NeedsCommit != 2)
            return;

        int candidateCount = PreviousContacts.Length + NewContacts.Length;
        int blockCount = (candidateCount + BlockSize - 1) / BlockSize;
        Candidates.ResizeUninitialized(candidateCount);
        SortScratch.ResizeUninitialized(candidateCount);
        CandidateWorkset.ResizeUninitialized(candidateCount);
        PublicationBlocks.ResizeUninitialized(blockCount);
        BlockWorkset.ResizeUninitialized(blockCount);
        int requiredMergePassCount = 0;
        for (int width = 1; width < blockCount; width <<= 1)
            requiredMergePassCount++;
        RequiredMergePassCount.Value = requiredMergePassCount;
        OutputContacts.Clear();
    }
}

[BurstCompile]
internal struct MaterializeRepairContactCandidatesJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactConstraint> PreviousContacts;
    [ReadOnly] public NativeArray<ContactConstraint> NewContacts;
    [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactViewCandidate> Candidates;

    public void Execute(int candidateIndex)
    {
        int previousCount = PreviousContacts.Length;
        if (candidateIndex < previousCount)
        {
            ContactConstraint contact = PreviousContacts[candidateIndex];
            Candidates[candidateIndex] = new ContactViewCandidate
            {
                Contact = contact,
                IsValid = 1,
                IsPrevious = 1,
                PreviousWasDirty = (byte)(
                    TimestepContactRepairViewKernel.IsDirty(
                        contact,
                        DirtyFlagsByBody)
                        ? 1
                        : 0)
            };
            return;
        }

        Candidates[candidateIndex] = new ContactViewCandidate
        {
            Contact = NewContacts[candidateIndex - previousCount],
            IsValid = 1
        };
    }
}

[BurstCompile]
internal struct SortContactViewCandidateBlocksJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactViewCandidate> Candidates;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        int start = blockIndex * BlockSize;
        int length = math.min(BlockSize, Candidates.Length - start);
        if (length > 1)
        {
            Candidates.GetSubArray(start, length).Sort(
                new ContactViewCandidateComparer());
        }
    }
}

[BurstCompile]
internal struct MergeContactViewCandidateBlocksJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactViewCandidate> Source;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactViewCandidate> Destination;
    public int BlockSize;
    public int MergePass;
    [ReadOnly] public NativeReference<int> RequiredMergePassCount;

    public void Execute(int blockIndex)
    {
        if (MergePass >= RequiredMergePassCount.Value)
            return;
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
        var comparer = new ContactViewCandidateComparer();
        while (left < middle && right < end)
        {
            ContactViewCandidate a = Source[left];
            ContactViewCandidate b = Source[right];
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
internal struct CopyContactViewCandidateSortResultJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactViewCandidate> Source;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactViewCandidate> Destination;
    [ReadOnly] public NativeReference<int> RequiredMergePassCount;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        if ((RequiredMergePassCount.Value & 1) == 0)
            return;
        int start = blockIndex * BlockSize;
        int end = math.min(start + BlockSize, Source.Length);
        for (int candidateIndex = start;
             candidateIndex < end;
             candidateIndex++)
        {
            Destination[candidateIndex] = Source[candidateIndex];
        }
    }
}

[BurstCompile]
internal struct CountRepairContactPublicationBlocksJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactViewCandidate> Candidates;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactViewPublicationBlock> Blocks;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        int start = blockIndex * BlockSize;
        int end = math.min(start + BlockSize, Candidates.Length);
        ContactViewPublicationBlock block = default;
        for (int candidateIndex = start;
             candidateIndex < end;
             candidateIndex++)
        {
            if (!TimestepContactRepairViewKernel.IsGroupStart(
                    Candidates,
                    candidateIndex))
                continue;
            if (TimestepContactRepairViewKernel.TrySelectRepairContact(
                    Candidates,
                    candidateIndex,
                    out _,
                    out byte wasFallback))
            {
                block.OutputCount++;
                block.FallbackCount += wasFallback;
            }
        }
        Blocks[blockIndex] = block;
    }
}

[BurstCompile]
internal struct PrefixRepairContactPublicationJob : IJob
{
    public NativeList<ContactViewPublicationBlock> Blocks;
    public NativeList<ContactConstraint> OutputContacts;

    public void Execute()
    {
        int outputOffset = 0;
        for (int blockIndex = 0; blockIndex < Blocks.Length; blockIndex++)
        {
            ContactViewPublicationBlock block = Blocks[blockIndex];
            block.OutputOffset = outputOffset;
            outputOffset += block.OutputCount;
            Blocks[blockIndex] = block;
        }
        OutputContacts.ResizeUninitialized(outputOffset);
    }
}

[BurstCompile]
internal struct ScatterRepairContactPublicationBlocksJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactViewCandidate> Candidates;
    [ReadOnly] public NativeArray<ContactViewPublicationBlock> Blocks;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> OutputContacts;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        ContactViewPublicationBlock block = Blocks[blockIndex];
        int writeIndex = block.OutputOffset;
        int start = blockIndex * BlockSize;
        int end = math.min(start + BlockSize, Candidates.Length);
        for (int candidateIndex = start;
             candidateIndex < end;
             candidateIndex++)
        {
            if (!TimestepContactRepairViewKernel.IsGroupStart(
                    Candidates,
                    candidateIndex))
                continue;
            if (TimestepContactRepairViewKernel.TrySelectRepairContact(
                    Candidates,
                    candidateIndex,
                    out ContactConstraint contact,
                    out _))
            {
                OutputContacts[writeIndex++] = contact;
            }
        }
    }
}

[BurstCompile]
internal struct FinalizeRepairContactViewPublicationJob : IJob
{
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;
    [ReadOnly] public NativeReference<PersistentClassificationPhaseState>
        PhaseState;
    [ReadOnly] public NativeList<ContactViewPublicationBlock> Blocks;
    [ReadOnly] public NativeList<ContactConstraint> OutputContacts;
#if RTS_CONTACT_DIAGNOSTICS
    public ContactPipelineConfiguration Configuration;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
    [ReadOnly] public NativeList<BodyPair> SoftAvoidancePairs;
    [ReadOnly] public NativeList<PersistentPredictiveContact>
        PersistentContacts;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
    public NativeList<BodyPair> OracleContactPairs;
#endif

    public void Execute()
    {
        if (RuntimeState.Value.IsValid == 0 ||
            PhaseState.Value.NeedsCommit != 2)
            return;
#if RTS_CONTACT_DIAGNOSTICS
        int fallbackCount = 0;
        for (int blockIndex = 0; blockIndex < Blocks.Length; blockIndex++)
            fallbackCount += Blocks[blockIndex].FallbackCount;

        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
        statistics.TimestepContactSetFallbackAddedPairCount += fallbackCount;
        statistics.TimestepContactSetBuildCount++;
        statistics.TimestepContactSetClassificationPassCount++;
        statistics.TimestepContactSetUniquePairCount = OutputContacts.Length;
        statistics.TimestepContactSetDormantPairCount =
            incremental.CurrentDormantPairCount;
        PersistentContactMath.RefreshCurrentContactStateGauges(
            PersistentContacts,
            ref incremental,
            OutputContacts.Length);
        SoftAvoidanceOracleKernel.ValidateSoftAvoidancePairViewAgainstQuadraticOracle(
            ref incremental,
            Configuration,
            Bodies,
            MotionEvidence,
            StepStates,
            SoftAvoidancePairs);
        ContactOracleKernel.ValidateIncrementalContactSetAgainstQuadraticOracle(
            ref incremental,
            Configuration,
            Bodies,
            MotionEvidence,
            OutputContacts,
            OracleContactPairs);
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
#endif
    }
}

[BurstCompile]
internal struct PrepareActivationContactViewPublicationJob : IJob
{
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;
    [ReadOnly] public NativeList<ContactConstraint> ExistingContacts;
    [ReadOnly] public NativeList<ContactConstraint> ActivatedContacts;
    public NativeList<ContactViewCandidate> Candidates;
    public NativeList<ContactViewCandidate> SortScratch;
    public NativeList<byte> CandidateWorkset;
    public NativeList<ContactViewPublicationBlock> PublicationBlocks;
    public NativeList<byte> BlockWorkset;
    public NativeReference<int> RequiredMergePassCount;
    public int BlockSize;

    public void Execute()
    {
        Candidates.Clear();
        SortScratch.Clear();
        CandidateWorkset.Clear();
        PublicationBlocks.Clear();
        BlockWorkset.Clear();
        RequiredMergePassCount.Value = 0;
        if (RuntimeState.Value.IsValid == 0 ||
            ActivatedContacts.Length == 0)
            return;

        int candidateCount =
            ExistingContacts.Length + ActivatedContacts.Length;
        int blockCount = (candidateCount + BlockSize - 1) / BlockSize;
        Candidates.ResizeUninitialized(candidateCount);
        SortScratch.ResizeUninitialized(candidateCount);
        CandidateWorkset.ResizeUninitialized(candidateCount);
        PublicationBlocks.ResizeUninitialized(blockCount);
        BlockWorkset.ResizeUninitialized(blockCount);
        int requiredMergePassCount = 0;
        for (int width = 1; width < blockCount; width <<= 1)
            requiredMergePassCount++;
        RequiredMergePassCount.Value = requiredMergePassCount;
    }
}

[BurstCompile]
internal struct MaterializeActivationContactCandidatesJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactConstraint> ExistingContacts;
    [ReadOnly] public NativeArray<ContactConstraint> ActivatedContacts;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactViewCandidate> Candidates;

    public void Execute(int candidateIndex)
    {
        int existingCount = ExistingContacts.Length;
        if (candidateIndex < existingCount)
        {
            Candidates[candidateIndex] = new ContactViewCandidate
            {
                Contact = ExistingContacts[candidateIndex],
                IsValid = 1,
                IsPrevious = 1
            };
            return;
        }

        Candidates[candidateIndex] = new ContactViewCandidate
        {
            Contact = ActivatedContacts[candidateIndex - existingCount],
            IsValid = 1
        };
    }
}

[BurstCompile]
internal struct CountActivationContactPublicationBlocksJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactViewCandidate> Candidates;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactViewPublicationBlock> Blocks;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        int start = blockIndex * BlockSize;
        int end = math.min(start + BlockSize, Candidates.Length);
        ContactViewPublicationBlock block = default;
        for (int candidateIndex = start;
             candidateIndex < end;
             candidateIndex++)
        {
            if (!TimestepContactRepairViewKernel.IsGroupStart(
                    Candidates,
                    candidateIndex))
                continue;
            if (TimestepContactRepairViewKernel.TrySelectActivationContact(
                    Candidates,
                    candidateIndex,
                    out _))
            {
                block.OutputCount++;
            }
        }
        Blocks[blockIndex] = block;
    }
}

[BurstCompile]
internal struct PrefixActivationContactPublicationJob : IJob
{
    public NativeList<ContactViewPublicationBlock> Blocks;
    public NativeList<ContactConstraint> OutputContacts;

    public void Execute()
    {
        if (Blocks.Length == 0)
            return;

        int outputOffset = 0;
        for (int blockIndex = 0; blockIndex < Blocks.Length; blockIndex++)
        {
            ContactViewPublicationBlock block = Blocks[blockIndex];
            block.OutputOffset = outputOffset;
            outputOffset += block.OutputCount;
            Blocks[blockIndex] = block;
        }
        OutputContacts.ResizeUninitialized(outputOffset);
    }
}

[BurstCompile]
internal struct ScatterActivationContactPublicationBlocksJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ContactViewCandidate> Candidates;
    [ReadOnly] public NativeArray<ContactViewPublicationBlock> Blocks;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> OutputContacts;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        ContactViewPublicationBlock block = Blocks[blockIndex];
        int writeIndex = block.OutputOffset;
        int start = blockIndex * BlockSize;
        int end = math.min(start + BlockSize, Candidates.Length);
        for (int candidateIndex = start;
             candidateIndex < end;
             candidateIndex++)
        {
            if (!TimestepContactRepairViewKernel.IsGroupStart(
                    Candidates,
                    candidateIndex))
                continue;
            if (TimestepContactRepairViewKernel.TrySelectActivationContact(
                    Candidates,
                    candidateIndex,
                    out ContactConstraint contact))
            {
                OutputContacts[writeIndex++] = contact;
            }
        }
    }
}
}
