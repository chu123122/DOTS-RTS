using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal enum PredictiveContactActivationAction : byte
{
    Future = 0,
    Activated = 1,
    Rescheduled = 2,
    Expired = 3
}

internal struct PredictiveContactActivationRecord
{
    public PredictiveContactScheduleEntry Entry;
    public ContactConstraint Constraint;
    public PersistentPredictiveContact PersistentContact;
    public int PersistentContactIndex;
    public PredictiveContactActivationAction Action;
    public byte HasPersistentUpdate;
}

internal struct PredictiveContactActivationBlock
{
    public int ActivatedCount;
    public int ScheduleCount;
    public int WakeupCount;
    public int ActivatedOffset;
    public int ScheduleOffset;
}

internal struct PredictiveContactActivationSummary
{
    public int ActivatedCount;
    public int ScheduleCount;
    public int WakeupCount;
}

[BurstCompile]
internal struct PreparePredictiveContactActivationJob : IJob
{
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;
    [ReadOnly] public NativeList<PredictiveContactScheduleEntry> Schedule;
    public NativeList<PredictiveContactActivationRecord> Records;
    public NativeList<byte> RecordWorkset;
    public NativeList<PredictiveContactActivationBlock> Blocks;
    public NativeList<byte> BlockWorkset;
    public NativeList<ContactConstraint> ActivatedContacts;
    public NativeList<PredictiveContactScheduleEntry> ScheduleScratch;
    public NativeReference<PredictiveContactActivationSummary> Summary;
    public NativeReference<long> StartTimestamp;
    public int BlockSize;

    public void Execute()
    {
        Records.Clear();
        RecordWorkset.Clear();
        Blocks.Clear();
        BlockWorkset.Clear();
        ActivatedContacts.Clear();
        ScheduleScratch.Clear();
        Summary.Value = default;
        StartTimestamp.Value = ProfilerUnsafeUtility.Timestamp;
        if (RuntimeState.Value.IsValid == 0)
            return;

        int scheduleCount = Schedule.Length;
        int blockCount = (scheduleCount + BlockSize - 1) / BlockSize;
        Records.ResizeUninitialized(scheduleCount);
        RecordWorkset.ResizeUninitialized(scheduleCount);
        Blocks.ResizeUninitialized(blockCount);
        BlockWorkset.ResizeUninitialized(blockCount);
    }
}

[BurstCompile]
internal struct EvaluateScheduledContactsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<PredictiveContactScheduleEntry> Schedule;
    public ContactPipelineConfiguration Configuration;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
    [ReadOnly] public NativeParallelHashMap<Entity, int>
        CurrentBodyIndexByEntity;
    [ReadOnly] public NativeParallelHashMap<StableEntityPairKey, int>
        ContactIndex;
    [ReadOnly]
    public NativeArray<PersistentPredictiveContact> PersistentContacts;
    [NativeDisableParallelForRestriction]
    public NativeArray<PredictiveContactActivationRecord> Records;
    public int SubstepIndex;
    public int SubstepCount;

    public void Execute(int scheduleIndex)
    {
        PredictiveContactScheduleEntry entry = Schedule[scheduleIndex];
        var record = new PredictiveContactActivationRecord
        {
            Entry = entry,
            Action = PredictiveContactActivationAction.Future
        };
        if (entry.Substep > SubstepIndex)
        {
            Records[scheduleIndex] = record;
            return;
        }

        int persistentContactIndex =
            PredictiveContactActivationKernel
                .FindPersistentPredictiveContactIndex(
                    entry.Key,
                    PersistentContacts,
                    ContactIndex);
        if (!PredictiveContactActivationKernel.TryFindCurrentBodyIndex(
                entry.Key.EntityA,
                Bodies.Length,
                CurrentBodyIndexByEntity,
                out int bodyA) ||
            !PredictiveContactActivationKernel.TryFindCurrentBodyIndex(
                entry.Key.EntityB,
                Bodies.Length,
                CurrentBodyIndexByEntity,
                out int bodyB))
        {
            if (PredictiveContactActivationKernel
                    .TryBuildExpiredPersistentContact(
                        persistentContactIndex,
                        PersistentContacts,
                        out PersistentPredictiveContact expiredContact))
            {
                record.PersistentContactIndex = persistentContactIndex;
                record.PersistentContact = expiredContact;
                record.HasPersistentUpdate = 1;
            }
            record.Action = PredictiveContactActivationAction.Expired;
            Records[scheduleIndex] = record;
            return;
        }

        if (PredictiveContactActivationKernel.TryBuildCurrentScheduledPair(
                bodyA,
                bodyB,
                Configuration,
                Bodies,
                MotionEvidence,
                StepStates,
                out ContactConstraint constraint))
        {
            if (PredictiveContactActivationKernel
                    .TryBuildPersistentContactAfterScheduledCheck(
                        persistentContactIndex,
                        constraint,
                        ushort.MaxValue,
                        Bodies,
                        StepStates,
                        PersistentContacts,
                        out PersistentPredictiveContact updatedContact))
            {
                record.PersistentContactIndex = persistentContactIndex;
                record.PersistentContact = updatedContact;
                record.HasPersistentUpdate = 1;
            }
            record.Constraint = constraint;
            record.Action = PredictiveContactActivationAction.Activated;
            Records[scheduleIndex] = record;
            return;
        }

        if (SubstepIndex + 1 < SubstepCount)
        {
            ushort nextSubstep = (ushort)(SubstepIndex + 1);
            entry.Substep = nextSubstep;
            if (PredictiveContactActivationKernel
                    .TryBuildPersistentContactNextCheck(
                        persistentContactIndex,
                        nextSubstep,
                        PersistentContacts,
                        out PersistentPredictiveContact updatedContact))
            {
                record.PersistentContactIndex = persistentContactIndex;
                record.PersistentContact = updatedContact;
                record.HasPersistentUpdate = 1;
            }
            record.Entry = entry;
            record.Action = PredictiveContactActivationAction.Rescheduled;
        }
        else
        {
            if (PredictiveContactActivationKernel
                    .TryBuildExpiredPersistentContact(
                        persistentContactIndex,
                        PersistentContacts,
                        out PersistentPredictiveContact expiredContact))
            {
                record.PersistentContactIndex = persistentContactIndex;
                record.PersistentContact = expiredContact;
                record.HasPersistentUpdate = 1;
            }
            record.Action = PredictiveContactActivationAction.Expired;
        }
        Records[scheduleIndex] = record;
    }
}

[BurstCompile]
internal struct CountPredictiveContactActivationBlocksJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<PredictiveContactActivationRecord> Records;
    [NativeDisableParallelForRestriction]
    public NativeArray<PredictiveContactActivationBlock> Blocks;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        int start = blockIndex * BlockSize;
        int end = math.min(start + BlockSize, Records.Length);
        PredictiveContactActivationBlock block = default;
        for (int recordIndex = start;
             recordIndex < end;
             recordIndex++)
        {
            switch (Records[recordIndex].Action)
            {
                case PredictiveContactActivationAction.Activated:
                    block.ActivatedCount++;
                    block.WakeupCount++;
                    break;
                case PredictiveContactActivationAction.Rescheduled:
                    block.ScheduleCount++;
                    block.WakeupCount++;
                    break;
                case PredictiveContactActivationAction.Expired:
                    block.WakeupCount++;
                    break;
                default:
                    block.ScheduleCount++;
                    break;
            }
        }
        Blocks[blockIndex] = block;
    }
}

[BurstCompile]
internal struct PrefixPredictiveContactActivationJob : IJob
{
    public NativeList<PredictiveContactActivationBlock> Blocks;
    public NativeList<ContactConstraint> ActivatedContacts;
    public NativeList<PredictiveContactScheduleEntry> ScheduleScratch;
    public NativeReference<PredictiveContactActivationSummary> Summary;
    public NativeReference<IncrementalContactCacheState> CacheState;
    public byte EnablePersistentContactCache;

    public void Execute()
    {
        int activatedOffset = 0;
        int scheduleOffset = 0;
        int wakeupCount = 0;
        for (int blockIndex = 0; blockIndex < Blocks.Length; blockIndex++)
        {
            PredictiveContactActivationBlock block = Blocks[blockIndex];
            block.ActivatedOffset = activatedOffset;
            block.ScheduleOffset = scheduleOffset;
            activatedOffset += block.ActivatedCount;
            scheduleOffset += block.ScheduleCount;
            wakeupCount += block.WakeupCount;
            Blocks[blockIndex] = block;
        }

        ActivatedContacts.ResizeUninitialized(activatedOffset);
        ScheduleScratch.ResizeUninitialized(scheduleOffset);
        Summary.Value = new PredictiveContactActivationSummary
        {
            ActivatedCount = activatedOffset,
            ScheduleCount = scheduleOffset,
            WakeupCount = wakeupCount
        };

        if (EnablePersistentContactCache != 0 && wakeupCount > 0)
        {
            IncrementalContactCacheState state = CacheState.Value;
            state.ContactViewsValid = 0;
            CacheState.Value = state;
        }
    }
}

[BurstCompile]
internal struct ScatterPredictiveContactActivationBlocksJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<PredictiveContactActivationRecord> Records;
    [ReadOnly] public NativeArray<PredictiveContactActivationBlock> Blocks;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> ActivatedContacts;
    [NativeDisableParallelForRestriction]
    public NativeArray<PredictiveContactScheduleEntry> ScheduleScratch;
    [NativeDisableParallelForRestriction]
    public NativeArray<PersistentPredictiveContact> PersistentContacts;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        PredictiveContactActivationBlock block = Blocks[blockIndex];
        int activatedWrite = block.ActivatedOffset;
        int scheduleWrite = block.ScheduleOffset;
        int start = blockIndex * BlockSize;
        int end = math.min(start + BlockSize, Records.Length);
        for (int recordIndex = start;
             recordIndex < end;
             recordIndex++)
        {
            PredictiveContactActivationRecord record = Records[recordIndex];
            if (record.HasPersistentUpdate != 0)
            {
                PersistentContacts[record.PersistentContactIndex] =
                    record.PersistentContact;
            }
            switch (record.Action)
            {
                case PredictiveContactActivationAction.Activated:
                    ActivatedContacts[activatedWrite++] = record.Constraint;
                    break;
                case PredictiveContactActivationAction.Rescheduled:
                case PredictiveContactActivationAction.Future:
                    ScheduleScratch[scheduleWrite++] = record.Entry;
                    break;
            }
        }
    }
}

[BurstCompile]
internal struct PreparePredictiveContactScheduleCommitJob : IJob
{
    [ReadOnly] public NativeList<PredictiveContactScheduleEntry>
        ScheduleScratch;
    public NativeList<PredictiveContactScheduleEntry> Schedule;

    public void Execute()
    {
        Schedule.ResizeUninitialized(ScheduleScratch.Length);
    }
}

[BurstCompile]
internal struct CopyPredictiveContactScheduleJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<PredictiveContactScheduleEntry> Source;
    [NativeDisableParallelForRestriction]
    public NativeArray<PredictiveContactScheduleEntry> Destination;

    public void Execute(int scheduleIndex)
    {
        Destination[scheduleIndex] = Source[scheduleIndex];
    }
}

[BurstCompile]
internal struct FinalizePredictiveContactActivationJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeList<BodyPair> TimestepInteractionPairs;
    [ReadOnly] public NativeList<BodyPair> SoftAvoidancePairs;
    [ReadOnly] public NativeList<ContactConstraint> TimestepContactPairs;
    [ReadOnly] public NativeParallelHashMap<Entity, int>
        CurrentBodyIndexByEntity;
    [ReadOnly] public NativeList<PersistentNeighborPair>
        PersistentNeighborPairs;
    [ReadOnly] public NativeList<PredictiveContactScheduleEntry> Schedule;
    public NativeReference<IncrementalContactCacheState> CacheState;
    [ReadOnly] public NativeList<IncrementalDirtyBody> DirtyBodies;
    [ReadOnly] public NativeReference<PredictiveContactActivationSummary>
        Summary;
    [ReadOnly] public NativeReference<long> StartTimestamp;
    public NativeReference<InteractionCertificate> InteractionCertificate;
    public NativeList<InteractionCertificateViolation> CertificateViolations;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
#endif
    public NativeReference<ContactPipelineExecutionState> RuntimeState;
    public int SubstepIndex;

    public void Execute()
    {
        ContactPipelineExecutionState runtime = RuntimeState.Value;
        if (runtime.IsValid == 0)
            return;

#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
#else
        PredictiveDiscContactStatistics statistics = default;
        IncrementalContactPipelineStatistics incremental = default;
#endif
        PredictiveContactActivationSummary summary = Summary.Value;
        incremental.CorrectedEscapeBodyCount += DirtyBodies.Length;
        incremental.ScheduledWakeupCount += summary.WakeupCount;
        PersistentContactMath.UpdateActiveConstraintGauges(
            ref incremental,
            TimestepContactPairs.Length);
        incremental.ContactActivationNanoseconds +=
            ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - StartTimestamp.Value);
        statistics.TimestepContactSetSubstepUseCount++;

        InteractionCertificateKernel.IssueCertificateForCommittedViews(
            incremental,
            Configuration,
            Bodies,
            TimestepInteractionPairs,
            SoftAvoidancePairs,
            TimestepContactPairs,
            CurrentBodyIndexByEntity,
            PersistentNeighborPairs,
            Schedule,
            CacheState,
            InteractionCertificate,
            CertificateViolations,
            SubstepIndex);
#if RTS_CONTACT_DIAGNOSTICS
        runtime.IterationStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        runtime.IterationAccountedStartNanoseconds =
            ContactPipelineDiagnosticsMath.AccountedCandidateNanoseconds(
                incremental);
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
#endif
        RuntimeState.Value = runtime;
    }
}
}
