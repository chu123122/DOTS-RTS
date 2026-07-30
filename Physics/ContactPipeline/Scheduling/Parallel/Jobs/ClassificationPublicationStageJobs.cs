using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal struct ClassificationPublicationRecord
{
    public BodyPair RawPair;
    public byte HasConstraint;
    public byte HasSoftAvoidance;
    public byte HasSchedule;
}

internal struct ClassificationPublicationBlock
{
    public int ConstraintCount;
    public int SoftAvoidanceCount;
    public int ScheduleCount;
    public int ConstraintOffset;
    public int SoftAvoidanceOffset;
    public int ScheduleOffset;
    public int ReclassifiedCount;
    public int ReusedCount;
    public int ActualCount;
    public int PredictiveCount;
    public int ApproachingCount;
    public int DormantCount;
    public int ExpiredCount;
}

[BurstCompile]
internal struct PrepareClassificationPublicationJob : IJob
{
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;
    [ReadOnly] public NativeReference<PersistentClassificationPhaseState>
        PhaseState;
    [ReadOnly] public NativeList<PersistentPairClassificationResult> Results;
    public NativeList<ClassificationPublicationRecord> Records;
    public NativeList<ClassificationPublicationBlock> Blocks;
    public NativeList<byte> BlockWorkset;
    public NativeList<PersistentPredictiveContact> PersistentContacts;
    public NativeList<ContactConstraint> Constraints;
    public NativeList<ContactConstraint> InitialTimestepContacts;
    public NativeList<BodyPair> SoftAvoidancePairs;
    public NativeList<PredictiveContactScheduleEntry> Schedule;
    public NativeParallelHashMap<StableEntityPairKey, int> ContactIndex;
    public byte ExpectedCommitState;
    public int BlockSize;

    public void Execute()
    {
        Records.Clear();
        Blocks.Clear();
        BlockWorkset.Clear();
        if (RuntimeState.Value.IsValid == 0 ||
            PhaseState.Value.NeedsCommit != ExpectedCommitState)
            return;

        int resultCount = Results.Length;
        Records.ResizeUninitialized(resultCount);
        int blockCount = (resultCount + BlockSize - 1) / BlockSize;
        Blocks.ResizeUninitialized(blockCount);
        BlockWorkset.ResizeUninitialized(blockCount);
        PersistentContacts.ResizeUninitialized(resultCount);
        Constraints.Clear();
        if (ExpectedCommitState == 1)
            InitialTimestepContacts.Clear();
        SoftAvoidancePairs.Clear();
        Schedule.Clear();
        if (ContactIndex.IsCreated)
            ContactIndex.Clear();
    }
}

[BurstCompile]
internal struct MaterializeClassificationPublicationJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<PersistentPairClassificationResult> Results;
    [NativeDisableParallelForRestriction]
    public NativeArray<ClassificationPublicationRecord> Records;
    [NativeDisableParallelForRestriction]
    public NativeArray<PersistentPredictiveContact> PersistentContacts;

    public void Execute(int pairIndex)
    {
        PersistentPairClassificationResult result = Results[pairIndex];
        PersistentPredictiveContact contact = result.Contact;
        PersistentContacts[pairIndex] = contact;
        Records[pairIndex] = new ClassificationPublicationRecord
        {
            RawPair = result.RawPair,
            HasConstraint = (byte)(
                contact.Lifecycle != PersistentContactLifecycle.Expired &&
                contact.Lifecycle != PersistentContactLifecycle.Dormant
                    ? 1
                    : 0),
            HasSoftAvoidance = contact.SoftAvoidanceCandidate,
            HasSchedule = (byte)(
                contact.Lifecycle == PersistentContactLifecycle.Dormant
                    ? 1
                    : 0)
        };
    }
}

[BurstCompile]
internal struct BuildClassificationContactIndexJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<ClassificationPublicationRecord> Workset;
    [ReadOnly] public NativeArray<PersistentPredictiveContact> Contacts;
    public NativeParallelHashMap<StableEntityPairKey, int>.ParallelWriter
        ContactIndex;

    public void Execute(int contactIndex)
    {
        ContactIndex.TryAdd(Contacts[contactIndex].Key, contactIndex);
    }
}

[BurstCompile]
internal struct CountClassificationPublicationBlocksJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ClassificationPublicationRecord> Records;
    [ReadOnly] public NativeArray<PersistentPredictiveContact> Contacts;
    [ReadOnly] public NativeArray<PersistentPairClassificationResult> Results;
    [NativeDisableParallelForRestriction]
    public NativeArray<ClassificationPublicationBlock> Blocks;
    public int BlockSize;

    public void Execute(int blockIndex)
    {
        int begin = blockIndex * BlockSize;
        int end = math.min(begin + BlockSize, Records.Length);
        ClassificationPublicationBlock block = default;
        for (int pairIndex = begin; pairIndex < end; pairIndex++)
        {
            ClassificationPublicationRecord record = Records[pairIndex];
            PersistentPredictiveContact contact = Contacts[pairIndex];
            block.ConstraintCount += record.HasConstraint;
            block.SoftAvoidanceCount += record.HasSoftAvoidance;
            block.ScheduleCount += record.HasSchedule;
            if (Results[pairIndex].WasReclassified != 0)
                block.ReclassifiedCount++;
            else
                block.ReusedCount++;
            switch (contact.Lifecycle)
            {
                case PersistentContactLifecycle.Actual:
                case PersistentContactLifecycle.Separating:
                    block.ActualCount++;
                    break;
                case PersistentContactLifecycle.Predictive:
                    block.PredictiveCount++;
                    break;
                case PersistentContactLifecycle.Approaching:
                    block.ApproachingCount++;
                    break;
                case PersistentContactLifecycle.Dormant:
                    block.DormantCount++;
                    break;
                default:
                    block.ExpiredCount++;
                    break;
            }
        }
        Blocks[blockIndex] = block;
    }
}

[BurstCompile]
internal struct PrefixClassificationPublicationJob : IJob
{
    public NativeList<ClassificationPublicationBlock> Blocks;
    public NativeList<ContactConstraint> Constraints;
    public NativeList<ContactConstraint> InitialTimestepContacts;
    public NativeList<BodyPair> SoftAvoidancePairs;
    public NativeList<PredictiveContactScheduleEntry> Schedule;
    public NativeReference<IncrementalContactCacheState> CacheState;
    public byte PublishInitialTimestepContacts;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
#endif

    public void Execute()
    {
        int constraintOffset = 0;
        int softOffset = 0;
        int scheduleOffset = 0;
        int reclassified = 0;
        int reused = 0;
        int actual = 0;
        int predictive = 0;
        int approaching = 0;
        int dormant = 0;
        int expired = 0;
        for (int blockIndex = 0; blockIndex < Blocks.Length; blockIndex++)
        {
            ClassificationPublicationBlock block = Blocks[blockIndex];
            block.ConstraintOffset = constraintOffset;
            block.SoftAvoidanceOffset = softOffset;
            block.ScheduleOffset = scheduleOffset;
            constraintOffset += block.ConstraintCount;
            softOffset += block.SoftAvoidanceCount;
            scheduleOffset += block.ScheduleCount;
            reclassified += block.ReclassifiedCount;
            reused += block.ReusedCount;
            actual += block.ActualCount;
            predictive += block.PredictiveCount;
            approaching += block.ApproachingCount;
            dormant += block.DormantCount;
            expired += block.ExpiredCount;
            Blocks[blockIndex] = block;
        }

        Constraints.ResizeUninitialized(constraintOffset);
        if (PublishInitialTimestepContacts != 0)
            InitialTimestepContacts.ResizeUninitialized(constraintOffset);
        SoftAvoidancePairs.ResizeUninitialized(softOffset);
        Schedule.ResizeUninitialized(scheduleOffset);
        IncrementalContactCacheState cache = CacheState.Value;
        cache.DormantContactCount = dormant;
        cache.ApproachingContactCount = approaching;
        cache.PredictiveContactCount = predictive;
        cache.ActualContactCount = actual;
        cache.ExpiredContactCount = expired;
        cache.ContactViewsValid = 1;
        CacheState.Value = cache;
#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        statistics.ActualGeneratedPairCount += actual;
        statistics.PredictiveGeneratedPairCount +=
            predictive + approaching;
        statistics.PredictivePairCount += predictive;
        statistics.PotentialPredictivePairCount += predictive;
        statistics.TimestepContactSetDormantPairCount += dormant;
        Statistics.Value = statistics;

        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
        incremental.ReclassifiedPairEvaluationCount += reclassified;
        incremental.SweptClassificationEvaluationCount += reclassified;
        incremental.ClassificationReuseCount += reused;
        incremental.ClassificationSkippedCount += reused;
        incremental.CurrentSweptContactCount =
            actual + predictive + approaching + dormant;
        incremental.CurrentActualPairCount = actual;
        incremental.CurrentPredictivePairCount = predictive;
        incremental.CurrentApproachingPairCount = approaching;
        incremental.CurrentDormantPairCount = dormant;
        PersistentContactMath.UpdateActiveConstraintGauges(
            ref incremental,
            constraintOffset);
        IncrementalStatistics.Value = incremental;
#endif
    }

}

[BurstCompile]
internal struct ScatterClassificationPublicationBlocksJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<ClassificationPublicationRecord> Records;
    [ReadOnly] public NativeArray<PersistentPredictiveContact> Contacts;
    [ReadOnly] public NativeArray<ClassificationPublicationBlock> Blocks;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> Constraints;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> InitialTimestepContacts;
    [NativeDisableParallelForRestriction]
    public NativeArray<BodyPair> SoftAvoidancePairs;
    [NativeDisableParallelForRestriction]
    public NativeArray<PredictiveContactScheduleEntry> Schedule;
    public int BlockSize;
    public byte PublishInitialTimestepContacts;

    public void Execute(int blockIndex)
    {
        ClassificationPublicationBlock block = Blocks[blockIndex];
        int constraintWrite = block.ConstraintOffset;
        int softWrite = block.SoftAvoidanceOffset;
        int scheduleWrite = block.ScheduleOffset;
        int begin = blockIndex * BlockSize;
        int end = math.min(begin + BlockSize, Records.Length);
        for (int pairIndex = begin; pairIndex < end; pairIndex++)
        {
            ClassificationPublicationRecord record = Records[pairIndex];
            PersistentPredictiveContact contact = Contacts[pairIndex];
            if (record.HasConstraint != 0)
            {
                ContactConstraint constraint =
                    PersistentContactMath.BuildConstraintFromPersistentContact(
                        record.RawPair.BodyA,
                        record.RawPair.BodyB,
                        contact);
                Constraints[constraintWrite] = constraint;
                if (PublishInitialTimestepContacts != 0)
                {
                    InitialTimestepContacts[constraintWrite] =
                        constraint;
                }
                constraintWrite++;
            }
            if (record.HasSoftAvoidance != 0)
                SoftAvoidancePairs[softWrite++] = record.RawPair;
            if (record.HasSchedule != 0)
            {
                Schedule[scheduleWrite++] =
                    new PredictiveContactScheduleEntry
                    {
                        Key = contact.Key,
                        Substep = contact.NextCheckSubstep
                    };
            }
        }
    }
}
}
