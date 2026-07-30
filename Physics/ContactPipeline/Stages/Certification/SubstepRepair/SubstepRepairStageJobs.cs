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
[BurstCompile]
internal struct FinalizeEnvelopeEscapesJob : IJob
{
    [ReadOnly] public NativeList<IncrementalDirtyBody> DirtyBodies;
    [ReadOnly] public NativeArray<ParallelBodyStageResult>
        ParallelBodyStatistics;
    public NativeReference<ContactPipelineExecutionState> RuntimeState;
    public byte EnableTimestepContactSetCache;
    public int SubstepIndex;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
#endif

    public void Execute()
    {
        if (RuntimeState.Value.IsValid == 0 ||
            EnableTimestepContactSetCache == 0)
            return;

#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
        int newlyEscaped = 0;
        for (int dirtyIndex = 0; dirtyIndex < DirtyBodies.Length; dirtyIndex++)
        {
            int bodyIndex = DirtyBodies[dirtyIndex].BodyIndex;
            if ((uint)bodyIndex < (uint)ParallelBodyStatistics.Length)
            {
                newlyEscaped +=
                    ParallelBodyStatistics[bodyIndex].EscapeCount;
            }
        }
        if (newlyEscaped > 0)
        {
            statistics.TimestepContactSetEscapeBodyCount += newlyEscaped;
            if (statistics.TimestepContactSetFirstEscapeSubstep < 0)
                statistics.TimestepContactSetFirstEscapeSubstep =
                    SubstepIndex;
        }
        incremental.InteractionEnvelopeEscapeCount += DirtyBodies.Length;
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
#endif
    }
}

[BurstCompile]
internal struct PrepareSubstepRepairBuffersJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    public NativeReference<byte> FullSweepPrepared;
    public NativeList<ContactConstraint> PreviousTimestepContactPairs;
    public NativeList<ContactConstraint> TimestepContactPairs;
    public NativeList<BodyPair> TimestepInteractionPairs;
    public NativeList<BodyPair> ClassificationBodyPairs;
    public NativeList<ContactConstraint> Pairs;
    [ReadOnly] public NativeList<IncrementalDirtyBody> DirtyBodies;
    public NativeReference<IncrementalContactCacheState> IncrementalCacheState;
    public NativeList<PersistentPairClassificationResult> ClassificationResults;
    public NativeReference<PersistentClassificationPhaseState> ClassificationState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PersistentClassificationTelemetryState> Telemetry;
#endif
    public NativeReference<ContactPipelineExecutionState> RuntimeState;

    public void Execute() => SubstepRepairDataFlow.PrepareSubstepRepairBuffers(
        RuntimeState,
        Configuration,
        FullSweepPrepared,
        PreviousTimestepContactPairs,
        TimestepContactPairs,
        TimestepInteractionPairs,
        ClassificationBodyPairs,
        Pairs,
        DirtyBodies,
        IncrementalCacheState,
        ClassificationResults,
        ClassificationState
#if RTS_CONTACT_DIAGNOSTICS
        , Telemetry
#endif
    );
}

[BurstCompile]
internal struct CopySubstepRepairInteractionPairsJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<BodyPair> Source;
    [NativeDisableParallelForRestriction]
    public NativeArray<BodyPair> Destination;

    public void Execute(int pairIndex)
    {
        Destination[pairIndex] = Source[pairIndex];
    }
}

[BurstCompile]
internal struct CopyPreviousTimestepContactPairsJob :
    IJobParallelForDefer
{
    [ReadOnly] public NativeArray<ContactConstraint> Source;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> Destination;

    public void Execute(int pairIndex)
    {
        Destination[pairIndex] = Source[pairIndex];
    }
}

[BurstCompile]
internal struct PublishSubstepRepairClassificationJob : IJob
{
    [ReadOnly] public NativeList<PersistentNeighborPair>
        PersistentNeighborPairs;
    [ReadOnly] public NativeList<ContactConstraint> Constraints;
    [ReadOnly] public NativeList<BodyPair> SoftAvoidancePairs;
    [ReadOnly] public NativeList<PredictiveContactScheduleEntry> Schedule;
    public NativeReference<IncrementalContactCacheState> CacheState;
    [ReadOnly] public NativeReference<PersistentClassificationPhaseState>
        PhaseState;
    public NativeReference<ActiveIncidentIndexState>
        ActiveIncidentIndexState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
#endif
    public NativeReference<ContactPipelineExecutionState> RuntimeState;

    public void Execute()
    {
        PersistentClassificationPhaseState phase = PhaseState.Value;
        if (RuntimeState.Value.IsValid == 0 || phase.NeedsCommit != 2)
            return;

        IncrementalContactCacheState cache = CacheState.Value;
        cache.ClassificationEpoch = phase.ClassificationEpoch;
        cache.LastUpdateWasFullRebuild = 1;
        cache.NeighborPairCount = PersistentNeighborPairs.Length;
        CacheState.Value = cache;
        ActiveIncidentIndexState incidentState =
            ActiveIncidentIndexState.Value;
        incidentState.SoftIsValid = 0;
        ActiveIncidentIndexState.Value = incidentState;
#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        statistics.ContactPairCount +=
            Constraints.Length + Schedule.Length;
        Statistics.Value = statistics;
        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
        incremental.CurrentInteractionPairCount =
            PersistentNeighborPairs.Length;
        incremental.CurrentSoftAvoidancePairCount =
            SoftAvoidancePairs.Length;
        incremental.PersistentViewRebuildCount++;
        IncrementalStatistics.Value = incremental;
#endif
    }
}

[BurstCompile]
internal struct MergeRepairedContactViewJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
    [ReadOnly] public NativeList<ContactConstraint> Constraints;
    public NativeList<ContactConstraint> TimestepContactPairs;
    [ReadOnly] public NativeList<ContactConstraint>
        PreviousTimestepContactPairs;
    [ReadOnly] public NativeList<BodyPair> SoftAvoidancePairs;
    [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
    [ReadOnly] public NativeList<PersistentPredictiveContact>
        PersistentContacts;
    [ReadOnly] public NativeReference<PersistentClassificationPhaseState>
        PhaseState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
    public NativeList<BodyPair> OracleContactPairs;
#endif
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;

    public void Execute()
    {
        if (RuntimeState.Value.IsValid == 0 ||
            PhaseState.Value.NeedsCommit != 2)
            return;
#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
#else
        PredictiveDiscContactStatistics statistics = default;
        IncrementalContactPipelineStatistics incremental = default;
#endif
        TimestepContactRepairViewKernel.MergeEscapedTimestepContactView(
            ref statistics,
            ref incremental,
            Configuration,
            Bodies,
            MotionEvidence,
            StepStates,
            Constraints,
            TimestepContactPairs,
            PreviousTimestepContactPairs,
            SoftAvoidancePairs,
            DirtyFlagsByBody,
            PersistentContacts
#if RTS_CONTACT_DIAGNOSTICS
            , OracleContactPairs
#endif
        );
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
#endif
    }
}

[BurstCompile]
internal struct ClearRepairedEnvelopeEscapeJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<IncrementalDirtyBody> Workset;
    [ReadOnly] public NativeReference<PersistentClassificationPhaseState>
        PhaseState;
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;
    [NativeDisableParallelForRestriction]
    public NativeArray<CrowdMotionEvidence> MotionEvidence;

    public void Execute(int dirtyIndex)
    {
        if (RuntimeState.Value.IsValid == 0 ||
            PhaseState.Value.NeedsCommit != 2)
            return;
        int bodyIndex = Workset[dirtyIndex].BodyIndex;
        CrowdMotionEvidence evidence = MotionEvidence[bodyIndex];
        evidence.EnvelopeEscaped = 0;
        MotionEvidence[bodyIndex] = evidence;
    }
}

[BurstCompile]
internal struct PreparePersistentIncidentLookupJob : IJob
{
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;
    [ReadOnly] public NativeReference<PersistentClassificationPhaseState>
        PhaseState;
    [ReadOnly] public NativeReference<IncrementalContactCacheState> CacheState;
    [ReadOnly] public NativeList<PersistentNeighborPair> Pairs;
    public NativeParallelMultiHashMap<Entity, int> IncidentPairLookup;
    public NativeReference<uint> IncidentLookupEpoch;
    public NativeList<byte> PairWorkset;
    public NativeReference<int> RebuildPairCount;
    public byte Enabled;

    public void Execute()
    {
        PairWorkset.Clear();
        RebuildPairCount.Value = -1;
        if (Enabled == 0 ||
            RuntimeState.Value.IsValid == 0 ||
            PhaseState.Value.NeedsCommit != 2)
            return;

        uint epoch = CacheState.Value.TopologyEpoch;
        int requiredEntryCount = Pairs.Length * 2;
        if (IncidentLookupEpoch.Value == epoch &&
            IncidentPairLookup.Count() == requiredEntryCount)
            return;
        if (requiredEntryCount > IncidentPairLookup.Capacity)
        {
            IncidentPairLookup.Clear();
            IncidentLookupEpoch.Value = uint.MaxValue;
            RebuildPairCount.Value = -2;
            return;
        }

        IncidentPairLookup.Clear();
        PairWorkset.ResizeUninitialized(Pairs.Length);
        RebuildPairCount.Value = Pairs.Length;
    }
}

[BurstCompile]
internal struct ScatterPersistentIncidentLookupJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeArray<PersistentNeighborPair> Pairs;
    public NativeParallelMultiHashMap<Entity, int>.ParallelWriter
        IncidentPairLookup;

    public void Execute(int pairIndex)
    {
        StableEntityPairKey key = Pairs[pairIndex].Key;
        IncidentPairLookup.Add(key.EntityA, pairIndex);
        IncidentPairLookup.Add(key.EntityB, pairIndex);
    }
}

[BurstCompile]
internal struct FinalizePersistentIncidentLookupJob : IJob
{
    [ReadOnly] public NativeReference<IncrementalContactCacheState> CacheState;
    [ReadOnly] public NativeList<PersistentNeighborPair> Pairs;
    [ReadOnly] public NativeReference<int> RebuildPairCount;
    public NativeReference<uint> IncidentLookupEpoch;

    public void Execute()
    {
        if (RebuildPairCount.Value == Pairs.Length)
            IncidentLookupEpoch.Value = CacheState.Value.TopologyEpoch;
    }
}

[BurstCompile]
internal struct FinalizeSubstepRepairCertificateJob : IJob
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
    public NativeReference<InteractionCertificate> InteractionCertificate;
    public NativeList<InteractionCertificateViolation> CertificateViolations;
    public NativeReference<PersistentClassificationPhaseState> PhaseState;
    public NativeReference<byte> FullSweepPrepared;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
    [ReadOnly] public NativeReference<PersistentClassificationTelemetryState>
        Telemetry;
#endif
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;
    public int SubstepIndex;

    public void Execute()
    {
        PersistentClassificationPhaseState phase = PhaseState.Value;
        if (RuntimeState.Value.IsValid == 0 || phase.NeedsCommit != 2)
            return;
#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
#else
        IncrementalContactPipelineStatistics incremental = default;
#endif
        incremental.UsedFullRebuild = 1;
        incremental.PersistentNeighborPairCount =
            PersistentNeighborPairs.Length;
#if RTS_CONTACT_DIAGNOSTICS
        PersistentClassificationTelemetryState telemetry = Telemetry.Value;
        incremental.SweptClassificationNanoseconds +=
            ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp -
                telemetry.ClassificationStartTimestamp);
        statistics.TimestepContactSetBuildNanoseconds +=
            ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp -
                telemetry.BuildStartTimestamp);
#endif
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
        phase.NeedsCommit = 0;
        PhaseState.Value = phase;
        FullSweepPrepared.Value = 0;
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
#endif
    }
}

[BurstCompile]
internal struct FinalizePreparedSubstepJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
    [ReadOnly] public NativeParallelHashMap<Entity, int>
        CurrentBodyIndexByEntity;
    [ReadOnly] public NativeList<BodyPair> TimestepInteractionPairs;
    [ReadOnly] public NativeList<BodyPair> SoftAvoidancePairs;
    public NativeList<ContactConstraint> TimestepContactPairs;
    [ReadOnly] public NativeList<PersistentNeighborPair>
        PersistentNeighborPairs;
    public NativeList<PersistentPredictiveContact> PersistentContacts;
    [ReadOnly] public NativeParallelHashMap<StableEntityPairKey, int> ContactIndex;
    public NativeList<PredictiveContactScheduleEntry> Schedule;
    public NativeList<PredictiveContactScheduleEntry> ScheduleScratch;
    public NativeReference<int> ScheduleCursor;
    public NativeReference<IncrementalContactCacheState> CacheState;
    [ReadOnly] public NativeList<IncrementalDirtyBody> DirtyBodies;
    public NativeReference<InteractionCertificate> InteractionCertificate;
    public NativeList<InteractionCertificateViolation> CertificateViolations;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
#endif
    public NativeReference<ContactPipelineExecutionState> RuntimeState;
    public int SubstepIndex;

    public void Execute() => SubstepRepairDataFlow.FinalizePreparedSubstep(
        SubstepIndex,
        RuntimeState,
        Configuration,
        Bodies,
        MotionEvidence,
        StepStates,
        CurrentBodyIndexByEntity,
        TimestepInteractionPairs,
        SoftAvoidancePairs,
        TimestepContactPairs,
        PersistentNeighborPairs,
        PersistentContacts,
        ContactIndex,
        Schedule,
        ScheduleScratch,
        ScheduleCursor,
        CacheState,
        DirtyBodies,
        InteractionCertificate,
        CertificateViolations
#if RTS_CONTACT_DIAGNOSTICS
        , Statistics,
        IncrementalStatistics
#endif
    );
}

}
