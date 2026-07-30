using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
[BurstCompile]
internal struct PreparePersistentClassificationJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    public NativeReference<byte> FullSweepPrepared;
    public NativeList<ContactConstraint> PreviousTimestepContactPairs;
    public NativeList<BodyPair> TimestepInteractionPairs;
    public NativeList<BodyPair> ClassificationBodyPairs;
    public NativeReference<IncrementalContactCacheState> IncrementalCacheState;
    public NativeList<PersistentPairClassificationResult> ClassificationResults;
    public NativeReference<PersistentClassificationPhaseState> ClassificationState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PersistentClassificationTelemetryState> Telemetry;
#endif
    public NativeReference<ContactPipelineExecutionState> RuntimeState;

    public void Execute() => PersistentClassificationDataFlow.PreparePersistentClassification(
        RuntimeState,
        Configuration,
        FullSweepPrepared,
        PreviousTimestepContactPairs,
        TimestepInteractionPairs,
        ClassificationBodyPairs,
        IncrementalCacheState,
        ClassificationResults,
        ClassificationState
#if RTS_CONTACT_DIAGNOSTICS
        , Telemetry
#endif
    );
}

[BurstCompile]
internal struct PublishPersistentClassificationStateJob : IJob
{
    [ReadOnly] public NativeList<BodyPair> TimestepInteractionPairs;
    [ReadOnly] public NativeList<BodyPair> SoftAvoidancePairs;
    [ReadOnly] public NativeList<PersistentNeighborPair>
        PersistentNeighborPairs;
    [ReadOnly] public NativeList<ContactConstraint> Constraints;
    [ReadOnly] public NativeList<PredictiveContactScheduleEntry> Schedule;
    public NativeReference<IncrementalContactCacheState> CacheState;
    [ReadOnly] public NativeReference<PersistentClassificationPhaseState>
        PhaseState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
#endif
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;

    public void Execute()
    {
        PersistentClassificationPhaseState phase = PhaseState.Value;
        if (RuntimeState.Value.IsValid == 0 || phase.NeedsCommit != 1)
            return;
        IncrementalContactCacheState cache = CacheState.Value;
        cache.ClassificationEpoch = phase.ClassificationEpoch;
        CacheState.Value = cache;
#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        statistics.CandidatePairCount += TimestepInteractionPairs.Length;
        statistics.ContactPairCount +=
            Constraints.Length + Schedule.Length;
        Statistics.Value = statistics;
        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
        incremental.CurrentInteractionPairCount =
            TimestepInteractionPairs.Length;
        incremental.CurrentSoftAvoidancePairCount =
            SoftAvoidancePairs.Length;
        incremental.PersistentViewRebuildCount++;
        incremental.PersistentNeighborPairCount =
            PersistentNeighborPairs.Length;
        IncrementalStatistics.Value = incremental;
#endif
    }
}

#if RTS_CONTACT_DIAGNOSTICS
[BurstCompile]
internal struct ValidatePersistentClassificationOraclesJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
    [ReadOnly] public NativeList<BodyPair> SoftAvoidancePairs;
    [ReadOnly] public NativeList<ContactConstraint> TimestepContactPairs;
    public NativeList<BodyPair> OracleContactPairs;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
    [ReadOnly] public NativeReference<PersistentClassificationTelemetryState>
        Telemetry;
    [ReadOnly] public NativeReference<PersistentClassificationPhaseState>
        PhaseState;
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;

    public void Execute()
    {
        if (RuntimeState.Value.IsValid == 0 ||
            PhaseState.Value.NeedsCommit != 1)
            return;
        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
        SoftAvoidanceOracleKernel
            .ValidateSoftAvoidancePairViewAgainstQuadraticOracle(
                ref incremental,
                Configuration,
                Bodies,
                MotionEvidence,
                StepStates,
                SoftAvoidancePairs);
        ContactOracleKernel
            .ValidateIncrementalContactSetAgainstQuadraticOracle(
                ref incremental,
                Configuration,
                Bodies,
                MotionEvidence,
                TimestepContactPairs,
                OracleContactPairs);
        PersistentClassificationTelemetryState telemetry = Telemetry.Value;
        incremental.SweptClassificationNanoseconds +=
            ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp -
                telemetry.ClassificationStartTimestamp);
        IncrementalStatistics.Value = incremental;

        PredictiveDiscContactStatistics statistics = Statistics.Value;
        long elapsed = ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - telemetry.BuildStartTimestamp);
        statistics.TimestepContactSetBuildNanoseconds += elapsed;
        statistics.PairGenerationNanoseconds += elapsed;
        Statistics.Value = statistics;
    }
}
#endif

[BurstCompile]
internal struct FinalizePersistentClassificationCertificateJob : IJob
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
#endif
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;

    public void Execute()
    {
        PersistentClassificationPhaseState phase = PhaseState.Value;
        if (RuntimeState.Value.IsValid == 0 || phase.NeedsCommit != 1)
            return;
#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
#else
        PredictiveDiscContactStatistics statistics = default;
        IncrementalContactPipelineStatistics incremental = default;
#endif
        statistics.TimestepContactSetBuildCount++;
        statistics.TimestepContactSetClassificationPassCount++;
        statistics.TimestepContactSetUniquePairCount =
            TimestepContactPairs.Length;
        statistics.TimestepContactSetDormantPairCount =
            incremental.CurrentDormantPairCount;
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
            CertificateViolations);
        phase.NeedsCommit = 0;
        PhaseState.Value = phase;
        FullSweepPrepared.Value = 0;
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incremental;
#endif
    }
}
}
