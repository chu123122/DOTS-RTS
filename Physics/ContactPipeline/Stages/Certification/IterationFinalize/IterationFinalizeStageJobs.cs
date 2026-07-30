using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
[BurstCompile]
internal struct FinalizeWallIterationJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
    public NativeArray<byte> DirtyFlagsByBody;
    public NativeList<IncrementalDirtyBody> DirtyBodies;
    public NativeReference<InteractionCertificate> InteractionCertificate;
    public NativeList<InteractionCertificateViolation> CertificateViolations;
    public NativeArray<byte> CorrectedBodyFlags;
    public NativeList<int> CorrectedBodyIndices;
    [ReadOnly] public NativeArray<ParallelBodyStageResult> ParallelBodyStatistics;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<ContactSolverIterationTelemetry> IterationState;
    [ReadOnly] public NativeList<JacobiBlockTelemetry> BlockStatistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
#endif
    public NativeReference<ContactPipelineExecutionState> RuntimeState;
    public int SubstepIndex;
    public int BodyBlockCount;

    public void Execute() => IterationFinalizeDataFlow.FinalizeWallIteration(
        SubstepIndex,
        RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
        , IterationState,
        BlockStatistics,
        IncrementalStatistics,
        Statistics
#endif
        , BodyBlockCount,
        Configuration,
        Bodies,
        MotionEvidence,
        StepStates,
        DirtyFlagsByBody,
        DirtyBodies,
        InteractionCertificate,
        CertificateViolations,
        CorrectedBodyFlags,
        CorrectedBodyIndices,
        ParallelBodyStatistics);
}

[BurstCompile]
internal struct FinalizeContactIterationJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
    public NativeList<ContactConstraint> TimestepContactPairs;
    public NativeArray<byte> DirtyFlagsByBody;
    public NativeList<IncrementalDirtyBody> DirtyBodies;
    public NativeReference<InteractionCertificate> InteractionCertificate;
    public NativeList<InteractionCertificateViolation> CertificateViolations;
    public NativeArray<byte> CorrectedBodyFlags;
    public NativeList<int> CorrectedBodyIndices;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<ContactSolverIterationTelemetry> IterationState;
    [ReadOnly] public NativeList<JacobiBlockTelemetry> BlockStatistics;
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<ContactIterationDiagnostic> IterationDiagnostics;
#endif
    public NativeReference<ContactPipelineExecutionState> RuntimeState;
    public int SubstepIndex;
    public int IterationIndex;

    public void Execute() => IterationFinalizeDataFlow.FinalizeContactIteration(
        SubstepIndex,
        IterationIndex,
        RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
        , IterationState,
        BlockStatistics,
        IncrementalStatistics,
        Statistics,
        IterationDiagnostics
#endif
        , Configuration,
        Bodies,
        MotionEvidence,
        StepStates,
        TimestepContactPairs,
        DirtyFlagsByBody,
        DirtyBodies,
        InteractionCertificate,
        CertificateViolations,
        CorrectedBodyFlags,
        CorrectedBodyIndices);
}
}
