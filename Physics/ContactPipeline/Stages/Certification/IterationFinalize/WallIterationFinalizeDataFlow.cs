using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal static partial class IterationFinalizeDataFlow
{
    internal static void FinalizeWallIteration(
        int substepIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState
#if RTS_CONTACT_DIAGNOSTICS
        , NativeReference<ContactSolverIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics,
        NativeReference<IncrementalContactPipelineStatistics>
            incrementalStatisticsState,
        NativeReference<PredictiveDiscContactStatistics> statisticsState
#endif
        , int bodyBlockCount,
        ContactPipelineConfiguration configuration,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdMotionEvidence> motionEvidence,
        NativeArray<CrowdSolverBodyState> stepStates,
        NativeArray<byte> dirtyFlagsByBody,
        NativeList<IncrementalDirtyBody> dirtyBodies,
        NativeReference<InteractionCertificate> interactionCertificate,
        NativeList<InteractionCertificateViolation> certificateViolations,
        NativeArray<byte> correctedBodyFlags,
        NativeList<int> correctedBodyIndices,
        NativeArray<ParallelBodyStageResult> parallelBodyStatistics)
    {
        ContactPipelineExecutionState runtime = runtimeState.Value;
#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = statisticsState.Value;
        IncrementalContactPipelineStatistics incremental =
            incrementalStatisticsState.Value;
        ContactSolverIterationTelemetry iteration = iterationState.Value;
        for (int blockIndex = 0; blockIndex < bodyBlockCount; blockIndex++)
        {
            int bodyIndex = blockIndex * CrowdContactPipelineScheduler.ParallelBodyBatchSize;
            ParallelBodyStageResult body = parallelBodyStatistics[bodyIndex];
            iteration.TotalWallPositionCorrection += body.Total;
            iteration.MaxWallPositionCorrection = math.max(
                iteration.MaxWallPositionCorrection,
                body.Maximum);
        }

#endif
#if !RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = default;
        IncrementalContactPipelineStatistics incremental = default;
#endif

        if (runtime.IsValid != 0)
        {
            ContactEnvelopeValidationKernel.ValidateSolverCorrections(
                substepIndex,
                configuration.PredictiveSkin,
                bodies,
                motionEvidence,
                stepStates,
                dirtyFlagsByBody,
                dirtyBodies,
                interactionCertificate,
                certificateViolations,
                correctedBodyIndices,
                ref statistics,
                ref incremental);

            CorrectedBodyTrackingKernel.Reset(
                correctedBodyFlags,
                correctedBodyIndices);
        }

        if (runtime.IsValid == 0)
            return;

#if RTS_CONTACT_DIAGNOSTICS
        statisticsState.Value = statistics;
        incrementalStatisticsState.Value = incremental;
        iterationState.Value = iteration;
#endif
    }
}
}
