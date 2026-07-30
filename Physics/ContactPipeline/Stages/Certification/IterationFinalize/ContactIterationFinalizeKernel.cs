using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;
namespace RTS.Unit.FlowField.Jobs
{
internal static partial class IterationFinalizeDataFlow
{
    internal static void FinalizeContactIteration(
        int substepIndex,
        int iterationIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState,
#if RTS_CONTACT_DIAGNOSTICS
        NativeReference<ContactSolverIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics,
        NativeReference<IncrementalContactPipelineStatistics>
            incrementalStatisticsState,
        NativeReference<PredictiveDiscContactStatistics> statisticsState,
        NativeList<ContactIterationDiagnostic> iterationDiagnostics,
#endif
        ContactPipelineConfiguration configuration,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdMotionEvidence> motionEvidence,
        NativeArray<CrowdSolverBodyState> stepStates,
        NativeList<ContactConstraint> timestepContactPairs,
        NativeArray<byte> dirtyFlagsByBody,
        NativeList<IncrementalDirtyBody> dirtyBodies,
        NativeReference<InteractionCertificate> interactionCertificate,
        NativeList<InteractionCertificateViolation> certificateViolations,
        NativeArray<byte> correctedBodyFlags,
        NativeList<int> correctedBodyIndices)
    {
        ContactPipelineExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;

#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = statisticsState.Value;
        IncrementalContactPipelineStatistics incrementalStatistics =
            incrementalStatisticsState.Value;
        ContactSolverIterationTelemetry iteration = iterationState.Value;
#else
        PredictiveDiscContactStatistics statistics = default;
        IncrementalContactPipelineStatistics incrementalStatistics = default;
#endif
        // Parallel bodies only set disjoint flags. Rebuild the corrected-body
        // list in body-index order so envelope repair stays deterministic.
        correctedBodyIndices.Clear();
        for (int bodyIndex = 0; bodyIndex < correctedBodyFlags.Length; bodyIndex++)
        {
            if (correctedBodyFlags[bodyIndex] != 0)
                correctedBodyIndices.Add(bodyIndex);
        }
#if RTS_CONTACT_DIAGNOSTICS
        float totalPositionCorrection =
            iteration.TotalContactPositionCorrection;
        float maxPositionCorrection =
            iteration.MaxContactPositionCorrection;
        if (configuration.ContactPositionSolver == ContactPositionSolverMode.Jacobi)
        {
            int newlyActivated = 0;
            int newlyCorrected = 0;
            for (int i = 0; i < blockStatistics.Length; i++)
            {
                JacobiBlockTelemetry block = blockStatistics[i];
                totalPositionCorrection += block.TotalPositionCorrection;
                maxPositionCorrection = math.max(
                    maxPositionCorrection,
                    block.MaxPositionCorrection);
                newlyActivated += block.NewlyActivatedPairCount;
                newlyCorrected += block.NewlyCorrectedPairCount;
            }

            statistics.TimestepContactSetUniqueActivatedPairCount +=
                newlyActivated;
            incrementalStatistics.UniqueCorrectedPairCount += newlyCorrected;
            incrementalStatistics.ActiveConstraintEvaluationCount +=
                timestepContactPairs.Length;
            statistics.TotalContactPositionCorrection +=
                totalPositionCorrection;
            statistics.MaxContactPositionCorrection = math.max(
                statistics.MaxContactPositionCorrection,
                maxPositionCorrection);
        }

        statistics.TotalWallPositionCorrection +=
            iteration.TotalWallPositionCorrection;
        statistics.MaxWallPositionCorrection = math.max(
            statistics.MaxWallPositionCorrection,
            iteration.MaxWallPositionCorrection);

        if (configuration.EnableDiagnostics)
        {
            ContactIterationDiagnostics.Record(
                bodies,
                stepStates,
                timestepContactPairs,
                iterationDiagnostics,
                substepIndex,
                iterationIndex,
                iteration.MaxViolationBeforeSolve,
                iteration.AverageViolationBeforeSolve,
                totalPositionCorrection,
                maxPositionCorrection,
                iteration.TotalWallPositionCorrection,
                iteration.MaxWallPositionCorrection);
        }
#endif

        if (!ContactEnvelopeValidationKernel.ValidateSolverCorrections(
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
                ref incrementalStatistics))
        {
            if (iterationIndex == math.max(1, configuration.IterationCount) - 1)
            {
                ContactConstraintStateKernel.ResetForSubstep(
                    timestepContactPairs);
                runtime.RecoveryRequired = 1;
            }
        }

#if RTS_CONTACT_DIAGNOSTICS
        statisticsState.Value = statistics;
        incrementalStatisticsState.Value = incrementalStatistics;
#endif
        runtimeState.Value = runtime;
    }
}
}
