using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;
namespace RTS.Unit.FlowField.Jobs
{
public partial struct InteractionCertificationJob
{
    private void FinalizeParallelJacobiIteration(
        int substepIndex,
        int iterationIndex,
        NativeReference<ParallelJacobiExecutionState> runtimeState
#if RTS_CONTACT_DIAGNOSTICS
        , NativeReference<ParallelJacobiIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics
#endif
        )
    {
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incrementalStatistics = LoadIncrementalStatistics();
#if RTS_CONTACT_DIAGNOSTICS
        ParallelJacobiIterationTelemetry iteration = iterationState.Value;
#endif
        // Parallel bodies only set disjoint flags. Rebuild the corrected-body
        // list in body-index order so envelope repair stays deterministic.
        CorrectedBodyIndices.Clear();
        for (int bodyIndex = 0; bodyIndex < CorrectedBodyFlags.Length; bodyIndex++)
        {
            if (CorrectedBodyFlags[bodyIndex] != 0)
                CorrectedBodyIndices.Add(bodyIndex);
        }
#if RTS_CONTACT_DIAGNOSTICS
        float totalPositionCorrection = 0f;
        float maxPositionCorrection = 0f;
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

        statistics.TimestepContactSetUniqueActivatedPairCount += newlyActivated;
        incrementalStatistics.UniqueCorrectedPairCount += newlyCorrected;
        incrementalStatistics.ActiveConstraintEvaluationCount +=
            TimestepContactPairs.Length;
        statistics.TotalContactPositionCorrection += totalPositionCorrection;
        statistics.MaxContactPositionCorrection = math.max(
            statistics.MaxContactPositionCorrection,
            maxPositionCorrection);
        statistics.TotalWallPositionCorrection +=
            iteration.TotalWallPositionCorrection;
        statistics.MaxWallPositionCorrection = math.max(
            statistics.MaxWallPositionCorrection,
            iteration.MaxWallPositionCorrection);

        if (EnableDiagnostics)
        {
            RecordIterationDiagnostic(
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

        if (!ValidateSolverCorrectionContactEnvelope(
                substepIndex,
                ref statistics,
                ref incrementalStatistics))
        {
            int substepCount = math.max(1, SubstepCount);
            float substepDeltaTime = DeltaTime / substepCount;
            RepairOrRebuildContactViewForRemainingTime(
                substepIndex,
                substepCount,
                substepDeltaTime,
                EnableTimestepContactSetCache,
                ref statistics,
                ref incrementalStatistics);
            ActiveIncidentIndexState.Value = default;
            EnsureActiveConstraintIncidentIndexP1P6();

            if (iterationIndex == math.max(1, IterationCount) - 1)
            {
                ResetTimestepContactSetForSubstep();
                runtime.RecoveryRequired = 1;
            }
        }

        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incrementalStatistics);
        runtimeState.Value = runtime;
    }
}
}
