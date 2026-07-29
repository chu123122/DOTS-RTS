using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;
namespace RTS.Unit.FlowField.Jobs
{
public partial struct InteractionCertificationAlgorithms
{
    private void FinalizeContactIteration(
        int substepIndex,
        int iterationIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState
#if RTS_CONTACT_DIAGNOSTICS
        , NativeReference<ContactSolverIterationTelemetry> iterationState,
        NativeList<JacobiBlockTelemetry> blockStatistics
#endif
        )
    {
        ContactPipelineExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incrementalStatistics = LoadIncrementalStatistics();
#if RTS_CONTACT_DIAGNOSTICS
        ContactSolverIterationTelemetry iteration = iterationState.Value;
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
        float totalPositionCorrection =
            iteration.TotalContactPositionCorrection;
        float maxPositionCorrection =
            iteration.MaxContactPositionCorrection;
        if (ContactPositionSolver == ContactPositionSolverMode.Jacobi)
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
                TimestepContactPairs.Length;
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

            if (iterationIndex == math.max(1, IterationCount) - 1)
            {
                ResetTimestepContactSetForSubstep();
                runtime.RecoveryRequired = 1;
            }
        }

        // Unconditionally rebuild the incident index against the current
        // TimestepContactPairs. The repair path above (or an upstream escaped-view
        // rebuild) can change the pair count; the fingerprint fast-path in Ensure
        // can otherwise leave a stale index that the next iteration's
        // GatherAndApply indexes out of range. Match FinalizeWallIteration.
        Solver.ActiveIncidentIndexState.Value = default;
        EnsureActiveConstraintIncidentIndex();

        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incrementalStatistics);
        runtimeState.Value = runtime;
    }
}
}
