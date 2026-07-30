using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct ConstraintSolverJob
{
    private void ExecuteSolveGaussSeidelContact()
    {
        ContactPipelineExecutionState runtime = RuntimeState.Value;
        if (runtime.IsValid == 0)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental =
            LoadIncrementalStatistics();
        float substepDeltaTime = DeltaTime / math.max(1, SubstepCount);
        SolveGaussSeidelContactIteration(
            substepDeltaTime,
            SubstepIndex,
            true,
            ref statistics,
            ref incremental,
            out float totalCorrection,
            out float maxCorrection);
        statistics.TotalContactPositionCorrection += totalCorrection;
        statistics.MaxContactPositionCorrection = math.max(
            statistics.MaxContactPositionCorrection,
            maxCorrection);
#if RTS_CONTACT_DIAGNOSTICS
        ContactSolverIterationTelemetry iteration = IterationState.Value;
        iteration.TotalContactPositionCorrection = totalCorrection;
        iteration.MaxContactPositionCorrection = maxCorrection;
        IterationState.Value = iteration;
#endif
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void ExecuteGaussSeidelRecovery()
    {
        ContactPipelineExecutionState runtime = RuntimeState.Value;
        if (runtime.IsValid == 0 || runtime.RecoveryRequired == 0)
            return;

        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental =
            LoadIncrementalStatistics();
        float substepDeltaTime = DeltaTime / math.max(1, SubstepCount);
        SolveGaussSeidelContactIteration(
            substepDeltaTime,
            SubstepIndex,
            true,
            ref statistics,
            ref incremental,
            out float totalCorrection,
            out float maxCorrection);
        statistics.TotalContactPositionCorrection += totalCorrection;
        statistics.MaxContactPositionCorrection = math.max(
            statistics.MaxContactPositionCorrection,
            maxCorrection);
        runtime.RecoveryRequired = 0;
        RuntimeState.Value = runtime;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }
}
}
