using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;
namespace RTS.Unit.FlowField.Jobs
{
public partial struct ConstraintSolverJob
{
    private void BeginP1P6Iteration(
        int substepIndex,
        NativeReference<ParallelJacobiExecutionState> runtimeState
#if RTS_CONTACT_DIAGNOSTICS
        , NativeReference<ParallelJacobiIterationTelemetry> iterationState
#endif
        )
    {
        if (runtimeState.Value.IsValid == 0)
            return;
#if RTS_CONTACT_DIAGNOSTICS
        ParallelJacobiIterationTelemetry iteration = default;
        if (EnableDiagnostics)
        {
            MeasureContactResidual(
                out iteration.MaxViolationBeforeSolve,
                out iteration.AverageViolationBeforeSolve);
        }
#endif
        ResetCorrectedBodyTracking();
#if RTS_CONTACT_DIAGNOSTICS
        iterationState.Value = iteration;
#endif
    }


    private void BeginP1P6FinalizeSubstep(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        statistics.IterationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - runtime.IterationStartTimestamp);
        AccumulateConstraintStatistics(ref statistics, ref runtime.PenetrationSum);
        StoreContactStatistics(statistics);
        runtimeState.Value = runtime;
    }

    private void FinalizeP1P6VelocityStatistics(
        NativeReference<ParallelJacobiExecutionState> runtimeState,
        int blockCount)
    {
        if (runtimeState.Value.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        float speedBefore = 0f;
        float speedAfter = 0f;
        int count = 0;
        for (int blockIndex = 0; blockIndex < blockCount; blockIndex++)
        {
            int bodyIndex = blockIndex * CrowdContactPipelineScheduler.ParallelBodyBatchSize;
            ParallelBodyStageResult body = ParallelBodyStatistics[bodyIndex];
            statistics.TotalVelocityChange += body.Total;
            statistics.MaxVelocityChange = math.max(statistics.MaxVelocityChange, body.Maximum);
            speedBefore += body.SecondaryTotal;
            speedAfter += body.TertiaryTotal;
            count += body.Count;
        }
        if (count > 0)
        {
            statistics.AverageSpeedBeforeContact += speedBefore / count;
            statistics.AverageSpeedAfterContact += speedAfter / count;
        }
        StoreContactStatistics(statistics);
    }

}
}
