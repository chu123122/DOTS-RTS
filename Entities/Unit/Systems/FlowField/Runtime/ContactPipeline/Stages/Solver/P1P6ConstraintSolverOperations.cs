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
        if (EnableDiagnostics)
            iterationState.Value = iteration;
#endif
    }


#if RTS_CONTACT_DIAGNOSTICS
    private void BeginP1P6FinalizeSubstep(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        // No EnableDiagnostics gate: this path only accumulates iteration
        // timing and constraint counters (ActiveConstraintCount etc.), both
        // cheap. Gating it would starve benchmarks of perf numbers when the
        // oracle is disabled.
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        long elapsed = ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - runtime.IterationStartTimestamp);
        IncrementalContactPipelineStatistics incremental =
            LoadIncrementalStatistics();
        // Escape repair can rebuild/reclassify pairs between solver rounds.
        // Those costs belong to Broad/Narrow/Activation, not to both buckets.
        long nestedCandidateNanoseconds =
            AccountedCandidateNanoseconds(incremental) -
            runtime.IterationAccountedStartNanoseconds;
        statistics.IterationNanoseconds += math.max(
            0L,
            elapsed - math.max(0L, nestedCandidateNanoseconds));
        AccumulateConstraintStatistics(ref statistics, ref runtime.PenetrationSum);
        StoreContactStatistics(statistics);
        runtimeState.Value = runtime;
    }
#endif

#if RTS_CONTACT_DIAGNOSTICS
    private void FinalizeP1P6VelocityStatistics(
        NativeReference<ParallelJacobiExecutionState> runtimeState,
        int blockCount)
    {
        // No EnableDiagnostics gate: velocity-change stats are cheap and needed
        // by benchmarks with the oracle disabled.
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
#endif

#if RTS_CONTACT_DIAGNOSTICS
    private static long AccountedCandidateNanoseconds(
        IncrementalContactPipelineStatistics statistics) =>
        statistics.ProxyValidationNanoseconds +
        statistics.FullSweepSourceNanoseconds +
        statistics.PersistentPairMappingNanoseconds +
        statistics.LocalBroadPhaseNanoseconds +
        statistics.PairDiffNanoseconds +
        statistics.FallbackNanoseconds +
        statistics.SweptClassificationNanoseconds +
        statistics.ContactActivationNanoseconds;
#endif

}
}
