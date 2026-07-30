using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;
namespace RTS.Unit.FlowField.Jobs
{
public partial struct ConstraintSolverJob
{
    private void PrepareJacobiRecovery()
    {
        ContactPipelineExecutionState runtime = RuntimeState.Value;
        if (runtime.IsValid == 0 || runtime.RecoveryRequired == 0)
            return;

        ResetCorrectedBodyTracking();
        JacobiPairCorrections.ResizeUninitialized(TimestepContactPairs.Length);
#if RTS_CONTACT_DIAGNOSTICS
        BlockStatistics.ResizeUninitialized(
            (TimestepContactPairs.Length +
             CrowdContactPipelineScheduler.JacobiPairBatchSize - 1) /
            CrowdContactPipelineScheduler.JacobiPairBatchSize);
#endif
    }

    private void FinalizeJacobiRecovery()
    {
        ContactPipelineExecutionState runtime = RuntimeState.Value;
        if (runtime.IsValid == 0 || runtime.RecoveryRequired == 0)
            return;

#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental =
            LoadIncrementalStatistics();
        float totalCorrection = 0f;
        float maxCorrection = 0f;
        int newlyActivated = 0;
        int newlyCorrected = 0;
        for (int blockIndex = 0;
             blockIndex < BlockStatistics.Length;
             blockIndex++)
        {
            JacobiBlockTelemetry block = BlockStatistics[blockIndex];
            totalCorrection += block.TotalPositionCorrection;
            maxCorrection = math.max(
                maxCorrection,
                block.MaxPositionCorrection);
            newlyActivated += block.NewlyActivatedPairCount;
            newlyCorrected += block.NewlyCorrectedPairCount;
        }

        statistics.TimestepContactSetUniqueActivatedPairCount += newlyActivated;
        statistics.TotalContactPositionCorrection += totalCorrection;
        statistics.MaxContactPositionCorrection = math.max(
            statistics.MaxContactPositionCorrection,
            maxCorrection);
        incremental.UniqueCorrectedPairCount += newlyCorrected;
        incremental.ActiveConstraintEvaluationCount +=
            TimestepContactPairs.Length;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
#endif
        runtime.RecoveryRequired = 0;
        RuntimeState.Value = runtime;
    }

    private void InitializeContactIteration(
        int substepIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState
#if RTS_CONTACT_DIAGNOSTICS
        , NativeReference<ContactSolverIterationTelemetry> iterationState
#endif
        )
    {
        if (runtimeState.Value.IsValid == 0)
            return;
#if RTS_CONTACT_DIAGNOSTICS
        ContactSolverIterationTelemetry iteration = default;
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


#if RTS_CONTACT_DIAGNOSTICS
    private void FinalizeSubstepTelemetry(
        NativeReference<ContactPipelineExecutionState> runtimeState)
    {
        // No EnableDiagnostics gate: this path only accumulates iteration
        // timing and constraint counters (ActiveConstraintCount etc.), both
        // cheap. Gating it would starve benchmarks of perf numbers when the
        // oracle is disabled.
        ContactPipelineExecutionState runtime = runtimeState.Value;
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
            ContactPipelineDiagnosticsMath.AccountedCandidateNanoseconds(
                incremental) -
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
    private void FinalizeVelocityStatistics(
        NativeReference<ContactPipelineExecutionState> runtimeState,
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

}
}
