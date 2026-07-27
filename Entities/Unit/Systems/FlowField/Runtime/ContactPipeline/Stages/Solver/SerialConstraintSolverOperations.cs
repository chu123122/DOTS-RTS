using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct ConstraintSolverJob
{
    private void ExecuteSolveWallSerial()
    {
        SerialContactPipelineControlState control = SerialControl.Value;
        if (control.IsValid == 0)
            return;
#if RTS_CONTACT_DIAGNOSTICS
        if (EnableDiagnostics)
        {
            MeasureContactResidual(
                out control.MaxViolationBeforeSolve,
                out control.AverageViolationBeforeSolve);
        }
#endif
        SolveWallConstraintIteration(
            true,
            out control.TotalWallPositionCorrection,
            out control.MaxWallPositionCorrection);
        SerialControl.Value = control;
    }

    private void ExecuteSolveContactSerial(bool recoveryOnly)
    {
        SerialContactPipelineControlState control = SerialControl.Value;
        if (control.IsValid == 0)
            return;
        if (recoveryOnly && control.RecoveryRequired == 0)
            return;
        if (recoveryOnly)
            ResetTimestepContactSetForSubstep();
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        float dt = DeltaTime / math.max(1, SubstepCount);
        SolveConfiguredContactIteration(
            dt,
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
        if (!recoveryOnly)
        {
            statistics.TotalWallPositionCorrection +=
                control.TotalWallPositionCorrection;
            statistics.MaxWallPositionCorrection = math.max(
                statistics.MaxWallPositionCorrection,
                control.MaxWallPositionCorrection);
#if RTS_CONTACT_DIAGNOSTICS
            if (EnableDiagnostics)
            {
                RecordIterationDiagnostic(
                    SubstepIndex,
                    IterationIndex,
                    control.MaxViolationBeforeSolve,
                    control.AverageViolationBeforeSolve,
                    totalCorrection,
                    maxCorrection,
                    control.TotalWallPositionCorrection,
                    control.MaxWallPositionCorrection);
            }
#endif
        }
        else
        {
            control.RecoveryRequired = 0;
        }
        control.TotalContactPositionCorrection = totalCorrection;
        control.MaxContactPositionCorrection = maxCorrection;
        SerialControl.Value = control;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void ExecuteFinalizeSerialSubstep()
    {
        SerialContactPipelineControlState control = SerialControl.Value;
        if (control.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
#if RTS_CONTACT_DIAGNOSTICS
        // Iteration timing + constraint counters: no EnableDiagnostics gate so
        // benchmarks (diagnostics off, oracle off) still capture valid numbers.
        statistics.IterationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - control.IterationStartTimestamp);
        AccumulateConstraintStatistics(ref statistics, ref control.PenetrationSum);
#endif
        SerialControl.Value = control;
        StoreContactStatistics(statistics);
    }

    private void ExecuteFinalizeSerialPipeline()
    {
        SerialContactPipelineControlState control = SerialControl.Value;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
#if RTS_CONTACT_DIAGNOSTICS
        if (EnableDiagnostics)
            CaptureSelectedBodyAndPairs(math.max(0, SubstepCount - 1));
        BuildContactHeatSamples();
#endif
        statistics.AveragePenetration = statistics.PenetratingPairCount > 0
            ? control.PenetrationSum / statistics.PenetratingPairCount
            : 0f;
        statistics.UnactivatedPairCount =
            statistics.ContactPairCount - statistics.ActiveConstraintCount;
        statistics.PredictiveUnactivatedCount =
            statistics.PredictivePairCount - statistics.PredictiveActivatedCount;
        statistics.UnactivatedRatio = statistics.ContactPairCount > 0
            ? (float)statistics.UnactivatedPairCount / statistics.ContactPairCount
            : 0f;
        statistics.PredictiveUnactivatedRatio = statistics.PredictivePairCount > 0
            ? (float)statistics.PredictiveUnactivatedCount / statistics.PredictivePairCount
            : 0f;
        statistics.AverageIterationNanoseconds =
            statistics.IterationNanoseconds / math.max(1, SubstepCount * IterationCount);
        statistics.AverageSoftAvoidanceNanoseconds =
            statistics.SoftAvoidanceNanoseconds / math.max(1, SubstepCount);
        statistics.AverageSpeedBeforeContact /= math.max(1, SubstepCount);
        statistics.AverageSpeedAfterContact /= math.max(1, SubstepCount);
#if RTS_CONTACT_DIAGNOSTICS
        // Solver timing: no EnableDiagnostics gate so benchmarks capture valid
        // total-solve time with the oracle disabled.
        statistics.SolverNanoseconds = ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - control.SolverStartTimestamp);
        CountFinalContactSetUtilization(
            out statistics.TimestepContactSetUniqueActivatedPairCount,
            out incremental.UniqueCorrectedPairCount);
#endif
        incremental.UniqueActivatedPairCount =
            statistics.TimestepContactSetUniqueActivatedPairCount;
        incremental.CurrentSweptContactCount =
            incremental.CurrentDormantPairCount +
            incremental.CurrentApproachingPairCount +
            incremental.CurrentPredictivePairCount +
            incremental.CurrentActualPairCount;
        incremental.CurrentActiveConstraintCount = TimestepContactPairs.Length;
        incremental.PeakActiveConstraintCount = math.max(
            incremental.PeakActiveConstraintCount,
            incremental.CurrentActiveConstraintCount);
        incremental.CleanProxyRatio = incremental.ProxyCount > 0
            ? 1f - math.saturate(
                (float)incremental.TopologyDirtyBodyCount / incremental.ProxyCount)
            : 0f;
        incremental.RetainedNeighborPairRatio =
            incremental.PersistentNeighborPairCount > 0
                ? math.saturate(
                    (float)incremental.NeighborPairRetainedCount /
                    incremental.PersistentNeighborPairCount)
                : 0f;
        incremental.NeighborToSweptRatio =
            incremental.PersistentNeighborPairCount > 0
                ? math.saturate(
                    (float)incremental.CurrentSweptContactCount /
                    incremental.PersistentNeighborPairCount)
                : 0f;
        incremental.SweptToCurrentActiveRatio =
            incremental.CurrentSweptContactCount > 0
                ? math.saturate(
                    (float)incremental.CurrentActiveConstraintCount /
                    incremental.CurrentSweptContactCount)
                : 0f;
        incremental.ActivatedToCorrectedRatio =
            incremental.UniqueActivatedPairCount > 0
                ? math.saturate(
                    (float)incremental.UniqueCorrectedPairCount /
                    incremental.UniqueActivatedPairCount)
                : 0f;
#if RTS_CONTACT_DIAGNOSTICS
        if (EnableDiagnostics)
            CaptureSimulationDebuggerSelectedUnit();
#endif
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }

    private void ExecuteParallelRecovery()
    {
        ParallelJacobiExecutionState runtime = RuntimeState.Value;
        if (runtime.IsValid == 0 || runtime.RecoveryRequired == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        float dt = DeltaTime / math.max(1, SubstepCount);
        SolveJacobiContactIteration(
            dt,
            SubstepIndex,
            true,
            ref statistics,
            ref incremental,
            out float correction,
            out float maxCorrection);
        statistics.TotalContactPositionCorrection += correction;
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
