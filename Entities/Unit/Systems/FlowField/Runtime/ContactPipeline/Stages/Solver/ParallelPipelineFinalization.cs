using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;
namespace RTS.Unit.FlowField.Jobs
{
public partial struct ConstraintSolverJob
{
    private void FinalizeParallelJacobiPipeline(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        ParallelJacobiExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;

#if RTS_CONTACT_DIAGNOSTICS
        int substepCount = math.max(1, SubstepCount);
        int iterationCount = math.max(1, IterationCount);
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incrementalStatistics = LoadIncrementalStatistics();

        if (EnableDiagnostics)
            CaptureSelectedBodyAndPairs(substepCount - 1);
        BuildContactHeatSamples();
        statistics.AveragePenetration = statistics.PenetratingPairCount > 0
            ? runtime.PenetrationSum / statistics.PenetratingPairCount
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
            statistics.IterationNanoseconds / math.max(1, substepCount * iterationCount);
        statistics.AverageSoftAvoidanceNanoseconds =
            statistics.SoftAvoidanceNanoseconds / substepCount;
        statistics.AverageSpeedBeforeContact /= substepCount;
        statistics.AverageSpeedAfterContact /= substepCount;
        // Solver timing: no EnableDiagnostics gate so benchmarks capture valid
        // total-solve time with the oracle disabled.
        statistics.SolverNanoseconds = ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - runtime.SolverStartTimestamp);

        incrementalStatistics.UniqueActivatedPairCount =
            statistics.TimestepContactSetUniqueActivatedPairCount;
        incrementalStatistics.CurrentSweptContactCount =
            incrementalStatistics.CurrentDormantPairCount +
            incrementalStatistics.CurrentApproachingPairCount +
            incrementalStatistics.CurrentPredictivePairCount +
            incrementalStatistics.CurrentActualPairCount;
        incrementalStatistics.CurrentActiveConstraintCount =
            TimestepContactPairs.Length;
        incrementalStatistics.PeakActiveConstraintCount = math.max(
            incrementalStatistics.PeakActiveConstraintCount,
            incrementalStatistics.CurrentActiveConstraintCount);
        incrementalStatistics.CleanProxyRatio = incrementalStatistics.ProxyCount > 0
            ? 1f - math.saturate(
                (float)incrementalStatistics.TopologyDirtyBodyCount /
                incrementalStatistics.ProxyCount)
            : 0f;
        incrementalStatistics.RetainedNeighborPairRatio =
            incrementalStatistics.PersistentNeighborPairCount > 0
                ? math.saturate(
                    (float)incrementalStatistics.NeighborPairRetainedCount /
                    incrementalStatistics.PersistentNeighborPairCount)
                : 0f;
        incrementalStatistics.NeighborToSweptRatio =
            incrementalStatistics.PersistentNeighborPairCount > 0
                ? math.saturate(
                    (float)incrementalStatistics.CurrentSweptContactCount /
                    incrementalStatistics.PersistentNeighborPairCount)
                : 0f;
        incrementalStatistics.SweptToCurrentActiveRatio =
            incrementalStatistics.CurrentSweptContactCount > 0
                ? math.saturate(
                    (float)incrementalStatistics.CurrentActiveConstraintCount /
                    incrementalStatistics.CurrentSweptContactCount)
                : 0f;
        incrementalStatistics.ActivatedToCorrectedRatio =
            incrementalStatistics.UniqueActivatedPairCount > 0
                ? math.saturate(
                    (float)incrementalStatistics.UniqueCorrectedPairCount /
                    incrementalStatistics.UniqueActivatedPairCount)
                : 0f;

        if (EnableDiagnostics)
            CaptureSimulationDebuggerSelectedUnit();
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incrementalStatistics);
#endif
    }
}
}
