using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{

/// <summary>
/// Frostbite-inspired Predictive Disc Contact 求解器。
/// 顶层跨帧拓扑或全量 Sweep 统一生成中层 InteractionSet；
/// 中层可跨 substep 复用，并派生 Soft 候选与 XPBD ContactSet。
/// 约束修正逃出 timestep envelope 时，针对剩余时间执行完整 Broad Phase 回退。
/// 缓存、软避让、Broad Phase 与诊断实现分布在同一 partial Job 的独立文件中。
/// </summary>
[BurstCompile]
public partial struct SolveXpbdUnitContactsJob : IJob
{
    public ContactPipelineConfiguration Configuration;

    private float DeltaTime => Configuration.DeltaTime;
    private int SubstepCount => Configuration.SubstepCount;
    private int IterationCount => Configuration.IterationCount;
    private ContactPositionSolverMode ContactPositionSolver =>
        Configuration.ContactPositionSolver;
    private float Compliance => Configuration.Compliance;
    private float PredictiveSkin => Configuration.PredictiveSkin;
    private float SoftAvoidanceResponseRate => Configuration.SoftAvoidanceResponseRate;
    private float SoftAvoidanceShell => Configuration.SoftAvoidanceShell;
    private float SettledSoftAvoidanceMultiplier => Configuration.SettledSoftAvoidanceMultiplier;
    private SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver =>
        Configuration.SoftAvoidanceVelocitySolver;
    private float RvoTimeHorizon => Configuration.RvoTimeHorizon;
    private bool EnablePredictivePairGeneration => Configuration.EnablePredictivePairGeneration;
    private bool EnablePredictiveContacts => Configuration.EnablePredictiveContacts;
    private bool EnableDiagnostics => Configuration.EnableDiagnostics;
    private bool EnablePersistentContactCache => Configuration.EnablePersistentContactCache;
    private bool EnableTimestepContactSetCache => Configuration.EnableTimestepContactSetCache;
    private float GuardEnvelopeMargin => Configuration.GuardEnvelopeMargin;
    private float TimestepContactMargin => Configuration.TimestepContactMargin;
    public Entity DiagnosticSelectedEntity;

    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    // Broad Phase 与软避让共用 Pairs 作为瞬时 scratch；权威接触必须独立保存。
    public NativeList<UnitCollisionPair> Pairs;
    public NativeList<UnitCollisionPair> TimestepContactPairs;
    public NativeParallelHashMap<Entity, int> CurrentBodyIndexByEntity;
    // Snapshot of the previous finalized timestep view, used only to preserve
    // activation/fallback history while rebuilding the current view.
    public NativeList<UnitCollisionPair> PreviousTimestepContactPairs;
    // 中层跨子步 InteractionSet。跨帧缓存与全量 Sweep 只能作为它的两种来源；
    // Soft Avoidance 和 XPBD 不得直接读取任何跨帧持久容器。
    public NativeList<UnitCollisionPair> TimestepInteractionPairs;
    // B 层 InteractionSet 的 Soft/RVO 紧凑派生视图。
    public NativeList<UnitCollisionPair> SoftAvoidancePairs;
    public NativeArray<byte> CorrectedBodyFlags;
    public NativeList<int> CorrectedBodyIndices;
    public NativeArray<int> ActiveIncidentOffsets;
    public NativeArray<int> ActiveIncidentWriteCursors;
    public NativeList<int> ActiveIncidentPairIndices;
    public NativeList<JacobiPairCorrection> JacobiPairCorrections;
    public NativeList<PersistentSweptProxy> CurrentIncrementalProxies;
    public NativeList<PersistentSweptProxy> PersistentSweptProxies;
    public NativeList<int> PersistentProxyIndexByBody;
    public NativeList<PersistentNeighborPair> PersistentNeighborPairs;
    public NativeList<PersistentPredictiveContact> PersistentPredictiveContacts;
    public NativeList<StableEntityPairKey> PersistentActiveContactKeys;
    public NativeList<StableEntityPairKey> PersistentSoftAvoidancePairKeys;
    public NativeList<PredictiveContactScheduleEntry> PersistentDormantContactSchedule;
    public NativeList<PersistentPredictiveContact> PredictiveContactScratch;
    public NativeList<IncrementalDirtyBody> IncrementalDirtyBodies;
    public NativeArray<byte> IncrementalDirtyFlagsByBody;
    public NativeList<PersistentNeighborPair> IncrementalNeighborPairScratch;
    // 仅在诊断校验中使用：保存当前帧的增量管线接触对，避免与求解 scratch 混用。
    public NativeList<UnitCollisionPair> IncrementalOracleContactPairs;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactSchedule;
    public NativeList<PredictiveContactScheduleEntry> PredictiveContactScheduleScratch;
    public NativeReference<int> PredictiveContactScheduleCursor;
    public NativeReference<IncrementalContactCacheState> IncrementalCacheState;
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
    public NativeArray<FlowMovementFrameState> States;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<Stage3ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<Stage3ContactPairDiagnostic> PairDiagnostics;
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBodyDiagnostic;
    public NativeArray<Stage3ContactHeatSample> HeatSamples;

    public void Execute()
    {
        long solverStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        int substepCount = math.max(1, SubstepCount);
        int iterationCount = math.max(1, IterationCount);
        float substepDeltaTime = DeltaTime / substepCount;
        var statistics = new PredictiveDiscContactStatistics();
        statistics.TimestepContactSetFirstEscapeSubstep = -1;
        var incrementalStatistics = new IncrementalContactPipelineStatistics();
        float penetrationSum = 0f;
        IterationDiagnostics.Clear();
        PairDiagnostics.Clear();
        SelectedBodyDiagnostic.Value = default;
        ResetSimulationDebuggerCapture();

        if (substepDeltaTime <= 0f)
        {
            Statistics.Value = statistics;
            IncrementalStatistics.Value = incrementalStatistics;
            return;
        }
        if (!EnablePersistentContactCache)
        {
            PersistentSweptProxies.Clear();
            PersistentProxyIndexByBody.Clear();
            PersistentNeighborPairs.Clear();
            PersistentPredictiveContacts.Clear();
            IncrementalCacheState.Value = default;
        }

        if (EnableTimestepContactSetCache)
        {
            PrepareTimestepContactPrediction(DeltaTime, false);
            if (EnablePersistentContactCache)
                PrepareInitialPersistentDirtyBodySet();
            long initialContactSetStart = ProfilerUnsafeUtility.Timestamp;
            BuildTimestepContactSet(
                ref statistics,
                ref incrementalStatistics,
                false,
                false);
            statistics.PairGenerationNanoseconds += TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - initialContactSetStart);
        }

        for (int substepIndex = 0; substepIndex < substepCount; substepIndex++)
        {
            PrepareBaseVelocitiesForSubstep(substepDeltaTime);
            if (!EnableTimestepContactSetCache)
            {
                // B0：每个 substep 重新生产同一种 InteractionSet；A 在该组合中
                // 被上层配置强制关闭，因此这里只会执行全量统一 Sweep。
                PrepareTimestepContactPrediction(substepDeltaTime, true);
                long substepInteractionStart = ProfilerUnsafeUtility.Timestamp;
                BuildSubstepInteractionSet(
                    ref statistics,
                    ref incrementalStatistics);
                statistics.PairGenerationNanoseconds += TimestampToNanoseconds(
                    ProfilerUnsafeUtility.Timestamp - substepInteractionStart);
            }
            else if (!ValidateBaseMotionInteractionEnvelope(
                         substepIndex,
                         ref statistics,
                         ref incrementalStatistics))
            {
                // RVO/base-velocity changes occur before position prediction.
                // Rebuild now so Soft Avoidance never consumes a stale view.
                RepairOrRebuildContactViewForRemainingTime(
                    substepIndex,
                    substepCount,
                    substepDeltaTime,
                    true,
                    ref statistics,
                    ref incrementalStatistics,
                    false);
            }

            long softAvoidanceStart = ProfilerUnsafeUtility.Timestamp;
            CalculateSoftAvoidanceForSubstep(
                substepDeltaTime,
                ref statistics,
                ref incrementalStatistics);
            ClampSoftOutputToInteractionEnvelope(
                substepDeltaTime,
                ref incrementalStatistics);
            statistics.SoftAvoidanceEvaluationCount++;
            statistics.SoftAvoidanceNanoseconds +=
                TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - softAvoidanceStart);

            PredictUnconstrainedPositions(substepDeltaTime);
            bool rebuiltPredictedContactView = false;
            if (!ValidatePredictedContactEnvelope(
                    substepIndex,
                    ref statistics,
                    ref incrementalStatistics))
            {
                RepairOrRebuildContactViewForRemainingTime(
                    substepIndex,
                    substepCount,
                    substepDeltaTime,
                    EnableTimestepContactSetCache,
                    ref statistics,
                    ref incrementalStatistics,
                    false);
                rebuiltPredictedContactView = true;
            }
            if (!EnableTimestepContactSetCache && !rebuiltPredictedContactView)
            {
                PrepareSubstepContactPrediction();
                long substepContactViewStart = ProfilerUnsafeUtility.Timestamp;
                BuildSubstepContactView(
                    ref statistics,
                    ref incrementalStatistics);
                statistics.PairGenerationNanoseconds += TimestampToNanoseconds(
                    ProfilerUnsafeUtility.Timestamp - substepContactViewStart);
            }
            ActivateScheduledPredictiveContactsForSubstep(
                EnableTimestepContactSetCache ? substepIndex : 0,
                EnableTimestepContactSetCache ? substepCount : 1,
                ref incrementalStatistics);
            ResetTimestepContactSetForSubstep();
            RebuildActiveConstraintIncidentIndexIfNeeded();
            statistics.TimestepContactSetSubstepUseCount++;

            long iterationStart = ProfilerUnsafeUtility.Timestamp;
            for (int iterationIndex = 0; iterationIndex < iterationCount; iterationIndex++)
            {
                float maxViolationBeforeSolve = 0f;
                float averageViolationBeforeSolve = 0f;
                if (EnableDiagnostics)
                {
                    MeasureContactResidual(
                        out maxViolationBeforeSolve,
                        out averageViolationBeforeSolve);
                }

                SolveWallConstraintIteration(
                    true,
                    out float totalWallPositionCorrection,
                    out float maxWallPositionCorrection);

                if (!ValidateSolverCorrectionContactEnvelope(
                        substepIndex,
                        ref statistics,
                        ref incrementalStatistics))
                {
                    RepairOrRebuildContactViewForRemainingTime(
                        substepIndex,
                        substepCount,
                        substepDeltaTime,
                        EnableTimestepContactSetCache,
                        ref statistics,
                        ref incrementalStatistics);
                    ResetTimestepContactSetForSubstep();
                    RebuildActiveConstraintIncidentIndexIfNeeded();
                }

                SolveConfiguredContactIteration(
                    substepDeltaTime,
                    substepIndex,
                    true,
                    ref statistics,
                    ref incrementalStatistics,
                    out float totalPositionCorrection,
                    out float maxPositionCorrection);

                statistics.TotalContactPositionCorrection += totalPositionCorrection;
                statistics.MaxContactPositionCorrection = math.max(
                    statistics.MaxContactPositionCorrection,
                    maxPositionCorrection);
                statistics.TotalWallPositionCorrection += totalWallPositionCorrection;
                statistics.MaxWallPositionCorrection = math.max(
                    statistics.MaxWallPositionCorrection,
                    maxWallPositionCorrection);

                if (EnableDiagnostics)
                {
                    RecordIterationDiagnostic(
                        substepIndex,
                        iterationIndex,
                        maxViolationBeforeSolve,
                        averageViolationBeforeSolve,
                        totalPositionCorrection,
                        maxPositionCorrection,
                        totalWallPositionCorrection,
                        maxWallPositionCorrection);
                }

                if (!ValidateSolverCorrectionContactEnvelope(
                        substepIndex,
                        ref statistics,
                        ref incrementalStatistics))
                {
                    RepairOrRebuildContactViewForRemainingTime(
                        substepIndex,
                        substepCount,
                        substepDeltaTime,
                        EnableTimestepContactSetCache,
                        ref statistics,
                        ref incrementalStatistics);
                    RebuildActiveConstraintIncidentIndexIfNeeded();

                    // 最后一轮之后没有正常恢复机会；补一轮只处理新发现的单位接触。
                    if (iterationIndex == iterationCount - 1)
                    {
                        ResetTimestepContactSetForSubstep();
                        SolveConfiguredContactIteration(
                            substepDeltaTime,
                            substepIndex,
                            true,
                            ref statistics,
                            ref incrementalStatistics,
                            out float recoveryCorrection,
                            out float recoveryMaxCorrection);
                        statistics.TotalContactPositionCorrection += recoveryCorrection;
                        statistics.MaxContactPositionCorrection = math.max(
                            statistics.MaxContactPositionCorrection,
                            recoveryMaxCorrection);
                    }
                }
            }
            statistics.IterationNanoseconds +=
                TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - iterationStart);

            AccumulateConstraintStatistics(ref statistics, ref penetrationSum);
            ReconstructVelocities(substepDeltaTime, ref statistics);

        }

        if (EnableDiagnostics)
            CaptureSelectedBodyAndPairs(substepCount - 1);
        BuildContactHeatSamples();


        statistics.AveragePenetration = statistics.PenetratingPairCount > 0
            ? penetrationSum / statistics.PenetratingPairCount
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
        statistics.SolverNanoseconds =
            TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - solverStartTimestamp);
        incrementalStatistics.UniqueActivatedPairCount =
            statistics.TimestepContactSetUniqueActivatedPairCount;
        // Keep the gauge independent from the path that produced the current
        // views. Expired persistent neighbors are not swept contacts.
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
        incrementalStatistics.CleanProxyRatio =
            incrementalStatistics.ProxyCount > 0
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
        // 记录本时间步的接触/修正结果，供下一帧热力图快照读取。
        CaptureSimulationDebuggerSelectedUnit();
        Statistics.Value = statistics;
        IncrementalStatistics.Value = incrementalStatistics;
    }


















}


}
