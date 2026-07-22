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
    public NativeList<PersistentSweptProxy> CurrentIncrementalProxies;
    public NativeList<PersistentSweptProxy> PersistentSweptProxies;
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
            PersistentNeighborPairs.Clear();
            PersistentPredictiveContacts.Clear();
            IncrementalCacheState.Value = default;
        }

        if (EnableTimestepContactSetCache)
        {
            PrepareTimestepContactPrediction(DeltaTime, false);
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
                }

                SolveContactIteration(
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

                    // 最后一轮之后没有正常恢复机会；补一轮只处理新发现的单位接触。
                    if (iterationIndex == iterationCount - 1)
                    {
                        ResetTimestepContactSetForSubstep();
                        SolveContactIteration(
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





    private void PredictUnconstrainedPositions(float substepDeltaTime)
    {
        for (int i = 0; i < States.Length; i++)
        {
            FlowMovementFrameState state = States[i];
            if (!state.IsInsideGrid)
                continue;

            // StartPosition 保存本 substep 的可信相对分离关系，不冻结实体位置。
            state.StartPosition = state.PredictedPosition;
            state.PreviousSubstepPosition = state.StartPosition;
            state.ContactPositionCorrection = float3.zero;
            state.WallPositionCorrection = float3.zero;

            float3 velocity = state.BasePredictedVelocity;
            float responseRate = math.max(0f, SoftAvoidanceResponseRate);
            if (state.IsSettled)
                responseRate *= math.max(0f, SettledSoftAvoidanceMultiplier);
            velocity = SoftAvoidanceMath.ApplyVelocityBuffer(
                velocity,
                state.SoftAvoidanceVelocity,
                responseRate,
                substepDeltaTime,
                state.MoveSpeed);
            if (state.IsSettled)
                velocity *= math.pow(0.8f, substepDeltaTime * 60f);

            if (math.lengthsq(velocity) > state.MoveSpeed * state.MoveSpeed)
                velocity = math.normalizesafe(velocity) * state.MoveSpeed;

            state.PredictedPosition = state.StartPosition + velocity * substepDeltaTime;
            state.PredictedPosition.y = state.CurrentPosition.y;
            state.UnconstrainedPredictedPosition = state.PredictedPosition;
            state.VelocityBeforeContact = velocity;
            state.IntegratedVelocity = velocity;
            States[i] = state;
        }
    }

    private void PrepareBaseVelocitiesForSubstep(float substepDeltaTime)
    {
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;

            state.BasePredictedVelocity = CalculateBaseVelocityForSubstep(
                state,
                substepDeltaTime);
            States[bodyIndex] = state;
        }
    }

    private float3 CalculateBaseVelocityForSubstep(
        FlowMovementFrameState state,
        float substepDeltaTime)
    {
        float3 totalForce = state.IndependentForce;
        if (state.Cell.Cost == 0 && math.lengthsq(totalForce) < 0.1f)
        {
            float3 cellCenter = GridOrigin + new float3(
                state.CellPosition.x * CellRadius * 2 + CellRadius,
                state.CurrentPosition.y,
                state.CellPosition.y * CellRadius * 2 + CellRadius);
            float3 escapeDirection = state.PredictedPosition - cellCenter;
            escapeDirection.y = 0;
            escapeDirection = math.normalizesafe(escapeDirection, new float3(1, 0, 0));
            totalForce += escapeDirection * state.MoveSpeed * 5f;
        }

        if (math.lengthsq(totalForce) > state.MaxForce * state.MaxForce)
            totalForce = math.normalizesafe(totalForce) * state.MaxForce;

        return state.IntegratedVelocity + totalForce * substepDeltaTime;
    }

    private void SolveContactIteration(
        float substepDeltaTime,
        int substepIndex,
        bool trackCorrectedBodies,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        out float totalPositionCorrection,
        out float maxPositionCorrection)
    {
        if (trackCorrectedBodies)
            ResetCorrectedBodyTracking();

        totalPositionCorrection = 0f;
        maxPositionCorrection = 0f;
        float alpha = Compliance / (substepDeltaTime * substepDeltaTime);
        incrementalStatistics.ActiveConstraintEvaluationCount +=
            TimestepContactPairs.Length;

        for (int i = 0; i < TimestepContactPairs.Length; i++)
        {
            UnitCollisionPair pair = TimestepContactPairs[i];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];

            float denominator = bodyA.InverseMass + bodyB.InverseMass + alpha;
            if (denominator <= 0f)
                continue;

            float3 currentDelta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            currentDelta.y = 0;
            float radiusSum = bodyA.Radius + bodyB.Radius;
            float3 normal;
            float constraintValue;

            if (pair.ContactMode == UnitContactMode.Predictive)
            {
                normal = pair.PredictiveNormal;
                if (pair.PredictiveNormalOriented == 0)
                {
                    // BodyIndex ordering is frame-local. Orient the persistent
                    // normal once for this timestep, then keep it fixed even if
                    // the pair later crosses to the opposite side.
                    if (math.dot(currentDelta, normal) < 0f)
                        normal = -normal;
                    normal = math.normalizesafe(
                        normal,
                        DeterministicFallbackNormal(pair.BodyA, pair.BodyB));
                    pair.PredictiveNormal = normal;
                    pair.PredictiveNormalOriented = 1;
                }
                constraintValue = math.dot(currentDelta, normal) - radiusSum;
            }
            else
            {
                float distance = math.length(currentDelta);
                normal = distance > 0.00001f
                    ? currentDelta / distance
                    : DeterministicFallbackNormal(pair.BodyA, pair.BodyB);
                constraintValue = distance - radiusSum;
            }

            float deltaLambda = -(constraintValue + alpha * pair.Lambda) / denominator;
            float nextLambda = math.max(0f, pair.Lambda + deltaLambda);
            float appliedLambda = nextLambda - pair.Lambda;
            pair.Lambda = nextLambda;

            if (nextLambda > 0.0000001f && pair.WasActivated == 0)
            {
                pair.WasActivated = 1;
                pair.ActivatedSubstepCount++;
                if (pair.WasActivatedThisTimestep == 0)
                {
                    pair.WasActivatedThisTimestep = 1;
                    pair.FirstActivatedSubstep = substepIndex;
                    statistics.TimestepContactSetUniqueActivatedPairCount++;
                }
            }
            TimestepContactPairs[i] = pair;

            float pairCorrection =
                (bodyA.InverseMass + bodyB.InverseMass) * math.abs(appliedLambda);
            CaptureSimulationDebuggerPair(
                substepIndex,
                pair,
                bodyA,
                bodyB,
                normal,
                constraintValue,
                pairCorrection);

            if (math.abs(appliedLambda) <= 0.0000001f)
                continue;

            if (pair.WasCorrectedThisTimestep == 0)
            {
                pair.WasCorrectedThisTimestep = 1;
                incrementalStatistics.UniqueCorrectedPairCount++;
                TimestepContactPairs[i] = pair;
            }

            totalPositionCorrection += pairCorrection;
            maxPositionCorrection = math.max(maxPositionCorrection, pairCorrection);

            bodyA.PredictedPosition += normal * (bodyA.InverseMass * appliedLambda);
            bodyB.PredictedPosition -= normal * (bodyB.InverseMass * appliedLambda);
            bodyA.ContactPositionCorrection += normal * (bodyA.InverseMass * appliedLambda);
            bodyB.ContactPositionCorrection -= normal * (bodyB.InverseMass * appliedLambda);
            bodyA.TimestepContactCorrection += normal * (bodyA.InverseMass * appliedLambda);
            bodyB.TimestepContactCorrection -= normal * (bodyB.InverseMass * appliedLambda);
            bodyA.PredictedPosition.y = bodyA.CurrentPosition.y;
            bodyB.PredictedPosition.y = bodyB.CurrentPosition.y;
            States[pair.BodyA] = bodyA;
            States[pair.BodyB] = bodyB;

            if (trackCorrectedBodies)
            {
                if (bodyA.InverseMass > 0f)
                    MarkCorrectedBody(pair.BodyA);
                if (bodyB.InverseMass > 0f)
                    MarkCorrectedBody(pair.BodyB);
            }
        }
    }

    private void SolveWallConstraintIteration(
        bool trackCorrectedBodies,
        out float totalPositionCorrection,
        out float maxPositionCorrection)
    {
        if (trackCorrectedBodies)
            ResetCorrectedBodyTracking();

        totalPositionCorrection = 0f;
        maxPositionCorrection = 0f;
        if (!Grid.IsCreated)
            return;

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid || state.InverseMass <= 0f)
                continue;

            int2 currentCell = FlowFieldUtils.WorldToCell(
                state.PredictedPosition,
                GridOrigin,
                CellRadius);

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    int2 checkCell = currentCell + new int2(x, y);
                    if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                        checkCell.y < 0 || checkCell.y >= GridDimensions.y)
                        continue;

                    int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);
                    if (Grid[checkIndex].Cost != 0)
                        continue;

                    float3 wallPosition = GridOrigin + new float3(
                        checkCell.x * CellRadius * 2f + CellRadius,
                        state.PredictedPosition.y,
                        checkCell.y * CellRadius * 2f + CellRadius);
                    float3 delta = state.PredictedPosition - wallPosition;
                    delta.y = 0f;
                    float distance = math.length(delta);
                    float hardDistance = CellRadius + math.max(0f, state.Radius);
                    if (distance >= hardDistance)
                        continue;

                    float3 normal = distance > 0.00001f
                        ? delta / distance
                        : DeterministicFallbackNormal(bodyIndex, checkIndex);
                    float3 correction = normal * ((hardDistance - distance) * 0.5f);
                    state.PredictedPosition += correction;
                    state.PredictedPosition.y = state.CurrentPosition.y;
                    state.WallPositionCorrection += correction;
                    state.TimestepWallCorrection += correction;

                    float correctionLength = math.length(correction);
                    totalPositionCorrection += correctionLength;
                    maxPositionCorrection = math.max(maxPositionCorrection, correctionLength);
                    if (trackCorrectedBodies)
                        MarkCorrectedBody(bodyIndex);
                }
            }

            States[bodyIndex] = state;
        }
    }

    private void ReconstructVelocities(
        float substepDeltaTime,
        ref PredictiveDiscContactStatistics statistics)
    {
        float speedBeforeSum = 0f;
        float speedAfterSum = 0f;
        int simulatedBodyCount = 0;

        for (int i = 0; i < States.Length; i++)
        {
            FlowMovementFrameState state = States[i];
            if (!state.IsInsideGrid)
                continue;

            state.IntegratedVelocity =
                (state.PredictedPosition - state.PreviousSubstepPosition) / substepDeltaTime;
            state.IntegratedVelocity.y = 0;
            float velocityChange = math.distance(
                state.IntegratedVelocity,
                state.VelocityBeforeContact);
            statistics.TotalVelocityChange += velocityChange;
            statistics.MaxVelocityChange = math.max(
                statistics.MaxVelocityChange,
                velocityChange);
            speedBeforeSum += math.length(state.VelocityBeforeContact);
            speedAfterSum += math.length(state.IntegratedVelocity);
            simulatedBodyCount++;
            States[i] = state;
        }

        if (simulatedBodyCount > 0)
        {
            statistics.AverageSpeedBeforeContact += speedBeforeSum / simulatedBodyCount;
            statistics.AverageSpeedAfterContact += speedAfterSum / simulatedBodyCount;
        }
    }

    public static float3 DeterministicFallbackNormal(int bodyA, int bodyB)
    {
        uint hash = math.hash(new int2(bodyA, bodyB));
        return (hash & 1u) == 0u
            ? new float3(1, 0, 0)
            : new float3(0, 0, 1);
    }
}


}
