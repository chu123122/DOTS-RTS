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
/// 每个 substep 保存可信起始构型、预测无约束终点、生成 swept disc Pair，
/// 随后全部 XPBD iteration 复用同一份 Pair，不在 iteration 内重复 Broad/Narrow Phase。
/// 缓存、软避让、Broad Phase 与诊断实现分布在同一 partial Job 的独立文件中。
/// </summary>
[BurstCompile]
public partial struct SolveXpbdUnitContactsJob : IJob
{
    public float DeltaTime;
    public int SubstepCount;
    public int IterationCount;
    public float Compliance;
    public float PredictiveSkin;
    public float SoftAvoidanceResponseRate;
    public float SoftAvoidanceShell;
    public float SettledSoftAvoidanceMultiplier;
    public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
    public float RvoTimeHorizon;
    public bool EnablePredictivePairGeneration;
    public bool EnablePredictiveContacts;
    public bool EnableDiagnostics;
    public bool EnableFatAabbCache;
    public float FatAabbCacheMargin;
    public AdaptiveFatAabbSettings AdaptiveSettings;
    public int2 AdaptiveCellDimensions;
    public Entity DiagnosticSelectedEntity;

    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    public NativeList<UnitCollisionPair> Pairs;
    public NativeList<SweptDiscCellEntry> ShadowCellEntries;
    public NativeList<UnitCollisionPair> ShadowBodyPairs;
    public NativeList<ShadowFatBodyProxy> ShadowCurrentProxies;
    public NativeList<ShadowEntityPair> ShadowCurrentPairs;
    public NativeParallelHashMap<Entity, int> CurrentBodyIndexByEntity;
    public NativeList<UnitCollisionPair> MappedFatCachePairs;
    public NativeArray<byte> CorrectedBodyFlags;
    public NativeList<int> CorrectedBodyIndices;
    public NativeList<ShadowFatBodyProxy> ShadowPreviousProxies;
    public NativeList<ShadowEntityPair> ShadowPreviousPairs;
    public NativeReference<FatAabbCacheState> FatAabbCacheState;
    public NativeArray<AdaptiveFatAabbCellHistory> AdaptiveCellHistory;
    public NativeArray<AdaptiveFatAabbCellMetric> AdaptiveCellMetrics;
    public NativeArray<AdaptiveFatAabbBodyRouting> AdaptiveBodyRouting;
    public NativeList<int> AdaptiveFloodQueue;
    public NativeList<int> AdaptiveFloodCells;
    public NativeList<AdaptiveFatAabbRegion> AdaptiveRegions;
    public NativeList<AdaptiveFatAabbDebugCell> AdaptiveDebugCells;
    public NativeList<AdaptiveFatAabbDebugRegion> AdaptiveDebugRegions;
    public NativeList<AdaptiveFatAabbDebugProxy> AdaptiveDebugProxies;
    public NativeList<AdaptiveFatAabbRegionHistory> AdaptiveRegionHistory;
    public NativeList<AdaptiveFatAabbRegionHistory> AdaptiveRegionHistoryScratch;
    public NativeReference<int> AdaptiveNextRegionId;
    public NativeReference<AdaptiveFatAabbCacheFeedback> AdaptiveCacheFeedback;
    public NativeArray<FlowMovementFrameState> States;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeReference<ShadowNeighborCacheStatistics> ShadowStatistics;
    public NativeList<Stage3ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<Stage3ContactPairDiagnostic> PairDiagnostics;
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBodyDiagnostic;

    public void Execute()
    {
        long solverStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        int substepCount = math.max(1, SubstepCount);
        int iterationCount = math.max(1, IterationCount);
        float substepDeltaTime = DeltaTime / substepCount;
        var statistics = new PredictiveDiscContactStatistics();
        var shadowStatistics = new ShadowNeighborCacheStatistics();
        float penetrationSum = 0f;
        bool fatCachePairsMappedThisFrame = false;
        IterationDiagnostics.Clear();
        PairDiagnostics.Clear();
        SelectedBodyDiagnostic.Value = default;

        if (substepDeltaTime <= 0f)
        {
            Statistics.Value = statistics;
            ShadowStatistics.Value = shadowStatistics;
            return;
        }

        if (!EnableFatAabbCache)
        {
            ShadowPreviousProxies.Clear();
            ShadowPreviousPairs.Clear();
            ShadowCurrentProxies.Clear();
            ShadowCurrentPairs.Clear();
            FatAabbCacheState.Value = default;
        }
        else
        {
            shadowStatistics.CacheEnabled = 1;
            PrepareCurrentBodyLookup();
            FatAabbCacheState cacheState = FatAabbCacheState.Value;
            shadowStatistics.PreviousFrameCacheAvailable =
                (byte)(cacheState.IsValid != 0 ? 1 : 0);
            shadowStatistics.CacheValidAtFrameStart = cacheState.IsValid;
            shadowStatistics.PreviousFrameCacheBodyCount = ShadowPreviousProxies.Length;
            shadowStatistics.PreviousFrameCachePairCount = ShadowPreviousPairs.Length;
        }

        InitializeSolverState();
        BuildAdaptiveFatAabbHotspots();
        if (AdaptiveFatAabbRequested)
            ResetAdaptiveFatAabbCacheWhenInactive();

        for (int substepIndex = 0; substepIndex < substepCount; substepIndex++)
        {
            long softAvoidanceStart = ProfilerUnsafeUtility.Timestamp;
            PrepareBaseVelocitiesForSubstep(substepDeltaTime);
            bool useFatCandidatesForSoftAvoidance = EnableFatAabbCache &&
                                                    !AdaptiveFatAabbRequested &&
                                                    SoftAvoidanceShell > 0f &&
                                                    SoftAvoidanceResponseRate > 0f &&
                                                    EnsureFatAabbRawCandidates(
                                                        ref shadowStatistics,
                                                        ref fatCachePairsMappedThisFrame,
                                                        false);
            CalculateSoftAvoidanceForSubstep(
                useFatCandidatesForSoftAvoidance,
                substepDeltaTime,
                ref statistics);
            statistics.SoftAvoidanceEvaluationCount++;
            statistics.SoftAvoidanceNanoseconds +=
                TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - softAvoidanceStart);

            PredictUnconstrainedPositions(substepDeltaTime);

            long pairGenerationStart = ProfilerUnsafeUtility.Timestamp;
            bool usingFatAabbCache = false;
            if (HasActiveAdaptiveFatRegions)
            {
                usingFatAabbCache = BuildAdaptiveHybridContactPairs(
                    ref statistics,
                    ref shadowStatistics,
                    ref fatCachePairsMappedThisFrame);
            }
            else if (EnableFatAabbCache && !AdaptiveFatAabbRequested)
            {
                usingFatAabbCache = BuildContactPairsFromFatAabbCache(
                    ref statistics,
                    ref shadowStatistics,
                    ref fatCachePairsMappedThisFrame);
            }
            else
            {
                BuildSweptContactPairs(ref statistics);
            }
            statistics.PairGenerationNanoseconds +=
                TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - pairGenerationStart);

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
                    usingFatAabbCache,
                    out float totalWallPositionCorrection,
                    out float maxWallPositionCorrection);

                if (usingFatAabbCache &&
                    !AreCorrectedDiscsInsideFatCache(ref shadowStatistics))
                {
                    InvalidateFatAabbCache(ref shadowStatistics, true);
                    BuildSweptContactPairs(ref statistics);
                    shadowStatistics.FullBroadPhaseFallbackCount++;
                    usingFatAabbCache = false;
                }

                SolveContactIteration(
                    substepDeltaTime,
                    usingFatAabbCache,
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

                if (usingFatAabbCache &&
                    !AreCorrectedDiscsInsideFatCache(ref shadowStatistics))
                {
                    InvalidateFatAabbCache(ref shadowStatistics, true);
                    BuildSweptContactPairs(ref statistics);
                    shadowStatistics.FullBroadPhaseFallbackCount++;
                    usingFatAabbCache = false;

                    // 最后一轮之后没有正常恢复机会；补一轮只处理新发现的单位接触。
                    if (iterationIndex == iterationCount - 1)
                    {
                        SolveContactIteration(
                            substepDeltaTime,
                            false,
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

            if (EnableDiagnostics)
                CaptureSelectedBodyAndPairs(substepIndex);
        }

        if (EnableFatAabbCache)
        {
            FatAabbCacheState cacheState = FatAabbCacheState.Value;
            if (cacheState.IsValid != 0 && shadowStatistics.CacheRebuildCount == 0)
                cacheState.AgeFrames++;
            shadowStatistics.CacheValidAtFrameEnd = cacheState.IsValid;
            shadowStatistics.CacheAgeFrames = cacheState.AgeFrames;
            shadowStatistics.CurrentFrameCacheBodyCount = ShadowPreviousProxies.Length;
            shadowStatistics.CurrentFrameCachePairCount = ShadowPreviousPairs.Length;
            shadowStatistics.CachedCandidatePairCount = ShadowPreviousPairs.Length;
            FatAabbCacheState.Value = cacheState;
        }

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
        UpdateAdaptiveFatAabbHistoryAfterSolve(
            ref shadowStatistics,
            ref statistics);
        statistics.AverageIterationNanoseconds =
            statistics.IterationNanoseconds / math.max(1, substepCount * iterationCount);
        statistics.AverageSoftAvoidanceNanoseconds =
            statistics.SoftAvoidanceNanoseconds / substepCount;
        statistics.AverageSpeedBeforeContact /= substepCount;
        statistics.AverageSpeedAfterContact /= substepCount;
        statistics.SolverNanoseconds =
            TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - solverStartTimestamp);
        Statistics.Value = statistics;
        ShadowStatistics.Value = shadowStatistics;
    }

    private void InitializeSolverState()
    {
        for (int i = 0; i < States.Length; i++)
        {
            FlowMovementFrameState state = States[i];
            state.IntegratedVelocity = state.IsInsideGrid ? state.CurrentVelocity : float3.zero;
            state.StartPosition = state.CurrentPosition;
            state.UnconstrainedPredictedPosition = state.CurrentPosition;
            state.PredictedPosition = state.CurrentPosition;
            state.PreviousSubstepPosition = state.CurrentPosition;
            state.ContactPositionCorrection = float3.zero;
            state.WallPositionCorrection = float3.zero;
            state.SoftAvoidanceVelocity = float3.zero;
            state.WallAvoidanceVelocity = float3.zero;
            state.SoftAvoidanceNeighborCount = 0;
            States[i] = state;
        }
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
        bool trackCorrectedBodies,
        out float totalPositionCorrection,
        out float maxPositionCorrection)
    {
        if (trackCorrectedBodies)
            ResetCorrectedBodyTracking();

        totalPositionCorrection = 0f;
        maxPositionCorrection = 0f;
        float alpha = Compliance / (substepDeltaTime * substepDeltaTime);

        for (int i = 0; i < Pairs.Length; i++)
        {
            UnitCollisionPair pair = Pairs[i];
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
                float3 initialDelta = bodyA.StartPosition - bodyB.StartPosition;
                initialDelta.y = 0;
                normal = math.normalizesafe(
                    initialDelta,
                    DeterministicFallbackNormal(pair.BodyA, pair.BodyB));
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

            if (nextLambda > 0.0000001f)
                pair.WasActivated = 1;
            Pairs[i] = pair;

            if (math.abs(appliedLambda) <= 0.0000001f)
                continue;

            float pairCorrection =
                (bodyA.InverseMass + bodyB.InverseMass) * math.abs(appliedLambda);
            totalPositionCorrection += pairCorrection;
            maxPositionCorrection = math.max(maxPositionCorrection, pairCorrection);

            bodyA.PredictedPosition += normal * (bodyA.InverseMass * appliedLambda);
            bodyB.PredictedPosition -= normal * (bodyB.InverseMass * appliedLambda);
            bodyA.ContactPositionCorrection += normal * (bodyA.InverseMass * appliedLambda);
            bodyB.ContactPositionCorrection -= normal * (bodyB.InverseMass * appliedLambda);
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
