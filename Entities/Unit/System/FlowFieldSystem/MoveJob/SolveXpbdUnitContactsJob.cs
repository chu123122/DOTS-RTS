using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;

/// <summary>
/// Frostbite-inspired Predictive Disc Contact 求解器。
/// 每个 substep 保存可信起始构型、预测无约束终点、生成 swept disc Pair，
/// 随后全部 XPBD iteration 复用同一份 Pair，不在 iteration 内重复 Broad/Narrow Phase。
/// </summary>
[BurstCompile]
public struct SolveXpbdUnitContactsJob : IJob
{
    public float DeltaTime;
    public int SubstepCount;
    public int IterationCount;
    public float Compliance;
    public float PredictiveSkin;

    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    public NativeList<SweptDiscCellEntry> SweptCellEntries;
    public NativeList<UnitCollisionPair> Pairs;
    public NativeArray<FlowMovementFrameState> States;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;

    public void Execute()
    {
        long solverStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        int substepCount = math.max(1, SubstepCount);
        int iterationCount = math.max(1, IterationCount);
        float substepDeltaTime = DeltaTime / substepCount;
        var statistics = new PredictiveDiscContactStatistics();
        float penetrationSum = 0f;

        if (substepDeltaTime <= 0f)
        {
            Statistics.Value = statistics;
            return;
        }

        InitializeSolverState();

        for (int substepIndex = 0; substepIndex < substepCount; substepIndex++)
        {
            PredictUnconstrainedPositions(substepDeltaTime);

            long pairGenerationStart = ProfilerUnsafeUtility.Timestamp;
            BuildSweptContactPairs(ref statistics);
            statistics.PairGenerationNanoseconds +=
                TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - pairGenerationStart);

            long iterationStart = ProfilerUnsafeUtility.Timestamp;
            for (int iterationIndex = 0; iterationIndex < iterationCount; iterationIndex++)
                SolveContactIteration(substepDeltaTime);
            statistics.IterationNanoseconds +=
                TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - iterationStart);

            AccumulateConstraintStatistics(ref statistics, ref penetrationSum);
            ReconstructVelocities(substepDeltaTime);
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
        statistics.AverageIterationNanoseconds =
            statistics.IterationNanoseconds / math.max(1, substepCount * iterationCount);
        statistics.SolverNanoseconds =
            TimestampToNanoseconds(ProfilerUnsafeUtility.Timestamp - solverStartTimestamp);
        Statistics.Value = statistics;
    }

    private void InitializeSolverState()
    {
        for (int i = 0; i < States.Length; i++)
        {
            FlowMovementFrameState state = States[i];
            state.IntegratedVelocity = state.IsInsideGrid ? state.CurrentVelocity : float3.zero;
            state.StartPosition = state.CurrentPosition;
            state.PredictedPosition = state.CurrentPosition;
            state.PreviousSubstepPosition = state.CurrentPosition;
            state.PositionCorrection = float3.zero;
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

            float3 totalForce = state.IndependentForce + state.SoftAvoidanceForce;
            if (state.Cell.Cost == 0 && math.lengthsq(totalForce) < 0.1f)
            {
                float3 cellCenter = GridOrigin + new float3(
                    state.CellPosition.x * CellRadius * 2 + CellRadius,
                    state.CurrentPosition.y,
                    state.CellPosition.y * CellRadius * 2 + CellRadius);
                float3 escapeDirection = state.StartPosition - cellCenter;
                escapeDirection.y = 0;
                escapeDirection = math.normalizesafe(escapeDirection, new float3(1, 0, 0));
                totalForce += escapeDirection * state.MoveSpeed * 5f;
            }

            if (math.lengthsq(totalForce) > state.MaxForce * state.MaxForce)
                totalForce = math.normalizesafe(totalForce) * state.MaxForce;

            float3 velocity = state.IntegratedVelocity + totalForce * substepDeltaTime;
            if (state.IsSettled)
                velocity *= math.pow(0.8f, substepDeltaTime * 60f);

            if (math.lengthsq(velocity) > state.MoveSpeed * state.MoveSpeed)
                velocity = math.normalizesafe(velocity) * state.MoveSpeed;

            state.PredictedPosition = state.StartPosition + velocity * substepDeltaTime;
            state.PredictedPosition.y = state.CurrentPosition.y;
            state.IntegratedVelocity = velocity;
            States[i] = state;
        }
    }

    private void BuildSweptContactPairs(ref PredictiveDiscContactStatistics statistics)
    {
        SweptCellEntries.Clear();
        Pairs.Clear();
        float cellSize = math.max(CellRadius * 2f, 0.0001f);
        float skin = math.max(0f, PredictiveSkin);

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;

            float sweptExtent = math.max(0f, state.Radius) + skin;
            float2 sweptMin = math.min(state.StartPosition.xz, state.PredictedPosition.xz) - sweptExtent;
            float2 sweptMax = math.max(state.StartPosition.xz, state.PredictedPosition.xz) + sweptExtent;
            int2 minCell = (int2)math.floor((sweptMin - GridOrigin.xz) / cellSize);
            int2 maxCell = (int2)math.floor((sweptMax - GridOrigin.xz) / cellSize);

            if (maxCell.x < 0 || maxCell.y < 0 ||
                minCell.x >= GridDimensions.x || minCell.y >= GridDimensions.y)
                continue;

            minCell = math.clamp(minCell, int2.zero, GridDimensions - 1);
            maxCell = math.clamp(maxCell, int2.zero, GridDimensions - 1);

            for (int x = minCell.x; x <= maxCell.x; x++)
            {
                for (int y = minCell.y; y <= maxCell.y; y++)
                {
                    SweptCellEntries.Add(new SweptDiscCellEntry
                    {
                        CellIndex = FlowFieldUtils.GetFlatIndex(new int2(x, y), GridDimensions),
                        BodyIndex = bodyIndex
                    });
                }
            }
        }

        SweptCellEntries.AsArray().Sort(new SweptDiscCellEntryComparer());
        EmitCellPairs();
        SortAndDeduplicatePairs();
        statistics.CandidatePairCount += Pairs.Length;
        FilterAndClassifyPairs(ref statistics, skin);
    }

    private void EmitCellPairs()
    {
        int cellStart = 0;
        while (cellStart < SweptCellEntries.Length)
        {
            int cellIndex = SweptCellEntries[cellStart].CellIndex;
            int cellEnd = cellStart + 1;
            while (cellEnd < SweptCellEntries.Length &&
                   SweptCellEntries[cellEnd].CellIndex == cellIndex)
                cellEnd++;

            for (int first = cellStart; first < cellEnd; first++)
            {
                int firstBody = SweptCellEntries[first].BodyIndex;
                for (int second = first + 1; second < cellEnd; second++)
                {
                    int secondBody = SweptCellEntries[second].BodyIndex;
                    if (firstBody == secondBody)
                        continue;

                    Pairs.Add(new UnitCollisionPair
                    {
                        BodyA = math.min(firstBody, secondBody),
                        BodyB = math.max(firstBody, secondBody)
                    });
                }
            }

            cellStart = cellEnd;
        }
    }

    private void SortAndDeduplicatePairs()
    {
        if (Pairs.Length <= 1)
            return;

        Pairs.AsArray().Sort(new UnitCollisionPairComparer());
        int writeIndex = 1;
        UnitCollisionPair previous = Pairs[0];

        for (int readIndex = 1; readIndex < Pairs.Length; readIndex++)
        {
            UnitCollisionPair current = Pairs[readIndex];
            if (current.BodyA == previous.BodyA && current.BodyB == previous.BodyB)
                continue;

            Pairs[writeIndex++] = current;
            previous = current;
        }

        Pairs.ResizeUninitialized(writeIndex);
    }

    private void FilterAndClassifyPairs(
        ref PredictiveDiscContactStatistics statistics,
        float skin)
    {
        int writeIndex = 0;

        for (int readIndex = 0; readIndex < Pairs.Length; readIndex++)
        {
            UnitCollisionPair pair = Pairs[readIndex];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            float radiusSum = bodyA.Radius + bodyB.Radius;

            float3 r0 = bodyB.StartPosition - bodyA.StartPosition;
            float3 relativeDisplacement =
                (bodyB.PredictedPosition - bodyB.StartPosition) -
                (bodyA.PredictedPosition - bodyA.StartPosition);
            r0.y = 0;
            relativeDisplacement.y = 0;

            float relativeLengthSq = math.lengthsq(relativeDisplacement);
            float closestTime = relativeLengthSq > 0.0000001f
                ? math.clamp(-math.dot(r0, relativeDisplacement) / relativeLengthSq, 0f, 1f)
                : 0f;
            float minDistanceSq = math.lengthsq(r0 + closestTime * relativeDisplacement);
            float candidateDistance = radiusSum + skin;
            if (minDistanceSq > candidateDistance * candidateDistance)
                continue;

            float startDistanceSq = math.lengthsq(r0);
            float3 endDelta = bodyB.PredictedPosition - bodyA.PredictedPosition;
            endDelta.y = 0;
            float endDistanceSq = math.lengthsq(endDelta);
            float radiusSumSq = radiusSum * radiusSum;

            // 只有“起终点均分离、但线性 swept path 实际穿过接触半径”的 Pair
            // 使用初始分离平面。普通重叠和仅进入 skin 的 Pair 保持径向约束。
            bool shouldPreventSideExchange =
                startDistanceSq >= radiusSumSq &&
                endDistanceSq >= radiusSumSq &&
                minDistanceSq <= radiusSumSq;

            pair.Lambda = 0f;
            pair.WasActivated = 0;
            pair.ContactMode = shouldPreventSideExchange
                ? UnitContactMode.Predictive
                : UnitContactMode.Regular;
            Pairs[writeIndex++] = pair;

            if (pair.ContactMode == UnitContactMode.Predictive)
                statistics.PredictivePairCount++;
        }

        Pairs.ResizeUninitialized(writeIndex);
        statistics.ContactPairCount += writeIndex;
    }

    private void SolveContactIteration(float substepDeltaTime)
    {
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

            bodyA.PredictedPosition += normal * (bodyA.InverseMass * appliedLambda);
            bodyB.PredictedPosition -= normal * (bodyB.InverseMass * appliedLambda);
            bodyA.PredictedPosition.y = bodyA.CurrentPosition.y;
            bodyB.PredictedPosition.y = bodyB.CurrentPosition.y;
            States[pair.BodyA] = bodyA;
            States[pair.BodyB] = bodyB;
        }
    }

    private void AccumulateConstraintStatistics(
        ref PredictiveDiscContactStatistics statistics,
        ref float penetrationSum)
    {
        for (int i = 0; i < Pairs.Length; i++)
        {
            UnitCollisionPair pair = Pairs[i];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];

            if (pair.WasActivated != 0)
            {
                statistics.ActiveConstraintCount++;
                if (pair.ContactMode == UnitContactMode.Predictive)
                    statistics.PredictiveActivatedCount++;
            }

            float3 delta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            delta.y = 0;
            float penetration = math.max(0f, bodyA.Radius + bodyB.Radius - math.length(delta));
            if (penetration <= 0f)
                continue;

            statistics.PenetratingPairCount++;
            statistics.MaxPenetration = math.max(statistics.MaxPenetration, penetration);
            penetrationSum += penetration;
        }
    }

    private void ReconstructVelocities(float substepDeltaTime)
    {
        for (int i = 0; i < States.Length; i++)
        {
            FlowMovementFrameState state = States[i];
            if (!state.IsInsideGrid)
                continue;

            state.IntegratedVelocity =
                (state.PredictedPosition - state.PreviousSubstepPosition) / substepDeltaTime;
            state.IntegratedVelocity.y = 0;
            States[i] = state;
        }
    }

    public static float3 DeterministicFallbackNormal(int bodyA, int bodyB)
    {
        uint hash = math.hash(new int2(bodyA, bodyB));
        return (hash & 1u) == 0u
            ? new float3(1, 0, 0)
            : new float3(0, 0, 1);
    }

    private static long TimestampToNanoseconds(long timestampDelta)
    {
        var ratio = ProfilerUnsafeUtility.TimestampToNanosecondsConversionRatio;
        return timestampDelta * ratio.Numerator / ratio.Denominator;
    }
}

[BurstCompile]
public partial struct PublishPredictiveDiscContactStatisticsJob : IJobEntity
{
    [ReadOnly] public NativeReference<PredictiveDiscContactStatistics> Source;

    public void Execute(ref PredictiveDiscContactStatistics destination)
    {
        destination = Source.Value;
    }
}
