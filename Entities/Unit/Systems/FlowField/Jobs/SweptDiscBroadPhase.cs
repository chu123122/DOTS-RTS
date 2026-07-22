using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct SolveXpbdUnitContactsJob
{
    private void BuildSweptInteractionPairs(ref PredictiveDiscContactStatistics statistics)
    {
        SweptCellEntries.Clear();
        Pairs.Clear();
        TimestepInteractionPairs.Clear();
        if (EnableDiagnostics)
            PairDiagnostics.Clear();
        float cellSize = math.max(CellRadius * 2f, 0.0001f);

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;

            // A0 与 A1 必须生产同一种中层 InteractionSet：包络同时覆盖
            // XPBD swept contact、Soft Avoidance shell 与 RVO horizon。
            float2 sweptMin = state.TimestepInteractionEnvelopeMin;
            float2 sweptMax = state.TimestepInteractionEnvelopeMax;
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
        TimestepInteractionPairs.AddRange(Pairs.AsArray());
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

            float3 r0 = bodyB.TimestepStartPosition - bodyA.TimestepStartPosition;
            float3 relativeDisplacement =
                (bodyB.TimestepPredictedPosition - bodyB.TimestepStartPosition) -
                (bodyA.TimestepPredictedPosition - bodyA.TimestepStartPosition);
            r0.y = 0;
            relativeDisplacement.y = 0;

            float relativeLengthSq = math.lengthsq(relativeDisplacement);
            float closestTime = relativeLengthSq > 0.0000001f
                ? math.clamp(-math.dot(r0, relativeDisplacement) / relativeLengthSq, 0f, 1f)
                : 0f;
            float minDistanceSq = math.lengthsq(r0 + closestTime * relativeDisplacement);
            float candidateDistance = radiusSum + skin;
            float retainedDistance = candidateDistance +
                                     math.max(0f, TimestepContactMargin) * 2f;
            if (minDistanceSq > retainedDistance * retainedDistance)
            {
                if (EnableDiagnostics)
                {
                    AddSelectedPairDiagnostic(
                        pair,
                        Stage3ContactDiagnosticPairKind.BroadPhaseRejected,
                        closestTime,
                        math.sqrt(minDistanceSq),
                        radiusSum,
                        0);
                }
                continue;
            }

            float startDistanceSq = math.lengthsq(r0);
            float3 endDelta = bodyB.TimestepPredictedPosition - bodyA.TimestepPredictedPosition;
            endDelta.y = 0;
            float endDistanceSq = math.lengthsq(endDelta);
            float radiusSumSq = radiusSum * radiusSum;

            bool isActualGeneratedPair = startDistanceSq <= radiusSumSq;
            if (!isActualGeneratedPair && !EnablePredictivePairGeneration)
                continue;

            bool isDormant = minDistanceSq > candidateDistance * candidateDistance;

            if (isActualGeneratedPair)
                statistics.ActualGeneratedPairCount++;
            else if (!isDormant)
                statistics.PredictiveGeneratedPairCount++;

            // 生成来源只用于统计。求解模式仍保持原有边界：只有起终点均分离、
            // 但 swept path 穿过接触半径的 Pair，才使用初始分离平面防止换侧。
            bool shouldPreventSideExchange =
                !isActualGeneratedPair &&
                !isDormant &&
                endDistanceSq >= radiusSumSq &&
                minDistanceSq <= radiusSumSq;

            if (shouldPreventSideExchange)
                statistics.PotentialPredictivePairCount++;

            pair.Lambda = 0f;
            pair.WasActivated = 0;
            pair.WasActivatedThisTimestep = 0;
            pair.WasCorrectedThisTimestep = 0;
            pair.IsDormant = (byte)(isDormant ? 1 : 0);
            pair.WasAddedByFallback = 0;
            pair.FirstActivatedSubstep = -1;
            pair.ActivatedSubstepCount = 0;
            pair.ContactMode = shouldPreventSideExchange && EnablePredictiveContacts
                ? UnitContactMode.Predictive
                : UnitContactMode.Regular;
            float3 predictiveNormal = bodyA.TimestepStartPosition - bodyB.TimestepStartPosition;
            predictiveNormal.y = 0f;
            pair.PredictiveNormal = math.normalizesafe(
                predictiveNormal,
                DeterministicFallbackNormal(pair.BodyA, pair.BodyB));
            Pairs[writeIndex++] = pair;

            if (isDormant)
                statistics.TimestepContactSetDormantPairCount++;

            if (pair.ContactMode == UnitContactMode.Predictive)
                statistics.PredictivePairCount++;
        }

        Pairs.ResizeUninitialized(writeIndex);
        statistics.ContactPairCount += writeIndex;
    }
}
}
