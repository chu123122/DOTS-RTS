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
    private void CalculateSoftAvoidanceForSubstep(
        bool useFatAabbCandidates,
        float substepDeltaTime,
        ref PredictiveDiscContactStatistics statistics)
    {
        SweptCellEntries.Clear();
        Pairs.Clear();

        float softShell = math.max(0f, SoftAvoidanceShell);
        float cellSize = math.max(CellRadius * 2f, 0.0001f);

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            state.SoftAvoidanceVelocity = float3.zero;
            state.WallAvoidanceVelocity = float3.zero;
            state.SoftAvoidanceNeighborCount = 0;
            if (!state.IsInsideGrid)
            {
                States[bodyIndex] = state;
                continue;
            }

            float3 position = state.PredictedPosition;
            int2 currentCell = FlowFieldUtils.WorldToCell(position, GridOrigin, CellRadius);
            AccumulateWallAvoidanceVelocity(
                position,
                currentCell,
                state.MoveSpeed,
                state.Radius,
                softShell,
                ref state.WallAvoidanceVelocity);
            States[bodyIndex] = state;

            if (softShell <= 0f || SoftAvoidanceResponseRate <= 0f ||
                useFatAabbCandidates)
                continue;

            float softExtent = math.max(0f, state.Radius) + softShell * 0.5f;
            float2 softPathEnd = position.xz;
            if (SoftAvoidanceVelocitySolver ==
                SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle)
            {
                softPathEnd += state.BasePredictedVelocity.xz *
                               math.max(0f, RvoTimeHorizon);
            }
            float2 softMin = math.min(position.xz, softPathEnd) - softExtent;
            float2 softMax = math.max(position.xz, softPathEnd) + softExtent;
            int2 minCell = (int2)math.floor((softMin - GridOrigin.xz) / cellSize);
            int2 maxCell = (int2)math.floor((softMax - GridOrigin.xz) / cellSize);
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

        if (softShell > 0f && SoftAvoidanceResponseRate > 0f &&
            useFatAabbCandidates)
        {
            statistics.SoftAvoidanceFatAabbUseCount++;
            statistics.SoftAvoidanceCandidatePairCount += MappedFatCachePairs.Length;
            statistics.SoftAvoidanceActivatedPairCount +=
                AccumulateUnitAvoidanceVelocities(
                    MappedFatCachePairs.AsArray(),
                    softShell,
                    substepDeltaTime);
        }
        else if (softShell > 0f && SoftAvoidanceResponseRate > 0f)
        {
            SweptCellEntries.AsArray().Sort(new SweptDiscCellEntryComparer());
            EmitCellPairs();
            SortAndDeduplicatePairs();
            statistics.SoftAvoidanceCandidatePairCount += Pairs.Length;
            statistics.SoftAvoidanceActivatedPairCount +=
                AccumulateUnitAvoidanceVelocities(
                    Pairs.AsArray(),
                    softShell,
                    substepDeltaTime);
        }

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;

            if (state.SoftAvoidanceNeighborCount > 0 &&
                SoftAvoidanceVelocitySolver ==
                SoftAvoidanceVelocitySolverMode.SurfaceVelocityBuffer)
                state.SoftAvoidanceVelocity /= state.SoftAvoidanceNeighborCount;

            state.SoftAvoidanceVelocity += state.WallAvoidanceVelocity;
            float maxAvoidanceSpeed = math.max(0f, state.MoveSpeed);
            if (math.lengthsq(state.SoftAvoidanceVelocity) >
                maxAvoidanceSpeed * maxAvoidanceSpeed)
            {
                state.SoftAvoidanceVelocity =
                    math.normalizesafe(state.SoftAvoidanceVelocity) * maxAvoidanceSpeed;
            }

            States[bodyIndex] = state;
        }
    }

    private int AccumulateUnitAvoidanceVelocities(
        NativeArray<UnitCollisionPair> candidates,
        float softShell,
        float substepDeltaTime)
    {
        int activatedPairCount = 0;
        for (int pairIndex = 0; pairIndex < candidates.Length; pairIndex++)
        {
            UnitCollisionPair pair = candidates[pairIndex];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            bool activated = SoftAvoidanceMath.TryCalculatePairVelocities(
                SoftAvoidanceVelocitySolver,
                bodyA.PredictedPosition,
                bodyB.PredictedPosition,
                bodyA.BasePredictedVelocity,
                bodyB.BasePredictedVelocity,
                bodyA.Radius,
                bodyB.Radius,
                bodyA.InverseMass,
                bodyB.InverseMass,
                bodyA.MoveSpeed,
                bodyB.MoveSpeed,
                softShell,
                RvoTimeHorizon,
                substepDeltaTime,
                DeterministicFallbackNormal(pair.BodyA, pair.BodyB),
                out float3 velocityA,
                out float3 velocityB);
            if (!activated)
                continue;
            bodyA.SoftAvoidanceVelocity += velocityA;
            bodyB.SoftAvoidanceVelocity += velocityB;
            bodyA.SoftAvoidanceNeighborCount++;
            bodyB.SoftAvoidanceNeighborCount++;
            activatedPairCount++;
            States[pair.BodyA] = bodyA;
            States[pair.BodyB] = bodyB;
        }

        return activatedPairCount;
    }

    private void AccumulateWallAvoidanceVelocity(
        float3 position,
        int2 currentCell,
        float moveSpeed,
        float bodyRadius,
        float softShell,
        ref float3 avoidanceVelocity)
    {
        if (currentCell.x < 0 || currentCell.x >= GridDimensions.x ||
            currentCell.y < 0 || currentCell.y >= GridDimensions.y)
            return;

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
                    position.y,
                    checkCell.y * CellRadius * 2f + CellRadius);
                float wallCheckRadius = CellRadius + math.max(0f, bodyRadius) + softShell;
                avoidanceVelocity += SoftAvoidanceMath.CalculateWallVelocity(
                    position,
                    wallPosition,
                    moveSpeed,
                    wallCheckRadius);
            }
        }
    }
}
}
