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
        float substepDeltaTime,
        ref PredictiveDiscContactStatistics statistics)
    {
        float softShell = math.max(0f, SoftAvoidanceShell);

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
        }

        if (softShell > 0f && SoftAvoidanceResponseRate > 0f)
        {
            if (EnablePersistentContactCache)
                statistics.SoftAvoidanceFatAabbUseCount++;
            statistics.SoftAvoidanceCandidatePairCount +=
                TimestepInteractionPairs.Length;
            statistics.SoftAvoidanceActivatedPairCount +=
                AccumulateUnitAvoidanceVelocities(
                    TimestepInteractionPairs.AsArray(),
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

            // 快速距离预检：跳过明显超出软避让范围的候选对。
            // Fat AABB 缓存可能因大 margin 产生大量候选对，但软避让
            // 只需处理 softShell 范围内的对。不预检则每个对都进
            // TryCalculatePairVelocities 做完整计算后才被 reject。
            float3 softDelta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            softDelta.y = 0;
            float softDistSq = math.lengthsq(softDelta);
            float softMaxDist = bodyA.Radius + bodyB.Radius + softShell;
            if (softDistSq > softMaxDist * softMaxDist)
                continue;

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
