using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct SolveXpbdUnitContactsJob
{
    private void CalculateSoftAvoidanceForSubstep(
        float substepDeltaTime,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        float softShell = math.max(0f, SoftAvoidanceShell);

        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CrowdBodyStepState step = StepStates[bodyIndex];
            step.SoftAvoidanceVelocity = float3.zero;
            step.WallAvoidanceVelocity = float3.zero;
            step.SoftAvoidanceNeighborCount = 0;
            CrowdBodySnapshot body = Bodies[bodyIndex];
            if (body.IsInsideSimulationDomain != 0)
            {
                int2 currentCell = EnvironmentGeometry.WorldToCell(step.SolvedPosition);
                AccumulateWallAvoidanceVelocity(
                    step.SolvedPosition,
                    currentCell,
                    body.MoveSpeed,
                    body.Radius,
                    softShell,
                    ref step.WallAvoidanceVelocity);
            }
            StepStates[bodyIndex] = step;
        }

        if (softShell > 0f && SoftAvoidanceResponseRate > 0f)
        {
            if (EnablePersistentContactCache)
                statistics.SoftAvoidanceFatAabbUseCount++;
            statistics.SoftAvoidanceCandidatePairCount += SoftAvoidancePairs.Length;
            incrementalStatistics.SoftAvoidancePairEvaluationCount +=
                SoftAvoidancePairs.Length;
            statistics.SoftAvoidanceActivatedPairCount +=
                AccumulateUnitAvoidanceVelocities(
                    SoftAvoidancePairs.AsArray(),
                    softShell,
                    substepDeltaTime);
        }

        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CrowdBodySnapshot body = Bodies[bodyIndex];
            if (body.IsInsideSimulationDomain == 0)
                continue;
            CrowdBodyStepState step = StepStates[bodyIndex];
            if (step.SoftAvoidanceNeighborCount > 0 &&
                SoftAvoidanceVelocitySolver ==
                SoftAvoidanceVelocitySolverMode.SurfaceVelocityBuffer)
            {
                step.SoftAvoidanceVelocity /= step.SoftAvoidanceNeighborCount;
            }

            step.SoftAvoidanceVelocity += step.WallAvoidanceVelocity;
            float maxAvoidanceSpeed = math.max(0f, body.MoveSpeed);
            if (math.lengthsq(step.SoftAvoidanceVelocity) >
                maxAvoidanceSpeed * maxAvoidanceSpeed)
            {
                step.SoftAvoidanceVelocity =
                    math.normalizesafe(step.SoftAvoidanceVelocity) *
                    maxAvoidanceSpeed;
            }
            StepStates[bodyIndex] = step;
        }
    }

    private void BuildSoftAvoidancePairViewFromInteractions(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        SoftAvoidancePairs.Clear();
        if (SoftAvoidanceShell <= 0f || SoftAvoidanceResponseRate <= 0f)
        {
            incrementalStatistics.CurrentSoftAvoidancePairCount = 0;
            return;
        }

        for (int pairIndex = 0;
             pairIndex < TimestepInteractionPairs.Length;
             pairIndex++)
        {
            BodyPair pair = TimestepInteractionPairs[pairIndex];
            if (!CouldEnterSoftAvoidanceRange(pair.BodyA, pair.BodyB))
                continue;
            SoftAvoidancePairs.Add(new BodyPair
            {
                BodyA = pair.BodyA,
                BodyB = pair.BodyB
            });
        }
        incrementalStatistics.CurrentSoftAvoidancePairCount =
            SoftAvoidancePairs.Length;
        ValidateSoftAvoidancePairViewAgainstQuadraticOracle(
            ref incrementalStatistics);
    }

    private bool CouldEnterSoftAvoidanceRange(int bodyAIndex, int bodyBIndex)
    {
        if (SoftAvoidanceShell <= 0f || SoftAvoidanceResponseRate <= 0f)
            return false;

        CrowdBodySnapshot bodyA = Bodies[bodyAIndex];
        CrowdBodySnapshot bodyB = Bodies[bodyBIndex];
        CrowdMotionEvidence evidenceA = MotionEvidence[bodyAIndex];
        CrowdMotionEvidence evidenceB = MotionEvidence[bodyBIndex];
        CrowdBodyStepState stepA = StepStates[bodyAIndex];
        CrowdBodyStepState stepB = StepStates[bodyBIndex];

        float maxDistance = bodyA.Radius + bodyB.Radius +
                            math.max(0f, SoftAvoidanceShell);
        float3 relativeStart = evidenceB.TrajectoryStart - evidenceA.TrajectoryStart;
        float3 relativeTimestepDisplacement =
            (evidenceB.BaselineEnd - evidenceB.TrajectoryStart) -
            (evidenceA.BaselineEnd - evidenceA.TrajectoryStart);
        relativeStart.y = 0f;
        relativeTimestepDisplacement.y = 0f;
        if (CouldRelativePathApproach(
                relativeStart,
                relativeTimestepDisplacement,
                maxDistance))
            return true;

        if (SoftAvoidanceVelocitySolver !=
            SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle)
            return false;

        float3 relativeHorizonDisplacement =
            (stepB.BaseVelocity - stepA.BaseVelocity) *
            math.max(0f, RvoTimeHorizon);
        relativeHorizonDisplacement.y = 0f;
        return CouldRelativePathApproach(
            relativeStart,
            relativeHorizonDisplacement,
            maxDistance);
    }

    private static bool CouldRelativePathApproach(
        float3 relativeStart,
        float3 relativeDisplacement,
        float maxDistance)
    {
        float relativeLengthSq = math.lengthsq(relativeDisplacement);
        float closestTime = relativeLengthSq > 0.0000001f
            ? math.clamp(
                -math.dot(relativeStart, relativeDisplacement) /
                relativeLengthSq,
                0f,
                1f)
            : 0f;
        float minDistanceSq = math.lengthsq(
            relativeStart + closestTime * relativeDisplacement);
        return minDistanceSq <= maxDistance * maxDistance;
    }

    private void ValidateSoftAvoidancePairViewAgainstQuadraticOracle(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        if (!EnableDiagnostics || SoftAvoidanceShell <= 0f ||
            SoftAvoidanceResponseRate <= 0f)
            return;

        int oracleCount = 0;
        int missingCount = 0;
        for (int bodyA = 0; bodyA < Bodies.Length; bodyA++)
        {
            if (Bodies[bodyA].IsInsideSimulationDomain == 0)
                continue;
            for (int bodyB = bodyA + 1; bodyB < Bodies.Length; bodyB++)
            {
                if (Bodies[bodyB].IsInsideSimulationDomain == 0 ||
                    !CouldEnterSoftAvoidanceRange(bodyA, bodyB))
                    continue;
                oracleCount++;
                if (FindPairIndex(SoftAvoidancePairs, bodyA, bodyB) < 0)
                    missingCount++;
            }
        }
        incrementalStatistics.SoftAvoidanceOraclePairCount += oracleCount;
        incrementalStatistics.SoftAvoidanceOracleMissingPairCount += missingCount;
        if (missingCount > 0)
            incrementalStatistics.OracleMismatch = 1;
    }

    private int AccumulateUnitAvoidanceVelocities(
        NativeArray<BodyPair> candidates,
        float softShell,
        float substepDeltaTime)
    {
        int activatedPairCount = 0;
        for (int pairIndex = 0; pairIndex < candidates.Length; pairIndex++)
        {
            BodyPair pair = candidates[pairIndex];
            CrowdBodySnapshot bodyA = Bodies[pair.BodyA];
            CrowdBodySnapshot bodyB = Bodies[pair.BodyB];
            CrowdBodyStepState stepA = StepStates[pair.BodyA];
            CrowdBodyStepState stepB = StepStates[pair.BodyB];

            float3 softDelta = stepA.SolvedPosition - stepB.SolvedPosition;
            softDelta.y = 0f;
            float softDistSq = math.lengthsq(softDelta);
            float softMaxDist = bodyA.Radius + bodyB.Radius + softShell;
            if (softDistSq > softMaxDist * softMaxDist)
                continue;

            bool activated = SoftAvoidanceMath.TryCalculatePairVelocities(
                SoftAvoidanceVelocitySolver,
                stepA.SolvedPosition,
                stepB.SolvedPosition,
                stepA.BaseVelocity,
                stepB.BaseVelocity,
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
            stepA.SoftAvoidanceVelocity += velocityA;
            stepB.SoftAvoidanceVelocity += velocityB;
            stepA.SoftAvoidanceNeighborCount++;
            stepB.SoftAvoidanceNeighborCount++;
            activatedPairCount++;
            StepStates[pair.BodyA] = stepA;
            StepStates[pair.BodyB] = stepB;
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
        if (!EnvironmentGeometry.Contains(currentCell))
            return;

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                int2 checkCell = currentCell + new int2(x, y);
                if (!IsObstacleCell(checkCell))
                    continue;

                float3 wallPosition = ObstacleCellCenter(checkCell, position.y);
                float wallCheckRadius = CellRadius +
                                        math.max(0f, bodyRadius) +
                                        softShell;
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
