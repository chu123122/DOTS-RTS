using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal static class SoftAvoidanceOracleKernel
{







    internal static bool CouldRelativePathApproach(
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




    internal static void ValidateSoftAvoidancePairViewAgainstQuadraticOracle(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        ContactPipelineConfiguration configuration,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdMotionEvidence> motionEvidence,
        NativeArray<CrowdSolverBodyState> stepStates,
        NativeList<BodyPair> softAvoidancePairs)
    {
        if (!configuration.EnableDiagnostics ||
            configuration.SoftAvoidanceShell <= 0f ||
            configuration.SoftAvoidanceResponseRate <= 0f)
            return;

        int oracleCount = 0;
        int missingCount = 0;
        for (int bodyA = 0; bodyA < bodies.Length; bodyA++)
        {
            if (bodies[bodyA].IsInsideSimulationDomain == 0)
                continue;
            for (int bodyB = bodyA + 1; bodyB < bodies.Length; bodyB++)
            {
                if (bodies[bodyB].IsInsideSimulationDomain == 0 ||
                    !CouldEnterSoftAvoidanceRange(
                        bodyA,
                        bodyB,
                        configuration,
                        bodies,
                        motionEvidence,
                        stepStates))
                    continue;
                oracleCount++;
                if (ContactPipelineShared.FindBodyPairIndex(softAvoidancePairs, bodyA, bodyB) < 0)
                    missingCount++;
            }
        }
        incrementalStatistics.SoftAvoidanceOraclePairCount += oracleCount;
        incrementalStatistics.SoftAvoidanceOracleMissingPairCount +=
            missingCount;
        if (missingCount > 0)
            incrementalStatistics.OracleMismatch = 1;
    }

    private static bool CouldEnterSoftAvoidanceRange(
        int bodyAIndex,
        int bodyBIndex,
        ContactPipelineConfiguration configuration,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdMotionEvidence> motionEvidence,
        NativeArray<CrowdSolverBodyState> stepStates)
    {
        CrowdBodySnapshot bodyA = bodies[bodyAIndex];
        CrowdBodySnapshot bodyB = bodies[bodyBIndex];
        CrowdMotionEvidence evidenceA = motionEvidence[bodyAIndex];
        CrowdMotionEvidence evidenceB = motionEvidence[bodyBIndex];
        CrowdSolverBodyState stepA = stepStates[bodyAIndex];
        CrowdSolverBodyState stepB = stepStates[bodyBIndex];
        float maxDistance =
            bodyA.Radius + bodyB.Radius +
            math.max(0f, configuration.SoftAvoidanceShell);
        float3 relativeStart =
            evidenceB.TrajectoryStart - evidenceA.TrajectoryStart;
        float3 relativeDisplacement =
            (evidenceB.BaselineEnd - evidenceB.TrajectoryStart) -
            (evidenceA.BaselineEnd - evidenceA.TrajectoryStart);
        relativeStart.y = 0f;
        relativeDisplacement.y = 0f;
        if (CouldRelativePathApproach(
                relativeStart,
                relativeDisplacement,
                maxDistance))
            return true;
        if (configuration.SoftAvoidanceVelocitySolver !=
            SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle)
            return false;
        float3 relativeHorizonDisplacement =
            (stepB.BaseVelocity - stepA.BaseVelocity) *
            math.max(0f, configuration.RvoTimeHorizon);
        relativeHorizonDisplacement.y = 0f;
        return CouldRelativePathApproach(
            relativeStart,
            relativeHorizonDisplacement,
            maxDistance);
    }
}
}
