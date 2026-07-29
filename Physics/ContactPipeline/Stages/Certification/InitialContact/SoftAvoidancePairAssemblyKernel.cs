using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal partial struct CertificationStageKernel
{

    private void BuildSoftAvoidancePairViewFromInteractions(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        SoftAvoidancePairs.Clear();
        if (Configuration.SoftAvoidanceShell <= 0f || Configuration.SoftAvoidanceResponseRate <= 0f)
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
        if (Configuration.SoftAvoidanceShell <= 0f || Configuration.SoftAvoidanceResponseRate <= 0f)
            return false;

        CrowdBodySnapshot bodyA = Bodies[bodyAIndex];
        CrowdBodySnapshot bodyB = Bodies[bodyBIndex];
        CrowdMotionEvidence evidenceA = MotionEvidence[bodyAIndex];
        CrowdMotionEvidence evidenceB = MotionEvidence[bodyBIndex];
        CrowdBodyStepState stepA = StepStates[bodyAIndex];
        CrowdBodyStepState stepB = StepStates[bodyBIndex];

        float maxDistance = bodyA.Radius + bodyB.Radius +
                            math.max(0f, Configuration.SoftAvoidanceShell);
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

        if (Configuration.SoftAvoidanceVelocitySolver !=
            SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle)
            return false;

        float3 relativeHorizonDisplacement =
            (stepB.BaseVelocity - stepA.BaseVelocity) *
            math.max(0f, Configuration.RvoTimeHorizon);
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
        if (!Configuration.EnableDiagnostics || Configuration.SoftAvoidanceShell <= 0f ||
            Configuration.SoftAvoidanceResponseRate <= 0f)
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
}
}
