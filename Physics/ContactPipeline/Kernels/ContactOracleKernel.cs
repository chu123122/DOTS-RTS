using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal static class ContactOracleKernel
{
#if RTS_CONTACT_DIAGNOSTICS
    /// <summary>
    /// 仅诊断用的 O(N²) Oracle。独立执行准确的 swept-disc 测试，不依赖增量拓扑，并记录漏报。
    /// </summary>


    internal static void ValidateIncrementalContactSetAgainstQuadraticOracle(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        ContactPipelineConfiguration configuration,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdMotionEvidence> motionEvidence,
        NativeList<ContactConstraint> timestepContactPairs,
        NativeList<BodyPair> oracleContactPairs)
    {
        if (!configuration.EnableDiagnostics)
            return;

        oracleContactPairs.Clear();
        float skin = math.max(0f, configuration.PredictiveSkin);
        for (int bodyAIndex = 0;
             bodyAIndex < bodies.Length;
             bodyAIndex++)
        {
            CrowdBodySnapshot bodyA = bodies[bodyAIndex];
            CrowdMotionEvidence evidenceA = motionEvidence[bodyAIndex];
            if (bodyA.IsInsideSimulationDomain == 0)
                continue;
            for (int bodyBIndex = bodyAIndex + 1;
                 bodyBIndex < bodies.Length;
                 bodyBIndex++)
            {
                CrowdBodySnapshot bodyB = bodies[bodyBIndex];
                CrowdMotionEvidence evidenceB = motionEvidence[bodyBIndex];
                if (bodyB.IsInsideSimulationDomain == 0)
                    continue;
                float radiusSum = bodyA.Radius + bodyB.Radius;
                float3 relativeStart =
                    evidenceB.TrajectoryStart - evidenceA.TrajectoryStart;
                float3 relativeDisplacement =
                    (evidenceB.BaselineEnd - evidenceB.TrajectoryStart) -
                    (evidenceA.BaselineEnd - evidenceA.TrajectoryStart);
                relativeStart.y = 0f;
                relativeDisplacement.y = 0f;
                float relativeLengthSq =
                    math.lengthsq(relativeDisplacement);
                float closestTime = relativeLengthSq > 0.0000001f
                    ? math.clamp(
                        -math.dot(relativeStart, relativeDisplacement) /
                        relativeLengthSq,
                        0f,
                        1f)
                    : 0f;
                float minDistanceSq = math.lengthsq(
                    relativeStart + closestTime * relativeDisplacement);
                float candidateDistance = radiusSum + skin;
                if (minDistanceSq >
                    candidateDistance * candidateDistance)
                    continue;
                bool startsOverlapping =
                    math.lengthsq(relativeStart) <=
                    radiusSum * radiusSum;
                if (!startsOverlapping &&
                    !configuration.EnablePredictivePairGeneration)
                    continue;
                oracleContactPairs.Add(
                    new BodyPair(bodyAIndex, bodyBIndex));
            }
        }

        ContactPipelineShared.SortAndDeduplicateBodyPairs(
            oracleContactPairs);
        int missingPairCount = 0;
        int extraPairCount = 0;
        for (int oracleIndex = 0;
             oracleIndex < oracleContactPairs.Length;
             oracleIndex++)
        {
            BodyPair pair = oracleContactPairs[oracleIndex];
            if (ContactPipelineShared.FindConstraintIndex(
                    timestepContactPairs,
                    pair.BodyA,
                    pair.BodyB) < 0)
                missingPairCount++;
        }
        for (int pairIndex = 0;
             pairIndex < timestepContactPairs.Length;
             pairIndex++)
        {
            ContactConstraint pair = timestepContactPairs[pairIndex];
            if (ContactPipelineShared.FindBodyPairIndex(
                    oracleContactPairs,
                    pair.BodyA,
                    pair.BodyB) < 0)
                extraPairCount++;
        }

        incrementalStatistics.OraclePairCount +=
            oracleContactPairs.Length;
        incrementalStatistics.OracleMissingPairCount +=
            missingPairCount;
        incrementalStatistics.OracleExtraPairCount += extraPairCount;
        if (missingPairCount > 0)
            incrementalStatistics.OracleMismatch = 1;
    }
#else


    internal static void ValidateIncrementalContactSetAgainstQuadraticOracle(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        ContactPipelineConfiguration configuration,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdMotionEvidence> motionEvidence,
        NativeList<ContactConstraint> timestepContactPairs,
        NativeList<BodyPair> oracleContactPairs)
    {
    }
#endif
}
}
