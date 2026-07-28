using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct InteractionCertificationJob
{
#if RTS_CONTACT_DIAGNOSTICS
    /// <summary>
    /// 仅诊断用的 O(N²) Oracle。独立执行准确的 swept-disc 测试，不依赖增量拓扑，并记录漏报。
    /// </summary>
    private void ValidateIncrementalContactSetAgainstQuadraticOracle(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        if (!EnableDiagnostics)
            return;

        IncrementalOracleContactPairs.Clear();
        float skin = math.max(0f, PredictiveSkin);
        for (int bodyAIndex = 0; bodyAIndex < Bodies.Length; bodyAIndex++)
        {
            CrowdBodySnapshot bodyASnapshot = Bodies[bodyAIndex];
            CrowdNavigationState bodyANavigation = NavigationStates[bodyAIndex];
            CrowdMotionIntent bodyAIntent = MotionIntents[bodyAIndex];
            CrowdMotionEvidence bodyAEvidence = MotionEvidence[bodyAIndex];
            CrowdBodyStepState bodyAStep = StepStates[bodyAIndex];
            if (!(bodyASnapshot.IsInsideSimulationDomain != 0))
                continue;

            for (int bodyBIndex = bodyAIndex + 1;
                 bodyBIndex < Bodies.Length;
                 bodyBIndex++)
            {
                CrowdBodySnapshot bodyBSnapshot = Bodies[bodyBIndex];
                CrowdNavigationState bodyBNavigation = NavigationStates[bodyBIndex];
                CrowdMotionIntent bodyBIntent = MotionIntents[bodyBIndex];
                CrowdMotionEvidence bodyBEvidence = MotionEvidence[bodyBIndex];
                CrowdBodyStepState bodyBStep = StepStates[bodyBIndex];
                if (!(bodyBSnapshot.IsInsideSimulationDomain != 0))
                    continue;

                float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;
                float3 relativeStart =
                    bodyBEvidence.TrajectoryStart - bodyAEvidence.TrajectoryStart;
                float3 relativeDisplacement =
                    (bodyBEvidence.BaselineEnd - bodyBEvidence.TrajectoryStart) -
                    (bodyAEvidence.BaselineEnd - bodyAEvidence.TrajectoryStart);
                relativeStart.y = 0f;
                relativeDisplacement.y = 0f;
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
                float candidateDistance = radiusSum + skin;
                if (minDistanceSq > candidateDistance * candidateDistance)
                    continue;

                bool startsOverlapping =
                    math.lengthsq(relativeStart) <= radiusSum * radiusSum;
                if (!startsOverlapping && !EnablePredictivePairGeneration)
                    continue;

                IncrementalOracleContactPairs.Add(new BodyPair
                {
                    BodyA = bodyAIndex,
                    BodyB = bodyBIndex
                });
            }
        }

        ContactPipelineShared.SortAndDeduplicateBodyPairs(IncrementalOracleContactPairs);
        int missingPairCount = 0;
        int extraPairCount = 0;

        for (int oracleIndex = 0;
             oracleIndex < IncrementalOracleContactPairs.Length;
             oracleIndex++)
        {
            BodyPair oraclePair = IncrementalOracleContactPairs[oracleIndex];
            if (FindPairIndex(
                    TimestepContactPairs,
                    oraclePair.BodyA,
                    oraclePair.BodyB) < 0)
            {
                missingPairCount++;
            }
        }

        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            ContactConstraint pair = TimestepContactPairs[pairIndex];
            if (FindPairIndex(
                    IncrementalOracleContactPairs,
                    pair.BodyA,
                    pair.BodyB) < 0)
            {
                extraPairCount++;
            }
        }

        incrementalStatistics.OraclePairCount += IncrementalOracleContactPairs.Length;
        incrementalStatistics.OracleMissingPairCount += missingPairCount;
        incrementalStatistics.OracleExtraPairCount += extraPairCount;
        if (missingPairCount <= 0)
            return;

        // 校验仅作观测；缓存失效属于显式的玩法正确性策略，跟诊断无关。
        incrementalStatistics.OracleMismatch = 1;
    }
#else
    private void ValidateIncrementalContactSetAgainstQuadraticOracle(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
    }
#endif
}
}
