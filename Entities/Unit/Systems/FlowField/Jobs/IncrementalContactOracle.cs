using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct SolveXpbdUnitContactsJob
{
    /// <summary>
    /// Diagnostic-only O(N^2) oracle. It evaluates the exact swept-disc test
    /// independently of the incremental topology and records false negatives.
    /// </summary>
    private void ValidateIncrementalContactSetAgainstQuadraticOracle(
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        if (!EnableDiagnostics)
            return;

        IncrementalOracleContactPairs.Clear();
        float skin = math.max(0f, PredictiveSkin);
        for (int bodyAIndex = 0; bodyAIndex < States.Length; bodyAIndex++)
        {
            FlowMovementFrameState bodyA = States[bodyAIndex];
            if (!bodyA.IsInsideGrid)
                continue;

            for (int bodyBIndex = bodyAIndex + 1;
                 bodyBIndex < States.Length;
                 bodyBIndex++)
            {
                FlowMovementFrameState bodyB = States[bodyBIndex];
                if (!bodyB.IsInsideGrid)
                    continue;

                float radiusSum = bodyA.Radius + bodyB.Radius;
                float3 relativeStart =
                    bodyB.TimestepStartPosition - bodyA.TimestepStartPosition;
                float3 relativeDisplacement =
                    (bodyB.TimestepPredictedPosition - bodyB.TimestepStartPosition) -
                    (bodyA.TimestepPredictedPosition - bodyA.TimestepStartPosition);
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

                IncrementalOracleContactPairs.Add(new UnitCollisionPair
                {
                    BodyA = bodyAIndex,
                    BodyB = bodyBIndex
                });
            }
        }

        SortAndDeduplicateBodyPairs(IncrementalOracleContactPairs);
        int missingPairCount = 0;
        int extraPairCount = 0;

        for (int oracleIndex = 0;
             oracleIndex < IncrementalOracleContactPairs.Length;
             oracleIndex++)
        {
            UnitCollisionPair oraclePair = IncrementalOracleContactPairs[oracleIndex];
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
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
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

        incrementalStatistics.OracleMismatch = 1;
        IncrementalContactCacheState cacheState = IncrementalCacheState.Value;
        cacheState.IsValid = 0;
        IncrementalCacheState.Value = cacheState;
    }
}
}
