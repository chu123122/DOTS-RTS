using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// XPBD projection for the compact frame-local active contact view.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private void SolveContactIteration(
        float substepDeltaTime,
        int substepIndex,
        bool trackCorrectedBodies,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        out float totalPositionCorrection,
        out float maxPositionCorrection)
    {
        if (ContactPositionSolver == ContactPositionSolverMode.Jacobi)
        {
            SolveSerialJacobiContactIteration(
                substepDeltaTime,
                substepIndex,
                trackCorrectedBodies,
                ref statistics,
                ref incrementalStatistics,
                out totalPositionCorrection,
                out maxPositionCorrection);
            return;
        }

        SolveGaussSeidelContactIteration(
            substepDeltaTime,
            substepIndex,
            trackCorrectedBodies,
            ref statistics,
            ref incrementalStatistics,
            out totalPositionCorrection,
            out maxPositionCorrection);
    }

    private void SolveGaussSeidelContactIteration(
        float substepDeltaTime,
        int substepIndex,
        bool trackCorrectedBodies,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        out float totalPositionCorrection,
        out float maxPositionCorrection)
    {
        if (trackCorrectedBodies)
            ResetCorrectedBodyTracking();

        totalPositionCorrection = 0f;
        maxPositionCorrection = 0f;
        float alpha = Compliance / (substepDeltaTime * substepDeltaTime);
        incrementalStatistics.ActiveConstraintEvaluationCount +=
            TimestepContactPairs.Length;

        for (int i = 0; i < TimestepContactPairs.Length; i++)
        {
            UnitCollisionPair pair = TimestepContactPairs[i];
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
                normal = pair.PredictiveNormal;
                if (pair.PredictiveNormalOriented == 0)
                {
                    // BodyIndex ordering is frame-local. Orient the persistent
                    // normal once for this timestep, then keep it fixed even if
                    // the pair later crosses to the opposite side.
                    if (math.dot(currentDelta, normal) < 0f)
                        normal = -normal;
                    normal = math.normalizesafe(
                        normal,
                        DeterministicFallbackNormal(pair.BodyA, pair.BodyB));
                    pair.PredictiveNormal = normal;
                    pair.PredictiveNormalOriented = 1;
                }
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

            if (nextLambda > 0.0000001f && pair.WasActivated == 0)
            {
                pair.WasActivated = 1;
                pair.ActivatedSubstepCount++;
                if (pair.WasActivatedThisTimestep == 0)
                {
                    pair.WasActivatedThisTimestep = 1;
                    pair.FirstActivatedSubstep = substepIndex;
                    statistics.TimestepContactSetUniqueActivatedPairCount++;
                }
            }
            TimestepContactPairs[i] = pair;

            float pairCorrection =
                (bodyA.InverseMass + bodyB.InverseMass) * math.abs(appliedLambda);
            CaptureSimulationDebuggerPair(
                substepIndex,
                pair,
                bodyA,
                bodyB,
                normal,
                constraintValue,
                pairCorrection);

            if (math.abs(appliedLambda) <= 0.0000001f)
                continue;

            if (pair.WasCorrectedThisTimestep == 0)
            {
                pair.WasCorrectedThisTimestep = 1;
                incrementalStatistics.UniqueCorrectedPairCount++;
                TimestepContactPairs[i] = pair;
            }

            totalPositionCorrection += pairCorrection;
            maxPositionCorrection = math.max(maxPositionCorrection, pairCorrection);

            bodyA.PredictedPosition += normal * (bodyA.InverseMass * appliedLambda);
            bodyB.PredictedPosition -= normal * (bodyB.InverseMass * appliedLambda);
            bodyA.ContactPositionCorrection += normal * (bodyA.InverseMass * appliedLambda);
            bodyB.ContactPositionCorrection -= normal * (bodyB.InverseMass * appliedLambda);
            bodyA.TimestepContactCorrection += normal * (bodyA.InverseMass * appliedLambda);
            bodyB.TimestepContactCorrection -= normal * (bodyB.InverseMass * appliedLambda);
            bodyA.PredictedPosition.y = bodyA.CurrentPosition.y;
            bodyB.PredictedPosition.y = bodyB.CurrentPosition.y;
            States[pair.BodyA] = bodyA;
            States[pair.BodyB] = bodyB;

            if (trackCorrectedBodies)
            {
                if (bodyA.InverseMass > 0f)
                    MarkCorrectedBody(pair.BodyA);
                if (bodyB.InverseMass > 0f)
                    MarkCorrectedBody(pair.BodyB);
            }
        }
    }


    /// <summary>
    /// Deterministic serial Jacobi reference using the same CSR gather layout
    /// that phase 4 parallelizes. Constraints read a shared iteration snapshot;
    /// each body then gathers and averages only its non-zero incident projections.
    /// </summary>
    private void SolveSerialJacobiContactIteration(
        float substepDeltaTime,
        int substepIndex,
        bool trackCorrectedBodies,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        out float totalPositionCorrection,
        out float maxPositionCorrection)
    {
        if (trackCorrectedBodies)
            ResetCorrectedBodyTracking();

        EnsureActiveConstraintIncidentIndex();
        JacobiConstraintProjections.ResizeUninitialized(TimestepContactPairs.Length);
        totalPositionCorrection = 0f;
        maxPositionCorrection = 0f;
        float alpha = Compliance / (substepDeltaTime * substepDeltaTime);
        incrementalStatistics.ActiveConstraintEvaluationCount +=
            TimestepContactPairs.Length;

        for (int i = 0; i < TimestepContactPairs.Length; i++)
        {
            UnitCollisionPair pair = TimestepContactPairs[i];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            JacobiContactProjection projection = default;

            float denominator = bodyA.InverseMass + bodyB.InverseMass + alpha;
            if (denominator <= 0f)
            {
                JacobiConstraintProjections[i] = projection;
                continue;
            }

            float3 currentDelta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            currentDelta.y = 0f;
            float radiusSum = bodyA.Radius + bodyB.Radius;
            float3 normal;
            float constraintValue;

            if (pair.ContactMode == UnitContactMode.Predictive)
            {
                normal = pair.PredictiveNormal;
                if (pair.PredictiveNormalOriented == 0)
                {
                    if (math.dot(currentDelta, normal) < 0f)
                        normal = -normal;
                    normal = math.normalizesafe(
                        normal,
                        DeterministicFallbackNormal(pair.BodyA, pair.BodyB));
                    pair.PredictiveNormal = normal;
                    pair.PredictiveNormalOriented = 1;
                }
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

            if (nextLambda > 0.0000001f && pair.WasActivated == 0)
            {
                pair.WasActivated = 1;
                pair.ActivatedSubstepCount++;
                if (pair.WasActivatedThisTimestep == 0)
                {
                    pair.WasActivatedThisTimestep = 1;
                    pair.FirstActivatedSubstep = substepIndex;
                    statistics.TimestepContactSetUniqueActivatedPairCount++;
                }
            }

            float pairCorrection =
                (bodyA.InverseMass + bodyB.InverseMass) * math.abs(appliedLambda);
            CaptureSimulationDebuggerPair(
                substepIndex,
                pair,
                bodyA,
                bodyB,
                normal,
                constraintValue,
                pairCorrection);

            if (math.abs(appliedLambda) > 0.0000001f)
            {
                if (pair.WasCorrectedThisTimestep == 0)
                {
                    pair.WasCorrectedThisTimestep = 1;
                    incrementalStatistics.UniqueCorrectedPairCount++;
                }
                totalPositionCorrection += pairCorrection;
                maxPositionCorrection = math.max(maxPositionCorrection, pairCorrection);
                projection.Normal = normal;
                projection.AppliedLambda = appliedLambda;
                projection.ConstraintValue = constraintValue;
                projection.PairCorrection = pairCorrection;
            }

            TimestepContactPairs[i] = pair;
            JacobiConstraintProjections[i] = projection;
        }

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            float3 correctionSum = float3.zero;
            int activeContributionCount = 0;
            int begin = ActiveConstraintIncidentOffsets[bodyIndex];
            int endIndex = ActiveConstraintIncidentOffsets[bodyIndex + 1];
            FlowMovementFrameState body = States[bodyIndex];

            for (int incidentIndex = begin; incidentIndex < endIndex; incidentIndex++)
            {
                int pairIndex = ActiveConstraintIncidentPairIndices[incidentIndex];
                JacobiContactProjection projection =
                    JacobiConstraintProjections[pairIndex];
                if (math.abs(projection.AppliedLambda) <= 0.0000001f)
                    continue;

                UnitCollisionPair pair = TimestepContactPairs[pairIndex];
                float direction = pair.BodyA == bodyIndex ? 1f : -1f;
                correctionSum += projection.Normal *
                    (direction * body.InverseMass * projection.AppliedLambda);
                activeContributionCount++;
            }

            if (activeContributionCount <= 0)
            {
                JacobiPositionCorrections[bodyIndex] = float3.zero;
                continue;
            }
            JacobiPositionCorrections[bodyIndex] =
                correctionSum / activeContributionCount;
        }

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            float3 correction = JacobiPositionCorrections[bodyIndex];
            if (math.lengthsq(correction) <= 0.00000000000001f)
                continue;

            FlowMovementFrameState body = States[bodyIndex];
            body.PredictedPosition += correction;
            body.ContactPositionCorrection += correction;
            body.TimestepContactCorrection += correction;
            body.PredictedPosition.y = body.CurrentPosition.y;
            States[bodyIndex] = body;
            if (trackCorrectedBodies)
                MarkCorrectedBody(bodyIndex);
        }
    }
}
}
