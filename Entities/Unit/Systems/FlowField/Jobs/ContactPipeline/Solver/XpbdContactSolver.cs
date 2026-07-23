using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// XPBD projection for the compact frame-local active contact view.
/// Gauss-Seidel writes each pair immediately; Jacobi evaluates from one
/// position snapshot and applies averaged per-body corrections afterwards.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private struct ContactConstraintEvaluation
    {
        public float3 Normal;
        public float ConstraintValue;
        public float AppliedLambda;
        public float PairCorrection;
    }

    private void SolveConfiguredContactIteration(
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
            SolveJacobiContactIteration(
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

        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            ContactConstraintEvaluation evaluation = EvaluateContactConstraint(
                ref pair,
                bodyA,
                bodyB,
                alpha,
                substepIndex,
                ref statistics);
            TimestepContactPairs[pairIndex] = pair;

            CaptureSimulationDebuggerPair(
                substepIndex,
                pair,
                bodyA,
                bodyB,
                evaluation.Normal,
                evaluation.ConstraintValue,
                evaluation.PairCorrection);

            if (math.abs(evaluation.AppliedLambda) <= 0.0000001f)
                continue;

            MarkPairCorrectedThisTimestep(
                pairIndex,
                ref pair,
                ref incrementalStatistics);
            totalPositionCorrection += evaluation.PairCorrection;
            maxPositionCorrection = math.max(
                maxPositionCorrection,
                evaluation.PairCorrection);

            float3 correctionA = evaluation.Normal *
                                 (bodyA.InverseMass * evaluation.AppliedLambda);
            float3 correctionB = -evaluation.Normal *
                                 (bodyB.InverseMass * evaluation.AppliedLambda);
            ApplyContactCorrection(ref bodyA, correctionA);
            ApplyContactCorrection(ref bodyB, correctionB);
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

    private void SolveJacobiContactIteration(
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
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            JacobiBodyCorrectionSums[bodyIndex] = float3.zero;
            JacobiBodyCorrectionCounts[bodyIndex] = 0;
        }

        float alpha = Compliance / (substepDeltaTime * substepDeltaTime);
        incrementalStatistics.ActiveConstraintEvaluationCount +=
            TimestepContactPairs.Length;

        // All pairs read the same predicted-position state. No body position is
        // changed until every pair has produced its contribution.
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            ContactConstraintEvaluation evaluation = EvaluateContactConstraint(
                ref pair,
                bodyA,
                bodyB,
                alpha,
                substepIndex,
                ref statistics);
            TimestepContactPairs[pairIndex] = pair;

            CaptureSimulationDebuggerPair(
                substepIndex,
                pair,
                bodyA,
                bodyB,
                evaluation.Normal,
                evaluation.ConstraintValue,
                evaluation.PairCorrection);

            if (math.abs(evaluation.AppliedLambda) <= 0.0000001f)
                continue;

            MarkPairCorrectedThisTimestep(
                pairIndex,
                ref pair,
                ref incrementalStatistics);
            totalPositionCorrection += evaluation.PairCorrection;
            maxPositionCorrection = math.max(
                maxPositionCorrection,
                evaluation.PairCorrection);

            if (bodyA.InverseMass > 0f)
            {
                JacobiBodyCorrectionSums[pair.BodyA] += evaluation.Normal *
                    (bodyA.InverseMass * evaluation.AppliedLambda);
                JacobiBodyCorrectionCounts[pair.BodyA]++;
            }
            if (bodyB.InverseMass > 0f)
            {
                JacobiBodyCorrectionSums[pair.BodyB] -= evaluation.Normal *
                    (bodyB.InverseMass * evaluation.AppliedLambda);
                JacobiBodyCorrectionCounts[pair.BodyB]++;
            }
        }

        // Constraint averaging prevents a high-degree body from applying every
        // simultaneously-computed correction at full strength in one Jacobi step.
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            int correctionCount = JacobiBodyCorrectionCounts[bodyIndex];
            if (correctionCount <= 0)
                continue;

            float3 correction = JacobiBodyCorrectionSums[bodyIndex] /
                                correctionCount;
            FlowMovementFrameState body = States[bodyIndex];
            ApplyContactCorrection(ref body, correction);
            States[bodyIndex] = body;
            if (trackCorrectedBodies)
                MarkCorrectedBody(bodyIndex);
        }
    }

    private ContactConstraintEvaluation EvaluateContactConstraint(
        ref UnitCollisionPair pair,
        FlowMovementFrameState bodyA,
        FlowMovementFrameState bodyB,
        float alpha,
        int substepIndex,
        ref PredictiveDiscContactStatistics statistics)
    {
        float denominator = bodyA.InverseMass + bodyB.InverseMass + alpha;
        if (denominator <= 0f)
            return default;

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

        return new ContactConstraintEvaluation
        {
            Normal = normal,
            ConstraintValue = constraintValue,
            AppliedLambda = appliedLambda,
            PairCorrection = (bodyA.InverseMass + bodyB.InverseMass) *
                             math.abs(appliedLambda)
        };
    }

    private void MarkPairCorrectedThisTimestep(
        int pairIndex,
        ref UnitCollisionPair pair,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        if (pair.WasCorrectedThisTimestep != 0)
            return;
        pair.WasCorrectedThisTimestep = 1;
        incrementalStatistics.UniqueCorrectedPairCount++;
        TimestepContactPairs[pairIndex] = pair;
    }

    private static void ApplyContactCorrection(
        ref FlowMovementFrameState body,
        float3 correction)
    {
        body.PredictedPosition += correction;
        body.ContactPositionCorrection += correction;
        body.TimestepContactCorrection += correction;
        body.PredictedPosition.y = body.CurrentPosition.y;
    }
}
}
