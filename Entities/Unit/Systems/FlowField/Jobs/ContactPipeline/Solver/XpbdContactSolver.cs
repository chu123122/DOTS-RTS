using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public struct JacobiPairCorrection
{
    public float3 DeltaA;
    public float3 DeltaB;
    public float PairCorrection;
    public byte ActiveA;
    public byte ActiveB;
    public byte NewlyActivated;
    public byte NewlyCorrected;
}

public struct ContactConstraintEvaluation
{
    public float3 Normal;
    public float ConstraintValue;
    public float AppliedLambda;
    public float PairCorrection;
    public byte NewlyActivated;
}

public static class XpbdContactConstraintMath
{
    public static ContactConstraintEvaluation Evaluate(
        ref UnitCollisionPair pair,
        FlowMovementFrameState bodyA,
        FlowMovementFrameState bodyB,
        float alpha,
        int substepIndex)
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
                    SolveXpbdUnitContactsJob.DeterministicFallbackNormal(
                        pair.BodyA,
                        pair.BodyB));
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
                : SolveXpbdUnitContactsJob.DeterministicFallbackNormal(
                    pair.BodyA,
                    pair.BodyB);
            constraintValue = distance - radiusSum;
        }

        float deltaLambda = -(constraintValue + alpha * pair.Lambda) / denominator;
        float nextLambda = math.max(0f, pair.Lambda + deltaLambda);
        float appliedLambda = nextLambda - pair.Lambda;
        pair.Lambda = nextLambda;
        byte newlyActivated = 0;
        if (nextLambda > 0.0000001f && pair.WasActivated == 0)
        {
            pair.WasActivated = 1;
            pair.ActivatedSubstepCount++;
            if (pair.WasActivatedThisTimestep == 0)
            {
                pair.WasActivatedThisTimestep = 1;
                pair.FirstActivatedSubstep = substepIndex;
                newlyActivated = 1;
            }
        }

        return new ContactConstraintEvaluation
        {
            Normal = normal,
            ConstraintValue = constraintValue,
            AppliedLambda = appliedLambda,
            PairCorrection = (bodyA.InverseMass + bodyB.InverseMass) *
                             math.abs(appliedLambda),
            NewlyActivated = newlyActivated
        };
    }
}

/// <summary>
/// XPBD projection for the compact frame-local active contact view.
/// Gauss-Seidel writes each pair immediately; Jacobi evaluates from one
/// position snapshot and applies averaged per-body corrections afterwards.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
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
            ContactConstraintEvaluation evaluation = XpbdContactConstraintMath.Evaluate(
                ref pair,
                bodyA,
                bodyB,
                alpha,
                substepIndex);
            statistics.TimestepContactSetUniqueActivatedPairCount +=
                evaluation.NewlyActivated;
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
        JacobiPairCorrections.ResizeUninitialized(TimestepContactPairs.Length);

        float alpha = Compliance / (substepDeltaTime * substepDeltaTime);
        incrementalStatistics.ActiveConstraintEvaluationCount +=
            TimestepContactPairs.Length;

        // All pairs read the same predicted-position state and write only their
        // own contribution slot. The body gather below is the same data shape
        // used by the parallel Jacobi path.
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            ContactConstraintEvaluation evaluation = XpbdContactConstraintMath.Evaluate(
                ref pair,
                bodyA,
                bodyB,
                alpha,
                substepIndex);
            statistics.TimestepContactSetUniqueActivatedPairCount +=
                evaluation.NewlyActivated;
            TimestepContactPairs[pairIndex] = pair;

            CaptureSimulationDebuggerPair(
                substepIndex,
                pair,
                bodyA,
                bodyB,
                evaluation.Normal,
                evaluation.ConstraintValue,
                evaluation.PairCorrection);

            JacobiPairCorrection correction = default;
            if (math.abs(evaluation.AppliedLambda) > 0.0000001f)
            {
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
                    correction.DeltaA = evaluation.Normal *
                        (bodyA.InverseMass * evaluation.AppliedLambda);
                    correction.ActiveA = 1;
                }
                if (bodyB.InverseMass > 0f)
                {
                    correction.DeltaB = -evaluation.Normal *
                        (bodyB.InverseMass * evaluation.AppliedLambda);
                    correction.ActiveB = 1;
                }
            }
            JacobiPairCorrections[pairIndex] = correction;
        }

        // Each body owns one disjoint output slot and gathers only the pair
        // contributions listed in its CSR incident range. No atomics are needed.
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            float3 correctionSum = float3.zero;
            int correctionCount = 0;
            int begin = ActiveIncidentOffsets[bodyIndex];
            int end = ActiveIncidentOffsets[bodyIndex + 1];
            for (int incidentIndex = begin; incidentIndex < end; incidentIndex++)
            {
                int pairIndex = ActiveIncidentPairIndices[incidentIndex];
                UnitCollisionPair pair = TimestepContactPairs[pairIndex];
                JacobiPairCorrection contribution = JacobiPairCorrections[pairIndex];
                if (pair.BodyA == bodyIndex && contribution.ActiveA != 0)
                {
                    correctionSum += contribution.DeltaA;
                    correctionCount++;
                }
                else if (pair.BodyB == bodyIndex && contribution.ActiveB != 0)
                {
                    correctionSum += contribution.DeltaB;
                    correctionCount++;
                }
            }

            if (correctionCount <= 0)
                continue;

            // Constraint averaging prevents a high-degree body from applying
            // all simultaneous corrections at full strength in one Jacobi step.
            FlowMovementFrameState body = States[bodyIndex];
            ApplyContactCorrection(ref body, correctionSum / correctionCount);
            States[bodyIndex] = body;
            if (trackCorrectedBodies)
                MarkCorrectedBody(bodyIndex);
        }
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
