using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

#if RTS_CONTACT_DIAGNOSTICS
namespace RTS.Unit.FlowField.Jobs
{
internal static class ContactIterationDiagnostics
{
    internal static void Record(
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdBodyStepState> stepStates,
        NativeList<ContactConstraint> constraints,
        NativeList<ContactIterationDiagnostic> diagnostics,
        int substepIndex,
        int iterationIndex,
        float maxViolationBeforeSolve,
        float averageViolationBeforeSolve,
        float totalPositionCorrection,
        float maxPositionCorrection,
        float totalWallPositionCorrection,
        float maxWallPositionCorrection)
    {
        float violationSum = 0f;
        float radialPenetrationSum = 0f;
        float maxViolation = 0f;
        float maxRadialPenetration = 0f;
        int violatingCount = 0;
        int penetratingCount = 0;
        int activeCount = 0;
        int predictiveActivatedCount = 0;

        for (int i = 0; i < constraints.Length; i++)
        {
            ContactConstraint pair = constraints[i];
            CrowdBodySnapshot bodyA = bodies[pair.BodyA];
            CrowdBodySnapshot bodyB = bodies[pair.BodyB];
            float radiusSum = bodyA.Radius + bodyB.Radius;
            float3 currentDelta =
                stepStates[pair.BodyA].SolvedPosition -
                stepStates[pair.BodyB].SolvedPosition;
            currentDelta.y = 0f;

            float constraintValue = pair.ContactMode == ContactConstraintMode.Predictive
                ? math.dot(currentDelta, pair.PredictiveNormal) - radiusSum
                : math.length(currentDelta) - radiusSum;
            float violation = math.max(0f, -constraintValue);
            if (violation > 0f)
            {
                violationSum += violation;
                maxViolation = math.max(maxViolation, violation);
                violatingCount++;
            }

            float radialPenetration = math.max(
                0f,
                radiusSum - math.length(currentDelta));
            if (radialPenetration > 0f)
            {
                radialPenetrationSum += radialPenetration;
                maxRadialPenetration = math.max(
                    maxRadialPenetration,
                    radialPenetration);
                penetratingCount++;
            }

            if (pair.Lambda <= 0.0000001f)
                continue;

            activeCount++;
            if (pair.ContactMode == ContactConstraintMode.Predictive)
                predictiveActivatedCount++;
        }

        diagnostics.Add(new ContactIterationDiagnostic
        {
            SubstepIndex = substepIndex,
            IterationIndex = iterationIndex,
            ActiveConstraintCount = activeCount,
            PredictiveActivatedCount = predictiveActivatedCount,
            MaxConstraintViolationBeforeSolve = maxViolationBeforeSolve,
            AverageConstraintViolationBeforeSolve = averageViolationBeforeSolve,
            MaxConstraintViolation = maxViolation,
            AverageConstraintViolation = violatingCount > 0
                ? violationSum / violatingCount
                : 0f,
            MaxRadialPenetration = maxRadialPenetration,
            AverageRadialPenetration = penetratingCount > 0
                ? radialPenetrationSum / penetratingCount
                : 0f,
            TotalPositionCorrection = totalPositionCorrection,
            MaxPositionCorrection = maxPositionCorrection,
            TotalWallPositionCorrection = totalWallPositionCorrection,
            MaxWallPositionCorrection = maxWallPositionCorrection
        });
    }
}

internal partial struct CertificationStageKernel
{
    private void RecordIterationDiagnostic(
        int substepIndex,
        int iterationIndex,
        float maxViolationBeforeSolve,
        float averageViolationBeforeSolve,
        float totalPositionCorrection,
        float maxPositionCorrection,
        float totalWallPositionCorrection,
        float maxWallPositionCorrection)
    {
        ContactIterationDiagnostics.Record(
            Bodies,
            StepStates,
            TimestepContactPairs,
            IterationDiagnostics,
            substepIndex,
            iterationIndex,
            maxViolationBeforeSolve,
            averageViolationBeforeSolve,
            totalPositionCorrection,
            maxPositionCorrection,
            totalWallPositionCorrection,
            maxWallPositionCorrection);
    }
}
}

#endif
