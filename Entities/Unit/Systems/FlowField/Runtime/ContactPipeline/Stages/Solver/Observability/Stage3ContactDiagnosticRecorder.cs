#if RTS_CONTACT_DIAGNOSTICS
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct ConstraintSolverJob
{
    private void AccumulateConstraintStatistics(
        ref PredictiveDiscContactStatistics statistics,
        ref float penetrationSum)
    {
        if (!EnableDiagnostics) return;
        for (int i = 0; i < TimestepContactPairs.Length; i++)
        {
            ContactConstraint pair = TimestepContactPairs[i];
            CrowdBodySnapshot bodyASnapshot = Bodies[pair.BodyA];
            CrowdNavigationState bodyANavigation = NavigationStates[pair.BodyA];
            CrowdMotionIntent bodyAIntent = MotionIntents[pair.BodyA];
            CrowdMotionEvidence bodyAEvidence = MotionEvidence[pair.BodyA];
            CrowdBodyStepState bodyAStep = StepStates[pair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = Bodies[pair.BodyB];
            CrowdNavigationState bodyBNavigation = NavigationStates[pair.BodyB];
            CrowdMotionIntent bodyBIntent = MotionIntents[pair.BodyB];
            CrowdMotionEvidence bodyBEvidence = MotionEvidence[pair.BodyB];
            CrowdBodyStepState bodyBStep = StepStates[pair.BodyB];

            if (pair.WasActivated != 0)
            {
                statistics.ActiveConstraintCount++;
                if (pair.ContactMode == ContactConstraintMode.Predictive)
                    statistics.PredictiveActivatedCount++;
            }

            float3 delta = bodyAStep.SolvedPosition - bodyBStep.SolvedPosition;
            delta.y = 0;
            float penetration = math.max(0f, bodyASnapshot.Radius + bodyBSnapshot.Radius - math.length(delta));
            if (penetration <= 0f)
                continue;

            statistics.PenetratingPairCount++;
            statistics.MaxPenetration = math.max(statistics.MaxPenetration, penetration);
            penetrationSum += penetration;
        }
    }

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
        float violationSum = 0f;
        float radialPenetrationSum = 0f;
        float maxViolation = 0f;
        float maxRadialPenetration = 0f;
        int violatingCount = 0;
        int penetratingCount = 0;
        int activeCount = 0;
        int predictiveActivatedCount = 0;

        for (int i = 0; i < TimestepContactPairs.Length; i++)
        {
            ContactConstraint pair = TimestepContactPairs[i];
            CrowdBodySnapshot bodyASnapshot = Bodies[pair.BodyA];
            CrowdNavigationState bodyANavigation = NavigationStates[pair.BodyA];
            CrowdMotionIntent bodyAIntent = MotionIntents[pair.BodyA];
            CrowdMotionEvidence bodyAEvidence = MotionEvidence[pair.BodyA];
            CrowdBodyStepState bodyAStep = StepStates[pair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = Bodies[pair.BodyB];
            CrowdNavigationState bodyBNavigation = NavigationStates[pair.BodyB];
            CrowdMotionIntent bodyBIntent = MotionIntents[pair.BodyB];
            CrowdMotionEvidence bodyBEvidence = MotionEvidence[pair.BodyB];
            CrowdBodyStepState bodyBStep = StepStates[pair.BodyB];
            float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;
            float3 currentDelta = bodyAStep.SolvedPosition - bodyBStep.SolvedPosition;
            currentDelta.y = 0;

            float constraintValue = CalculateConstraintValue(pair, currentDelta, radiusSum);

            float violation = math.max(0f, -constraintValue);
            if (violation > 0f)
            {
                violationSum += violation;
                maxViolation = math.max(maxViolation, violation);
                violatingCount++;
            }

            float radialPenetration = math.max(0f, radiusSum - math.length(currentDelta));
            if (radialPenetration > 0f)
            {
                radialPenetrationSum += radialPenetration;
                maxRadialPenetration = math.max(maxRadialPenetration, radialPenetration);
                penetratingCount++;
            }

            if (pair.Lambda <= 0.0000001f)
                continue;

            activeCount++;
            if (pair.ContactMode == ContactConstraintMode.Predictive)
                predictiveActivatedCount++;
        }

        IterationDiagnostics.Add(new Stage3ContactIterationDiagnostic
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

    private void MeasureContactResidual(
        out float maxViolation,
        out float averageViolation)
    {
        float violationSum = 0f;
        maxViolation = 0f;
        int violatingCount = 0;
        for (int i = 0; i < TimestepContactPairs.Length; i++)
        {
            ContactConstraint pair = TimestepContactPairs[i];
            CrowdBodySnapshot bodyASnapshot = Bodies[pair.BodyA];
            CrowdNavigationState bodyANavigation = NavigationStates[pair.BodyA];
            CrowdMotionIntent bodyAIntent = MotionIntents[pair.BodyA];
            CrowdMotionEvidence bodyAEvidence = MotionEvidence[pair.BodyA];
            CrowdBodyStepState bodyAStep = StepStates[pair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = Bodies[pair.BodyB];
            CrowdNavigationState bodyBNavigation = NavigationStates[pair.BodyB];
            CrowdMotionIntent bodyBIntent = MotionIntents[pair.BodyB];
            CrowdMotionEvidence bodyBEvidence = MotionEvidence[pair.BodyB];
            CrowdBodyStepState bodyBStep = StepStates[pair.BodyB];
            float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;
            float3 currentDelta = bodyAStep.SolvedPosition - bodyBStep.SolvedPosition;
            currentDelta.y = 0;
            float violation = math.max(
                0f,
                -CalculateConstraintValue(pair, currentDelta, radiusSum));
            if (violation <= 0f)
                continue;

            violationSum += violation;
            maxViolation = math.max(maxViolation, violation);
            violatingCount++;
        }

        averageViolation = violatingCount > 0
            ? violationSum / violatingCount
            : 0f;
    }

    private static float CalculateConstraintValue(
        ContactConstraint pair,
        float3 currentDelta,
        float radiusSum)
    {
        if (pair.ContactMode != ContactConstraintMode.Predictive)
            return math.length(currentDelta) - radiusSum;

        return math.dot(currentDelta, pair.PredictiveNormal) - radiusSum;
    }

    private void CaptureSelectedBodyAndPairs(int substepIndex)
    {
        int selectedBodyIndex = FindSelectedBodyIndex();
        if (selectedBodyIndex < 0)
        {
            SelectedBodyDiagnostic.Value = default;
            return;
        }

        CrowdBodySnapshot selectedSnapshot = Bodies[selectedBodyIndex];
        CrowdNavigationState selectedNavigation = NavigationStates[selectedBodyIndex];
        CrowdMotionIntent selectedIntent = MotionIntents[selectedBodyIndex];
        CrowdMotionEvidence selectedEvidence = MotionEvidence[selectedBodyIndex];
        CrowdBodyStepState selectedStep = StepStates[selectedBodyIndex];
        var selectedDiagnostic = new Stage3SelectedBodyDiagnostic
        {
            IsValid = 1,
            SubstepIndex = substepIndex,
            Radius = selectedSnapshot.Radius,
            Skin = math.max(0f, PredictiveSkin),
            StartPosition = selectedStep.SubstepStartPosition,
            UnconstrainedPredictedPosition = selectedStep.UnconstrainedPosition,
            SolvedPosition = selectedStep.SolvedPosition,
            ContactCorrection = selectedStep.ContactCorrection,
            WallCorrection = selectedStep.WallCorrection,
            VelocityBeforeContact = selectedStep.VelocityBeforeContact,
            VelocityAfterContact = selectedStep.IntegratedVelocity,
            TimestepStartPosition = selectedEvidence.TrajectoryStart,
            TimestepPredictedPosition = selectedEvidence.BaselineEnd,
            TimestepEnvelopeMin = selectedEvidence.ContactEnvelopeMin,
            TimestepEnvelopeMax = selectedEvidence.ContactEnvelopeMax,
            TimestepEscaped = selectedEvidence.EnvelopeEscaped,
            TimestepContactCorrection = selectedEvidence.ContactCorrection,
            TimestepWallCorrection = selectedEvidence.WallCorrection
        };

        selectedDiagnostic.ShadowReferenceAvailable = 1;
        selectedDiagnostic.ShadowEscaped = selectedEvidence.EnvelopeEscaped;
        selectedDiagnostic.ShadowFatMin = selectedEvidence.InteractionEnvelopeMin;
        selectedDiagnostic.ShadowFatMax = selectedEvidence.InteractionEnvelopeMax;

        SelectedBodyDiagnostic.Value = selectedDiagnostic;

        for (int i = 0; i < TimestepContactPairs.Length; i++)
        {
            ContactConstraint pair = TimestepContactPairs[i];
            if (pair.BodyA != selectedBodyIndex && pair.BodyB != selectedBodyIndex)
                continue;

            CrowdBodySnapshot bodyASnapshot = Bodies[pair.BodyA];
            CrowdNavigationState bodyANavigation = NavigationStates[pair.BodyA];
            CrowdMotionIntent bodyAIntent = MotionIntents[pair.BodyA];
            CrowdMotionEvidence bodyAEvidence = MotionEvidence[pair.BodyA];
            CrowdBodyStepState bodyAStep = StepStates[pair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = Bodies[pair.BodyB];
            CrowdNavigationState bodyBNavigation = NavigationStates[pair.BodyB];
            CrowdMotionIntent bodyBIntent = MotionIntents[pair.BodyB];
            CrowdMotionEvidence bodyBEvidence = MotionEvidence[pair.BodyB];
            CrowdBodyStepState bodyBStep = StepStates[pair.BodyB];
            float3 r0 = bodyBEvidence.TrajectoryStart - bodyAEvidence.TrajectoryStart;
            float3 relativeDisplacement =
                (bodyBEvidence.BaselineEnd - bodyBEvidence.TrajectoryStart) -
                (bodyAEvidence.BaselineEnd - bodyAEvidence.TrajectoryStart);
            r0.y = 0;
            relativeDisplacement.y = 0;
            float relativeLengthSq = math.lengthsq(relativeDisplacement);
            float closestTime = relativeLengthSq > 0.0000001f
                ? math.clamp(-math.dot(r0, relativeDisplacement) / relativeLengthSq, 0f, 1f)
                : 0f;
            float minDistance = math.length(r0 + closestTime * relativeDisplacement);
            float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;
            float startDistanceSq = math.lengthsq(r0);
            float3 endDelta =
                bodyBEvidence.BaselineEnd - bodyAEvidence.BaselineEnd;
            endDelta.y = 0;
            bool potentialPredictive =
                startDistanceSq >= radiusSum * radiusSum &&
                math.lengthsq(endDelta) >= radiusSum * radiusSum &&
                minDistance <= radiusSum;

            Stage3ContactDiagnosticPairKind kind;
            if (potentialPredictive)
            {
                kind = EnablePredictiveContacts
                    ? Stage3ContactDiagnosticPairKind.Predictive
                    : Stage3ContactDiagnosticPairKind.PredictiveDisabled;
            }
            else
            {
                kind = Stage3ContactDiagnosticPairKind.Regular;
            }

            AddSelectedPairDiagnostic(
                pair,
                kind,
                closestTime,
                minDistance,
                radiusSum,
                pair.WasActivatedThisTimestep);
        }
    }

    private void AddSelectedPairDiagnostic(
        ContactConstraint pair,
        Stage3ContactDiagnosticPairKind kind,
        float closestTime,
        float minimumDistance,
        float radiusSum,
        byte wasActivated)
    {
        int selectedBodyIndex = FindSelectedBodyIndex();
        if (selectedBodyIndex < 0 ||
            (pair.BodyA != selectedBodyIndex && pair.BodyB != selectedBodyIndex))
            return;

        int otherBodyIndex = pair.BodyA == selectedBodyIndex ? pair.BodyB : pair.BodyA;
        CrowdBodySnapshot selectedSnapshot = Bodies[selectedBodyIndex];
        CrowdNavigationState selectedNavigation = NavigationStates[selectedBodyIndex];
        CrowdMotionIntent selectedIntent = MotionIntents[selectedBodyIndex];
        CrowdMotionEvidence selectedEvidence = MotionEvidence[selectedBodyIndex];
        CrowdBodyStepState selectedStep = StepStates[selectedBodyIndex];
        CrowdBodySnapshot otherSnapshot = Bodies[otherBodyIndex];
        CrowdNavigationState otherNavigation = NavigationStates[otherBodyIndex];
        CrowdMotionIntent otherIntent = MotionIntents[otherBodyIndex];
        CrowdMotionEvidence otherEvidence = MotionEvidence[otherBodyIndex];
        CrowdBodyStepState otherStep = StepStates[otherBodyIndex];
        float3 selectedClosest = math.lerp(
            selectedEvidence.TrajectoryStart,
            selectedEvidence.BaselineEnd,
            closestTime);
        float3 otherClosest = math.lerp(
            otherEvidence.TrajectoryStart,
            otherEvidence.BaselineEnd,
            closestTime);

        PairDiagnostics.Add(new Stage3ContactPairDiagnostic
        {
            OtherEntity = otherSnapshot.Entity,
            Kind = kind,
            WasActivated = wasActivated,
            WasAddedByFallback = pair.WasAddedByFallback,
            FirstActivatedSubstep = pair.FirstActivatedSubstep,
            ActivatedSubstepCount = pair.ActivatedSubstepCount,
            ClosestTime = closestTime,
            MinimumDistance = minimumDistance,
            RadiusSum = radiusSum,
            OtherRadius = otherSnapshot.Radius,
            OtherStartPosition = otherEvidence.TrajectoryStart,
            OtherPredictedPosition = otherEvidence.BaselineEnd,
            SelectedClosestPosition = selectedClosest,
            OtherClosestPosition = otherClosest
        });
    }

    private int FindSelectedBodyIndex()
    {
        if (DiagnosticSelectedEntity == Entity.Null)
            return -1;

        for (int i = 0; i < Bodies.Length; i++)
        {
            if (Bodies[i].Entity == DiagnosticSelectedEntity)
                return i;
        }

        return -1;
    }

    private static long TimestampToNanoseconds(long timestampDelta)
    {
        var ratio = ProfilerUnsafeUtility.TimestampToNanosecondsConversionRatio;
        return timestampDelta * ratio.Numerator / ratio.Denominator;
    }
}
}
#endif
