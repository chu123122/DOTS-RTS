using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct SolveXpbdUnitContactsJob
{
    private void AccumulateConstraintStatistics(
        ref PredictiveDiscContactStatistics statistics,
        ref float penetrationSum)
    {
        for (int i = 0; i < TimestepContactPairs.Length; i++)
        {
            UnitCollisionPair pair = TimestepContactPairs[i];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];

            if (pair.WasActivated != 0)
            {
                statistics.ActiveConstraintCount++;
                if (pair.ContactMode == UnitContactMode.Predictive)
                    statistics.PredictiveActivatedCount++;
            }

            float3 delta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            delta.y = 0;
            float penetration = math.max(0f, bodyA.Radius + bodyB.Radius - math.length(delta));
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
            UnitCollisionPair pair = TimestepContactPairs[i];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            float radiusSum = bodyA.Radius + bodyB.Radius;
            float3 currentDelta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            currentDelta.y = 0;

            float constraintValue = CalculateConstraintValue(pair, bodyA, bodyB, currentDelta, radiusSum);

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
            if (pair.ContactMode == UnitContactMode.Predictive)
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
            UnitCollisionPair pair = TimestepContactPairs[i];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            float radiusSum = bodyA.Radius + bodyB.Radius;
            float3 currentDelta = bodyA.PredictedPosition - bodyB.PredictedPosition;
            currentDelta.y = 0;
            float violation = math.max(
                0f,
                -CalculateConstraintValue(pair, bodyA, bodyB, currentDelta, radiusSum));
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
        UnitCollisionPair pair,
        FlowMovementFrameState bodyA,
        FlowMovementFrameState bodyB,
        float3 currentDelta,
        float radiusSum)
    {
        if (pair.ContactMode != UnitContactMode.Predictive)
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

        FlowMovementFrameState selected = States[selectedBodyIndex];
        var selectedDiagnostic = new Stage3SelectedBodyDiagnostic
        {
            IsValid = 1,
            SubstepIndex = substepIndex,
            Radius = selected.Radius,
            Skin = math.max(0f, PredictiveSkin),
            StartPosition = selected.StartPosition,
            UnconstrainedPredictedPosition = selected.UnconstrainedPredictedPosition,
            SolvedPosition = selected.PredictedPosition,
            ContactCorrection = selected.ContactPositionCorrection,
            WallCorrection = selected.WallPositionCorrection,
            VelocityBeforeContact = selected.VelocityBeforeContact,
            VelocityAfterContact = selected.IntegratedVelocity,
            TimestepStartPosition = selected.TimestepStartPosition,
            TimestepPredictedPosition = selected.TimestepPredictedPosition,
            TimestepEnvelopeMin = selected.TimestepEnvelopeMin,
            TimestepEnvelopeMax = selected.TimestepEnvelopeMax,
            TimestepEscaped = selected.TimestepEscaped,
            TimestepContactCorrection = selected.TimestepContactCorrection,
            TimestepWallCorrection = selected.TimestepWallCorrection
        };

        if (EnablePersistentContactCache &&
            TryFindPersistentProxy(
                selected.Entity,
                out PersistentSweptProxy proxy) &&
            proxy.IsValid != 0)
        {
            float coreExtent = math.max(0f, selected.Radius) +
                               math.max(0f, PredictiveSkin);
            float2 finalMin = selected.PredictedPosition.xz - coreExtent;
            float2 finalMax = selected.PredictedPosition.xz + coreExtent;
            selectedDiagnostic.ShadowReferenceAvailable = 1;
            selectedDiagnostic.ShadowEscaped =
                (byte)(AabbContains(proxy.GuardMin, proxy.GuardMax, finalMin, finalMax) ? 0 : 1);
            selectedDiagnostic.ShadowFatMin = proxy.GuardMin;
            selectedDiagnostic.ShadowFatMax = proxy.GuardMax;
        }

        SelectedBodyDiagnostic.Value = selectedDiagnostic;

        for (int i = 0; i < TimestepContactPairs.Length; i++)
        {
            UnitCollisionPair pair = TimestepContactPairs[i];
            if (pair.BodyA != selectedBodyIndex && pair.BodyB != selectedBodyIndex)
                continue;

            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];
            float3 r0 = bodyB.TimestepStartPosition - bodyA.TimestepStartPosition;
            float3 relativeDisplacement =
                (bodyB.TimestepPredictedPosition - bodyB.TimestepStartPosition) -
                (bodyA.TimestepPredictedPosition - bodyA.TimestepStartPosition);
            r0.y = 0;
            relativeDisplacement.y = 0;
            float relativeLengthSq = math.lengthsq(relativeDisplacement);
            float closestTime = relativeLengthSq > 0.0000001f
                ? math.clamp(-math.dot(r0, relativeDisplacement) / relativeLengthSq, 0f, 1f)
                : 0f;
            float minDistance = math.length(r0 + closestTime * relativeDisplacement);
            float radiusSum = bodyA.Radius + bodyB.Radius;
            float startDistanceSq = math.lengthsq(r0);
            float3 endDelta =
                bodyB.TimestepPredictedPosition - bodyA.TimestepPredictedPosition;
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
        UnitCollisionPair pair,
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
        FlowMovementFrameState selected = States[selectedBodyIndex];
        FlowMovementFrameState other = States[otherBodyIndex];
        float3 selectedClosest = math.lerp(
            selected.TimestepStartPosition,
            selected.TimestepPredictedPosition,
            closestTime);
        float3 otherClosest = math.lerp(
            other.TimestepStartPosition,
            other.TimestepPredictedPosition,
            closestTime);

        PairDiagnostics.Add(new Stage3ContactPairDiagnostic
        {
            OtherEntity = other.Entity,
            Kind = kind,
            WasActivated = wasActivated,
            WasAddedByFallback = pair.WasAddedByFallback,
            FirstActivatedSubstep = pair.FirstActivatedSubstep,
            ActivatedSubstepCount = pair.ActivatedSubstepCount,
            ClosestTime = closestTime,
            MinimumDistance = minimumDistance,
            RadiusSum = radiusSum,
            OtherRadius = other.Radius,
            OtherStartPosition = other.TimestepStartPosition,
            OtherPredictedPosition = other.TimestepPredictedPosition,
            SelectedClosestPosition = selectedClosest,
            OtherClosestPosition = otherClosest
        });
    }

    private int FindSelectedBodyIndex()
    {
        if (DiagnosticSelectedEntity == Entity.Null)
            return -1;

        for (int i = 0; i < States.Length; i++)
        {
            if (States[i].Entity == DiagnosticSelectedEntity)
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
