using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Pure value helpers for the persistent (P1P6) contact-classification path:
/// constraint assembly, statistics accumulation, active-gauge tracking, pair
/// trajectory queries and neighbour-pair deduplication. No job state, no
/// instance fields — all inputs arrive as parameters.
/// </summary>
internal static class PersistentContactMath
{
    /// <summary>
    /// Builds a deterministic (lower body index first) contact constraint from
    /// a cached predictive contact's stable normal and mode.
    /// </summary>
    internal static ContactConstraint BuildConstraintFromPersistentContact(
        int firstBodyIndex,
        int secondBodyIndex,
        PersistentPredictiveContact contact) =>
        new ContactConstraint
        {
            BodyA = math.min(firstBodyIndex, secondBodyIndex),
            BodyB = math.max(firstBodyIndex, secondBodyIndex),
            PredictiveNormal = contact.StableNormal,
            ContactMode = contact.ContactMode,
            FirstActivatedSubstep = -1
        };

    /// <summary>
    /// Accumulates one persistent contact's lifecycle into the solver-facing
    /// statistics counters (actual / predictive / approaching / dormant).
    /// </summary>
    internal static void AccumulateClassificationStatistics(
        PersistentPredictiveContact contact,
        ref PredictiveDiscContactStatistics statistics)
    {
        switch (contact.Lifecycle)
        {
            case PersistentContactLifecycle.Actual:
                statistics.ActualGeneratedPairCount++;
                break;
            case PersistentContactLifecycle.Predictive:
                statistics.PredictiveGeneratedPairCount++;
                statistics.PredictivePairCount++;
                statistics.PotentialPredictivePairCount++;
                break;
            case PersistentContactLifecycle.Approaching:
                statistics.PredictiveGeneratedPairCount++;
                break;
            case PersistentContactLifecycle.Dormant:
                statistics.TimestepContactSetDormantPairCount++;
                break;
        }
    }

    /// <summary>
    /// Tracks the current active-constraint count plus its running peak.
    /// </summary>
    internal static void UpdateActiveConstraintGauges(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int currentActiveConstraintCount)
    {
        incrementalStatistics.CurrentActiveConstraintCount =
            math.max(0, currentActiveConstraintCount);
        incrementalStatistics.PeakActiveConstraintCount = math.max(
            incrementalStatistics.PeakActiveConstraintCount,
            incrementalStatistics.CurrentActiveConstraintCount);
    }

    /// <summary>
    /// Recomputes the per-lifecycle current-contact gauges (actual / predictive /
    /// approaching / dormant) from the predictive-contact scratch and updates
    /// the active-constraint gauge. The dormant/approaching/predictive/actual
    /// counts mirror AccumulateClassificationStatistics but reflect the current
    /// view rather than the cumulative generation counters.
    /// </summary>
    internal static void RefreshCurrentContactStateGauges(
        NativeList<PersistentPredictiveContact> predictiveContactScratch,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int currentActiveConstraintCount)
    {
        incrementalStatistics.CurrentSweptContactCount =
            predictiveContactScratch.Length;
        incrementalStatistics.CurrentDormantPairCount = 0;
        incrementalStatistics.CurrentApproachingPairCount = 0;
        incrementalStatistics.CurrentPredictivePairCount = 0;
        incrementalStatistics.CurrentActualPairCount = 0;

        for (int contactIndex = 0;
             contactIndex < predictiveContactScratch.Length;
             contactIndex++)
        {
            switch (predictiveContactScratch[contactIndex].Lifecycle)
            {
                case PersistentContactLifecycle.Dormant:
                    incrementalStatistics.CurrentDormantPairCount++;
                    break;
                case PersistentContactLifecycle.Approaching:
                    incrementalStatistics.CurrentApproachingPairCount++;
                    break;
                case PersistentContactLifecycle.Predictive:
                    incrementalStatistics.CurrentPredictivePairCount++;
                    break;
                case PersistentContactLifecycle.Actual:
                    incrementalStatistics.CurrentActualPairCount++;
                    break;
            }
        }

        UpdateActiveConstraintGauges(
            ref incrementalStatistics,
            currentActiveConstraintCount);
    }

    /// <summary>
    /// Closest-time-of-approach parameter t∈[0,1] for two swept-disc bodies,
    /// projected onto the xz-plane. Returns 0 when the relative displacement is
    /// negligible (parallel / stationary).
    /// </summary>
    internal static float CalculatePairClosestTime(
        CrowdMotionEvidence bodyAEvidence,
        CrowdMotionEvidence bodyBEvidence)
    {
        float3 relativeStart =
            bodyBEvidence.TrajectoryStart - bodyAEvidence.TrajectoryStart;
        float3 relativeDisplacement =
            (bodyBEvidence.BaselineEnd - bodyBEvidence.TrajectoryStart) -
            (bodyAEvidence.BaselineEnd - bodyAEvidence.TrajectoryStart);
        relativeStart.y = 0f;
        relativeDisplacement.y = 0f;
        float relativeLengthSq = math.lengthsq(relativeDisplacement);
        return relativeLengthSq > 0.0000001f
            ? math.clamp(
                -math.dot(relativeStart, relativeDisplacement) / relativeLengthSq,
                0f,
                1f)
            : 0f;
    }

    /// <summary>
    /// Whether two bodies move relative to each other over the timestep on the
    /// xz-plane (displacement above epsilon). Used to skip stationary pair
    /// closest-time evaluation.
    /// </summary>
    internal static bool HasRelativeTimestepTrajectory(
        CrowdMotionEvidence bodyAEvidence,
        CrowdMotionEvidence bodyBEvidence)
    {
        float3 relativeDisplacement =
            (bodyBEvidence.BaselineEnd - bodyBEvidence.TrajectoryStart) -
            (bodyAEvidence.BaselineEnd - bodyAEvidence.TrajectoryStart);
        relativeDisplacement.y = 0f;
        return math.lengthsq(relativeDisplacement) > 0.0000001f;
    }

    /// <summary>
    /// In-place sort + key dedup of the persistent neighbour-pair list. Pairs
    /// are ordered by a stable key so topology diffs are deterministic.
    /// </summary>
    internal static void SortAndDeduplicatePersistentNeighborPairs(
        NativeList<PersistentNeighborPair> pairs)
    {
        if (pairs.Length <= 1)
            return;

        pairs.AsArray().Sort(new PersistentNeighborPairComparer());
        int writeIndex = 1;
        PersistentNeighborPair previous = pairs[0];
        for (int readIndex = 1; readIndex < pairs.Length; readIndex++)
        {
            PersistentNeighborPair current = pairs[readIndex];
            if (current.Key.Equals(previous.Key))
                continue;
            pairs[writeIndex++] = current;
            previous = current;
        }
        pairs.ResizeUninitialized(writeIndex);
    }

    /// <summary>
    /// Tight swept bounds for a body's incremental proxy: the path AABB
    /// (trajectory start/end plus solved/unconstrained positions) inflated by
    /// the contact skin, double timestep margin, and half the soft-avoidance
    /// shell — whichever is larger.
    /// </summary>
    internal static void CalculateIncrementalTightSweptBounds(
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        float predictiveSkin,
        float timestepContactMargin,
        float softAvoidanceShell,
        float rvoTimeHorizon,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        out float2 tightMin,
        out float2 tightMax)
    {
        float contactPadding = math.max(0f, predictiveSkin) +
                               math.max(0f, timestepContactMargin) * 2f;
        float avoidancePadding = math.max(0f, softAvoidanceShell) * 0.5f;
        float extent = math.max(0f, stateSnapshot.Radius) +
                       math.max(contactPadding, avoidancePadding);
        CalculateNeighborPathBounds(
            stateEvidence, stateStep, softSolverMode, softAvoidanceShell, rvoTimeHorizon,
            out float2 pathMin, out float2 pathMax);
        tightMin = pathMin - extent;
        tightMax = pathMax + extent;
    }

    /// <summary>
    /// Validation bounds for a body's incremental proxy: the path AABB inflated
    /// by the current contact/avoidance footprint only. The stored interaction
    /// envelope already carries the retained-contact budget, so validation must
    /// not re-apply it or every unchanged proxy would look escaped.
    /// </summary>
    internal static void CalculateIncrementalValidationBounds(
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        float predictiveSkin,
        float timestepContactMargin,
        float softAvoidanceShell,
        out float2 validationMin,
        out float2 validationMax)
    {
        float contactPadding = math.max(0f, predictiveSkin) +
                               math.max(0f, timestepContactMargin);
        float avoidancePadding = math.max(0f, softAvoidanceShell) * 0.5f;
        float extent = math.max(0f, stateSnapshot.Radius) +
                       math.max(contactPadding, avoidancePadding);
        // Validation bounds intentionally do not extend the RVO horizon: the
        // interaction envelope is fixed at build time. Pass a non-RVO mode so
        // CalculateNeighborPathBounds skips the horizon projection.
        CalculateNeighborPathBounds(
            stateEvidence, stateStep, SoftAvoidanceVelocitySolverMode.SurfaceVelocityBuffer, 0f, 0f,
            out float2 pathMin, out float2 pathMax);
        validationMin = pathMin - extent;
        validationMax = pathMax + extent;
    }

    private static void CalculateNeighborPathBounds(
        CrowdMotionEvidence evidence,
        CrowdBodyStepState step,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float softAvoidanceShell,
        float rvoTimeHorizon,
        out float2 pathMin,
        out float2 pathMax)
    {
        pathMin = math.min(
            evidence.TrajectoryStart.xz,
            math.min(
                evidence.BaselineEnd.xz,
                math.min(step.UnconstrainedPosition.xz, step.SolvedPosition.xz)));
        pathMax = math.max(
            evidence.TrajectoryStart.xz,
            math.max(
                evidence.BaselineEnd.xz,
                math.max(step.UnconstrainedPosition.xz, step.SolvedPosition.xz)));
        if (softSolverMode !=
                SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle ||
            softAvoidanceShell <= 0f)
            return;

        float2 horizonEnd = step.SolvedPosition.xz +
                            step.BaseVelocity.xz * math.max(0f, rvoTimeHorizon);
        pathMin = math.min(pathMin, horizonEnd);
        pathMax = math.max(pathMax, horizonEnd);
    }
}
}
