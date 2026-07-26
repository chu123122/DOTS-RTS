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
}
}
