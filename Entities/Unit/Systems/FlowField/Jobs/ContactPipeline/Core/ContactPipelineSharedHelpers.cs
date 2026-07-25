using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Shared frame-local utilities used by the authoritative contact pipeline.
/// These helpers have no ownership of persistent topology or legacy Fat AABB state.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private void PrepareCurrentBodyLookup()
    {
        CurrentBodyIndexByEntity.Clear();
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
            CurrentBodyIndexByEntity.TryAdd(Bodies[bodyIndex].Entity, bodyIndex);
    }

    private bool TryFindCurrentBodyIndex(Entity entity, out int bodyIndex)
    {
        return CurrentBodyIndexByEntity.TryGetValue(entity, out bodyIndex) &&
               bodyIndex >= 0 && bodyIndex < Bodies.Length;
    }

    private void CalculateNeighborPathBounds(
        CrowdMotionEvidence evidence,
        CrowdBodyStepState step,
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
        if (SoftAvoidanceVelocitySolver !=
                SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle ||
            SoftAvoidanceShell <= 0f || SoftAvoidanceResponseRate <= 0f)
            return;

        float2 horizonEnd = step.SolvedPosition.xz +
                            step.BaseVelocity.xz * math.max(0f, RvoTimeHorizon);
        pathMin = math.min(pathMin, horizonEnd);
        pathMax = math.max(pathMax, horizonEnd);
    }

    private void ResetCorrectedBodyTracking()
    {
        for (int i = 0; i < CorrectedBodyIndices.Length; i++)
            CorrectedBodyFlags[CorrectedBodyIndices[i]] = 0;
        CorrectedBodyIndices.Clear();
    }

    private void MarkCorrectedBody(int bodyIndex)
    {
        if (CorrectedBodyFlags[bodyIndex] != 0)
            return;
        CorrectedBodyFlags[bodyIndex] = 1;
        CorrectedBodyIndices.Add(bodyIndex);
    }

    private static void SortAndDeduplicateBodyPairs(NativeList<BodyPair> pairs)
    {
        if (pairs.Length <= 1)
            return;
        pairs.AsArray().Sort(new BodyPairComparer());
        int writeIndex = 1;
        BodyPair previous = pairs[0];
        for (int readIndex = 1; readIndex < pairs.Length; readIndex++)
        {
            BodyPair current = pairs[readIndex];
            if (current.BodyA == previous.BodyA && current.BodyB == previous.BodyB)
                continue;
            pairs[writeIndex++] = current;
            previous = current;
        }
        pairs.ResizeUninitialized(writeIndex);
    }

    private static void SortAndDeduplicateConstraints(
        NativeList<ContactConstraint> constraints)
    {
        if (constraints.Length <= 1)
            return;
        constraints.AsArray().Sort(new ContactConstraintComparer());
        int writeIndex = 1;
        ContactConstraint previous = constraints[0];
        for (int readIndex = 1; readIndex < constraints.Length; readIndex++)
        {
            ContactConstraint current = constraints[readIndex];
            if (current.BodyA == previous.BodyA && current.BodyB == previous.BodyB)
                continue;
            constraints[writeIndex++] = current;
            previous = current;
        }
        constraints.ResizeUninitialized(writeIndex);
    }

    private static void CopyConstraintsToBodyPairs(
        NativeArray<ContactConstraint> source,
        NativeList<BodyPair> destination)
    {
        for (int i = 0; i < source.Length; i++)
        {
            ContactConstraint constraint = source[i];
            destination.Add(new BodyPair(constraint.BodyA, constraint.BodyB));
        }
    }

    private static void AppendBodyPairsAsConstraints(
        NativeArray<BodyPair> source,
        NativeList<ContactConstraint> destination)
    {
        for (int i = 0; i < source.Length; i++)
        {
            BodyPair pair = source[i];
            destination.Add(new ContactConstraint
            {
                BodyA = pair.BodyA,
                BodyB = pair.BodyB
            });
        }
    }

    private static bool TryFindProxy(
        NativeList<ShadowFatBodyProxy> proxies,
        Entity entity,
        out ShadowFatBodyProxy proxy)
    {
        int low = 0;
        int high = proxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            ShadowFatBodyProxy candidate = proxies[middle];
            int comparison = ShadowEntityOrdering.Compare(candidate.Entity, entity);
            if (comparison == 0)
            {
                proxy = candidate;
                return proxy.IsValid != 0;
            }
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        proxy = default;
        return false;
    }

    private static bool AabbContains(
        float2 outerMin,
        float2 outerMax,
        float2 innerMin,
        float2 innerMax)
    {
        const float tolerance = 0.00001f;
        return math.all(innerMin >= outerMin - tolerance) &&
               math.all(innerMax <= outerMax + tolerance);
    }

    private static bool AabbOverlaps(
        float2 minA,
        float2 maxA,
        float2 minB,
        float2 maxB)
    {
        return math.all(maxA >= minB) && math.all(maxB >= minA);
    }
}
}
