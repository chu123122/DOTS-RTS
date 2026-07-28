using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
internal static class ContactPipelineShared
{
    internal static void SortAndDeduplicateBodyPairs(NativeList<BodyPair> pairs)
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

    internal static void SortAndDeduplicateConstraints(
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

    internal static void CopyConstraintsToBodyPairs(
        NativeArray<ContactConstraint> source,
        NativeList<BodyPair> destination)
    {
        for (int i = 0; i < source.Length; i++)
        {
            ContactConstraint constraint = source[i];
            destination.Add(new BodyPair(constraint.BodyA, constraint.BodyB));
        }
    }

    internal static void AppendBodyPairsAsConstraints(
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

    internal static bool TryFindProxy(
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

    internal static bool AabbContains(
        float2 outerMin,
        float2 outerMax,
        float2 innerMin,
        float2 innerMax)
    {
        const float tolerance = 0.00001f;
        return math.all(innerMin >= outerMin - tolerance) &&
               math.all(innerMax <= outerMax + tolerance);
    }

    internal static bool AabbOverlaps(
        float2 minA,
        float2 maxA,
        float2 minB,
        float2 maxB)
    {
        return math.all(maxA >= minB) && math.all(maxB >= minA);
    }
}
}
