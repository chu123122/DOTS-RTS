using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

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
                Definition = new ContactConstraintDefinition
                {
                    BodyA = pair.BodyA,
                    BodyB = pair.BodyB
                }
            });
        }
    }

    internal static int FindConstraintIndex(
        NativeList<ContactConstraint> constraints,
        int bodyA,
        int bodyB)
    {
        int low = 0;
        int high = constraints.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            ContactConstraint candidate = constraints[middle];
            if (candidate.BodyA == bodyA && candidate.BodyB == bodyB)
                return middle;
            if (candidate.BodyA < bodyA ||
                (candidate.BodyA == bodyA && candidate.BodyB < bodyB))
                low = middle + 1;
            else
                high = middle - 1;
        }
        return -1;
    }

    internal static int FindBodyPairIndex(
        NativeList<BodyPair> pairs,
        int bodyA,
        int bodyB)
    {
        int low = 0;
        int high = pairs.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            BodyPair candidate = pairs[middle];
            if (candidate.BodyA == bodyA && candidate.BodyB == bodyB)
                return middle;
            if (candidate.BodyA < bodyA ||
                (candidate.BodyA == bodyA && candidate.BodyB < bodyB))
                low = middle + 1;
            else
                high = middle - 1;
        }
        return -1;
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

internal static class ContactEnvelopeValidationKernel
{
    internal static bool ValidateSolverCorrections(
        int substepIndex,
        float predictiveSkin,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdMotionEvidence> motionEvidence,
        NativeArray<CrowdSolverBodyState> stepStates,
        NativeArray<byte> dirtyFlagsByBody,
        NativeList<IncrementalDirtyBody> dirtyBodies,
        NativeReference<InteractionCertificate> interactionCertificate,
        NativeList<InteractionCertificateViolation> certificateViolations,
        NativeList<int> correctedBodyIndices,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        bool allInside = true;
        IncrementalDirtyBodyStore.Clear(dirtyFlagsByBody, dirtyBodies);
        float skin = math.max(0f, predictiveSkin);
        for (int correctedIndex = 0;
             correctedIndex < correctedBodyIndices.Length;
             correctedIndex++)
        {
            int bodyIndex = correctedBodyIndices[correctedIndex];
            CrowdBodySnapshot snapshot = bodies[bodyIndex];
            CrowdMotionEvidence evidence = motionEvidence[bodyIndex];
            CrowdSolverBodyState step = stepStates[bodyIndex];
            float extent = math.max(0f, snapshot.Radius) + skin;
            float2 currentMin = step.SolvedPosition.xz - extent;
            float2 currentMax = step.SolvedPosition.xz + extent;
            if (ContactPipelineShared.AabbContains(
                    evidence.ContactEnvelopeMin,
                    evidence.ContactEnvelopeMax,
                    currentMin,
                    currentMax))
                continue;

            allInside = false;
            RevokeCertificate(
                bodyIndex,
                substepIndex,
                InteractionCertificateViolationReason
                    .SolverCorrectionEnvelopeEscape,
                currentMin,
                currentMax,
                interactionCertificate,
                certificateViolations);
            IncrementalDirtyBodyStore.SetFlags(
                bodyIndex,
                IncrementalBodyDirtyFlags.Motion |
                IncrementalBodyDirtyFlags.CorrectedEscape,
                dirtyFlagsByBody,
                dirtyBodies);
            if (evidence.EnvelopeEscaped != 0)
                continue;

            evidence.EnvelopeEscaped = 1;
            statistics.TimestepContactSetEscapeBodyCount++;
            if (statistics.TimestepContactSetFirstEscapeSubstep < 0)
                statistics.TimestepContactSetFirstEscapeSubstep =
                    substepIndex;
            motionEvidence[bodyIndex] = evidence;
        }
        incrementalStatistics.CorrectedEscapeBodyCount += dirtyBodies.Length;
        return allInside;
    }

    private static void RevokeCertificate(
        int bodyIndex,
        int substepIndex,
        InteractionCertificateViolationReason reason,
        float2 observedMin,
        float2 observedMax,
        NativeReference<InteractionCertificate> interactionCertificate,
        NativeList<InteractionCertificateViolation> certificateViolations)
    {
        if (interactionCertificate.IsCreated)
        {
            InteractionCertificate certificate = interactionCertificate.Value;
            certificate.Flags &= ~InteractionCertificationFlags.Issued;
            interactionCertificate.Value = certificate;
        }

        if (!certificateViolations.IsCreated)
            return;
        certificateViolations.Add(new InteractionCertificateViolation
        {
            BodyIndex = bodyIndex,
            FirstInvalidSubstep = (ushort)math.max(0, substepIndex),
            Reason = reason,
            ObservedMin = observedMin,
            ObservedMax = observedMax
        });
    }
}

internal static class CorrectedBodyTrackingKernel
{
    internal static void Reset(
        NativeArray<byte> correctedBodyFlags,
        NativeList<int> correctedBodyIndices)
    {
        for (int i = 0; i < correctedBodyIndices.Length; i++)
            correctedBodyFlags[correctedBodyIndices[i]] = 0;
        correctedBodyIndices.Clear();
    }
}

internal static class ContactConstraintStateKernel
{
    internal static void ResetForSubstep(
        NativeList<ContactConstraint> timestepContactPairs)
    {
        for (int pairIndex = 0;
             pairIndex < timestepContactPairs.Length;
             pairIndex++)
        {
            ContactConstraint pair = timestepContactPairs[pairIndex];
            pair.Lambda = 0f;
            pair.WasActivated = 0;
            timestepContactPairs[pairIndex] = pair;
        }
    }
}
}
