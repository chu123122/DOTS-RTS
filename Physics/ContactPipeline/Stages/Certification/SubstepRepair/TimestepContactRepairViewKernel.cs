using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal static class TimestepContactRepairViewKernel
{
    internal static void MergeEscapedTimestepContactView(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        ContactPipelineConfiguration configuration,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdMotionEvidence> motionEvidence,
        NativeArray<CrowdSolverBodyState> stepStates,
        NativeList<ContactConstraint> pairs,
        NativeList<ContactConstraint> timestepContactPairs,
        NativeList<ContactConstraint> previousTimestepContactPairs,
        NativeList<BodyPair> softAvoidancePairs,
        NativeArray<byte> dirtyFlagsByBody,
        NativeList<PersistentPredictiveContact> persistentContacts
#if RTS_CONTACT_DIAGNOSTICS
        , NativeList<BodyPair> oracleContactPairs
#endif
    )
    {
        timestepContactPairs.Clear();
        int previousIndex = 0;
        int pairIndex = 0;
        while (previousIndex < previousTimestepContactPairs.Length &&
               pairIndex < pairs.Length)
        {
            ContactConstraint previousContact =
                previousTimestepContactPairs[previousIndex];
            ContactConstraint newContact = pairs[pairIndex];
            int comparison = Compare(previousContact, newContact);
            if (comparison < 0)
            {
                if (!IsDirty(previousContact, dirtyFlagsByBody))
                    timestepContactPairs.Add(previousContact);
                previousIndex++;
            }
            else if (comparison > 0)
            {
                AppendNewContact(
                    newContact,
                    ref statistics,
                    timestepContactPairs);
                pairIndex++;
            }
            else
            {
                CopyTimestepRuntime(previousContact, ref newContact);
                timestepContactPairs.Add(newContact);
                previousIndex++;
                pairIndex++;
            }
        }
        while (previousIndex < previousTimestepContactPairs.Length)
        {
            ContactConstraint previousContact =
                previousTimestepContactPairs[previousIndex++];
            if (!IsDirty(previousContact, dirtyFlagsByBody))
                timestepContactPairs.Add(previousContact);
        }
        while (pairIndex < pairs.Length)
        {
            AppendNewContact(
                pairs[pairIndex++],
                ref statistics,
                timestepContactPairs);
        }

        PersistentContactMath.RefreshCurrentContactStateGauges(
            persistentContacts,
            ref incrementalStatistics,
            timestepContactPairs.Length);
        statistics.TimestepContactSetBuildCount++;
        statistics.TimestepContactSetClassificationPassCount++;
        statistics.TimestepContactSetUniquePairCount =
            timestepContactPairs.Length;
        statistics.TimestepContactSetDormantPairCount =
            incrementalStatistics.CurrentDormantPairCount;
        SoftAvoidanceOracleKernel.ValidateSoftAvoidancePairViewAgainstQuadraticOracle(
            ref incrementalStatistics,
            configuration,
            bodies,
            motionEvidence,
            stepStates,
            softAvoidancePairs);
#if RTS_CONTACT_DIAGNOSTICS
        ContactOracleKernel.ValidateIncrementalContactSetAgainstQuadraticOracle(
            ref incrementalStatistics,
            configuration,
            bodies,
            motionEvidence,
            timestepContactPairs,
            oracleContactPairs);
#endif
    }

    private static int Compare(
        ContactConstraint left,
        ContactConstraint right)
    {
        int bodyA = left.BodyA.CompareTo(right.BodyA);
        return bodyA != 0
            ? bodyA
            : left.BodyB.CompareTo(right.BodyB);
    }

    private static bool IsDirty(
        ContactConstraint contact,
        NativeArray<byte> dirtyFlagsByBody)
    {
        return IncrementalDirtyBodyStore.IsDirtyBodyIndex(
                   dirtyFlagsByBody,
                   contact.BodyA) ||
               IncrementalDirtyBodyStore.IsDirtyBodyIndex(
                   dirtyFlagsByBody,
                   contact.BodyB);
    }

    private static void AppendNewContact(
        ContactConstraint contact,
        ref PredictiveDiscContactStatistics statistics,
        NativeList<ContactConstraint> destination)
    {
        contact.WasAddedByFallback = 1;
        statistics.TimestepContactSetFallbackAddedPairCount++;
        destination.Add(contact);
    }

    private static void CopyTimestepRuntime(
        ContactConstraint previous,
        ref ContactConstraint current)
    {
        current.WasActivatedThisTimestep =
            previous.WasActivatedThisTimestep;
        current.WasCorrectedThisTimestep =
            previous.WasCorrectedThisTimestep;
        current.FirstActivatedSubstep =
            previous.FirstActivatedSubstep;
        current.ActivatedSubstepCount =
            previous.ActivatedSubstepCount;
        current.WasAddedByFallback =
            previous.WasAddedByFallback;
    }

}
}
