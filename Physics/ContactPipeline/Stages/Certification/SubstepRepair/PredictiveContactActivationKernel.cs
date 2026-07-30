using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal static class PredictiveContactActivationKernel
{
    private static bool TryFindCurrentBodyIndex(
        Entity entity,
        int bodyCount,
        NativeParallelHashMap<Entity, int> currentBodyIndexByEntity,
        out int bodyIndex) =>
        currentBodyIndexByEntity.TryGetValue(entity, out bodyIndex) &&
        (uint)bodyIndex < (uint)bodyCount;

    internal static void ActivateScheduledPredictiveContactsForSubstep(
        int substepIndex,
        int substepCount,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        ContactPipelineConfiguration configuration,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdMotionEvidence> motionEvidence,
        NativeArray<CrowdSolverBodyState> stepStates,
        NativeParallelHashMap<Entity, int> currentBodyIndexByEntity,
        NativeList<BodyPair> timestepInteractionPairs,
        NativeList<BodyPair> softAvoidancePairs,
        NativeList<ContactConstraint> timestepContactPairs,
        NativeList<PersistentNeighborPair> persistentNeighborPairs,
        NativeList<PersistentPredictiveContact> persistentContacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex,
        NativeList<PredictiveContactScheduleEntry> schedule,
        NativeList<PredictiveContactScheduleEntry> scheduleScratch,
        NativeReference<int> scheduleCursor,
        NativeReference<IncrementalContactCacheState> cacheState,
        NativeReference<InteractionCertificate> interactionCertificate,
        NativeList<InteractionCertificateViolation> certificateViolations)
    {
        if (schedule.Length == 0)
            return;

        long activationStart = ProfilerUnsafeUtility.Timestamp;
        scheduleScratch.Clear();
        for (int scheduleIndex = 0;
             scheduleIndex < schedule.Length;
             scheduleIndex++)
        {
            PredictiveContactScheduleEntry entry = schedule[scheduleIndex];
            if (entry.Substep > substepIndex)
            {
                scheduleScratch.Add(entry);
                continue;
            }

            incrementalStatistics.ScheduledWakeupCount++;
            if (!TryFindCurrentBodyIndex(
                    entry.Key.EntityA,
                    bodies.Length,
                    currentBodyIndexByEntity,
                    out int bodyA) ||
                !TryFindCurrentBodyIndex(
                    entry.Key.EntityB,
                    bodies.Length,
                    currentBodyIndexByEntity,
                    out int bodyB))
            {
                MarkPersistentContactExpired(
                    entry.Key,
                    configuration.EnablePersistentContactCache,
                    persistentContacts,
                    contactIndex,
                    cacheState);
                continue;
            }

            if (TryBuildCurrentScheduledPair(
                    bodyA,
                    bodyB,
                    configuration,
                    bodies,
                    motionEvidence,
                    stepStates,
                    out ContactConstraint pair))
            {
                if (ContactPipelineShared.FindConstraintIndex(
                        timestepContactPairs,
                        pair.BodyA,
                        pair.BodyB) < 0)
                {
                    InsertConstraintSorted(timestepContactPairs, pair);
                }
                UpdatePersistentContactAfterScheduledCheck(
                    entry.Key,
                    pair,
                    ushort.MaxValue,
                    configuration.EnablePersistentContactCache,
                    bodies,
                    stepStates,
                    persistentContacts,
                    contactIndex,
                    cacheState);
                continue;
            }
            else if (substepIndex + 1 < substepCount)
            {
                ushort nextSubstep = (ushort)(substepIndex + 1);
                scheduleScratch.Add(
                    new PredictiveContactScheduleEntry
                    {
                        Key = entry.Key,
                        Substep = nextSubstep
                    });
                UpdatePersistentContactNextCheck(
                    entry.Key,
                    nextSubstep,
                    configuration.EnablePersistentContactCache,
                    persistentContacts,
                    contactIndex,
                    cacheState);
            }
            else
            {
                MarkPersistentContactExpired(
                    entry.Key,
                    configuration.EnablePersistentContactCache,
                    persistentContacts,
                    contactIndex,
                    cacheState);
            }
        }

        schedule.Clear();
        schedule.AddRange(scheduleScratch.AsArray());
        scheduleCursor.Value = 0;
        PersistentContactMath.UpdateActiveConstraintGauges(
            ref incrementalStatistics,
            timestepContactPairs.Length);
        incrementalStatistics.ContactActivationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - activationStart);
        // Activation can add constraints or rewrite the dormant schedule after
        // the previous commit, so certify the final consumer-visible views.
        InteractionCertificateKernel.IssueCertificateForCommittedViews(
            incrementalStatistics,
            configuration,
            bodies,
            timestepInteractionPairs,
            softAvoidancePairs,
            timestepContactPairs,
            currentBodyIndexByEntity,
            persistentNeighborPairs,
            schedule,
            cacheState,
            interactionCertificate,
            certificateViolations,
            substepIndex);
    }

    private static void InsertConstraintSorted(
        NativeList<ContactConstraint> constraints,
        ContactConstraint value)
    {
        int low = 0;
        int high = constraints.Length;
        var comparer = new ContactConstraintComparer();
        while (low < high)
        {
            int middle = (low + high) >> 1;
            if (comparer.Compare(constraints[middle], value) < 0)
                low = middle + 1;
            else
                high = middle;
        }

        int oldLength = constraints.Length;
        constraints.ResizeUninitialized(oldLength + 1);
        for (int index = oldLength; index > low; index--)
            constraints[index] = constraints[index - 1];
        constraints[low] = value;
    }

    internal static void UpdatePersistentContactAfterScheduledCheck(
        StableEntityPairKey key,
        ContactConstraint pair,
        ushort nextCheckSubstep,
        bool enablePersistentContactCache,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdSolverBodyState> stepStates,
        NativeList<PersistentPredictiveContact> persistentContacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex,
        NativeReference<IncrementalContactCacheState> cacheState)
    {
        int persistentContactIndex = FindPersistentPredictiveContactIndex(
            key,
            persistentContacts,
            contactIndex);
        if (persistentContactIndex < 0)
            return;

        PersistentPredictiveContact contact =
            persistentContacts[persistentContactIndex];
        CrowdBodySnapshot bodyASnapshot = bodies[pair.BodyA];
        CrowdSolverBodyState bodyAStep = stepStates[pair.BodyA];
        CrowdBodySnapshot bodyBSnapshot = bodies[pair.BodyB];
        CrowdSolverBodyState bodyBStep = stepStates[pair.BodyB];
        float3 delta = bodyAStep.SolvedPosition - bodyBStep.SolvedPosition;
        delta.y = 0f;
        float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;
        contact.Lifecycle = math.lengthsq(delta) <= radiusSum * radiusSum
            ? PersistentContactLifecycle.Actual
            : pair.ContactMode == ContactConstraintMode.Predictive
                ? PersistentContactLifecycle.Predictive
                : PersistentContactLifecycle.Approaching;
        contact.ContactMode = pair.ContactMode;
        contact.StableNormal = pair.PredictiveNormal;
        contact.NextCheckSubstep = nextCheckSubstep;
        persistentContacts[persistentContactIndex] = contact;
        InvalidatePersistentContactViews(
            enablePersistentContactCache,
            cacheState);
    }

    internal static void UpdatePersistentContactNextCheck(
        StableEntityPairKey key,
        ushort nextCheckSubstep,
        bool enablePersistentContactCache,
        NativeList<PersistentPredictiveContact> persistentContacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex,
        NativeReference<IncrementalContactCacheState> cacheState)
    {
        int persistentContactIndex = FindPersistentPredictiveContactIndex(
            key,
            persistentContacts,
            contactIndex);
        if (persistentContactIndex < 0)
            return;
        PersistentPredictiveContact contact =
            persistentContacts[persistentContactIndex];
        contact.NextCheckSubstep = nextCheckSubstep;
        persistentContacts[persistentContactIndex] = contact;
        InvalidatePersistentContactViews(
            enablePersistentContactCache,
            cacheState);
    }

    internal static void MarkPersistentContactExpired(
        StableEntityPairKey key,
        bool enablePersistentContactCache,
        NativeList<PersistentPredictiveContact> persistentContacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex,
        NativeReference<IncrementalContactCacheState> cacheState)
    {
        int persistentContactIndex = FindPersistentPredictiveContactIndex(
            key,
            persistentContacts,
            contactIndex);
        if (persistentContactIndex < 0)
            return;
        PersistentPredictiveContact contact =
            persistentContacts[persistentContactIndex];
        contact.Lifecycle = PersistentContactLifecycle.Expired;
        contact.NextCheckSubstep = ushort.MaxValue;
        persistentContacts[persistentContactIndex] = contact;
        InvalidatePersistentContactViews(
            enablePersistentContactCache,
            cacheState);
    }

    internal static int FindPersistentPredictiveContactIndex(
        StableEntityPairKey key,
        NativeList<PersistentPredictiveContact> persistentContacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex)
    {
        if (!contactIndex.IsCreated ||
            !contactIndex.TryGetValue(
                key, out int persistentContactIndex) ||
            (uint)persistentContactIndex >=
            (uint)persistentContacts.Length)
            return -1;
        return persistentContactIndex;
    }

    internal static void InvalidatePersistentContactViews(
        bool enablePersistentContactCache,
        NativeReference<IncrementalContactCacheState> cacheState)
    {
        if (!enablePersistentContactCache)
            return;
        IncrementalContactCacheState state = cacheState.Value;
        state.ContactViewsValid = 0;
        cacheState.Value = state;
    }

    internal static bool TryBuildCurrentScheduledPair(
        int firstBodyIndex,
        int secondBodyIndex,
        ContactPipelineConfiguration configuration,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdMotionEvidence> motionEvidence,
        NativeArray<CrowdSolverBodyState> stepStates,
        out ContactConstraint pair)
    {
        int bodyAIndex = math.min(firstBodyIndex, secondBodyIndex);
        int bodyBIndex = math.max(firstBodyIndex, secondBodyIndex);
        CrowdBodySnapshot bodyASnapshot = bodies[bodyAIndex];
        CrowdMotionEvidence bodyAEvidence = motionEvidence[bodyAIndex];
        CrowdSolverBodyState bodyAStep = stepStates[bodyAIndex];
        CrowdBodySnapshot bodyBSnapshot = bodies[bodyBIndex];
        CrowdMotionEvidence bodyBEvidence = motionEvidence[bodyBIndex];
        CrowdSolverBodyState bodyBStep = stepStates[bodyBIndex];
        float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;
        float candidateDistance =
            radiusSum + math.max(0f, configuration.PredictiveSkin);

        float3 relativeStart = bodyBStep.SolvedPosition - bodyAStep.SolvedPosition;
        float3 relativeDisplacement =
            (bodyBEvidence.BaselineEnd - bodyBStep.SolvedPosition) -
            (bodyAEvidence.BaselineEnd - bodyAStep.SolvedPosition);
        relativeStart.y = 0f;
        relativeDisplacement.y = 0f;
        float relativeLengthSq = math.lengthsq(relativeDisplacement);
        float closestTime = relativeLengthSq > 0.0000001f
            ? math.clamp(
                -math.dot(relativeStart, relativeDisplacement) / relativeLengthSq,
                0f,
                1f)
            : 0f;
        float minDistanceSq = math.lengthsq(
            relativeStart + closestTime * relativeDisplacement);
        if (minDistanceSq > candidateDistance * candidateDistance)
        {
            pair = default;
            return false;
        }

        float startDistanceSq = math.lengthsq(relativeStart);
        float3 endDelta =
            bodyBEvidence.BaselineEnd - bodyAEvidence.BaselineEnd;
        endDelta.y = 0f;
        float endDistanceSq = math.lengthsq(endDelta);
        float radiusSumSq = radiusSum * radiusSum;
        bool isActual = startDistanceSq <= radiusSumSq;
        bool preventSideExchange =
            !isActual &&
            endDistanceSq >= radiusSumSq &&
            minDistanceSq <= radiusSumSq;

        pair = new ContactConstraint
        {
            Definition = new ContactConstraintDefinition
            {
                BodyA = bodyAIndex,
                BodyB = bodyBIndex,
                ContactMode = preventSideExchange &&
                              configuration.EnablePredictiveContacts
                    ? ContactConstraintMode.Predictive
                    : ContactConstraintMode.Regular,
                PredictiveNormal = math.normalizesafe(
                    bodyAStep.SolvedPosition - bodyBStep.SolvedPosition,
                    ContactPipelineMath.DeterministicFallbackNormal(
                        bodyAIndex, bodyBIndex))
            },
            Runtime = new ContactConstraintRuntime
            {
                FirstActivatedSubstep = -1
            }
        };
        return true;
    }

}
}
