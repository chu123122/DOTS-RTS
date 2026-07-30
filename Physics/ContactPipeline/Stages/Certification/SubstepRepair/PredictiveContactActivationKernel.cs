using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
internal static class PredictiveContactActivationKernel
{
    internal static bool TryFindCurrentBodyIndex(
        Entity entity,
        int bodyCount,
        NativeParallelHashMap<Entity, int> currentBodyIndexByEntity,
        out int bodyIndex)
    {
        return currentBodyIndexByEntity.TryGetValue(entity, out bodyIndex) &&
               (uint)bodyIndex < (uint)bodyCount;
    }

    internal static int FindPersistentPredictiveContactIndex(
        StableEntityPairKey key,
        NativeArray<PersistentPredictiveContact> persistentContacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex)
    {
        if (!contactIndex.IsCreated ||
            !contactIndex.TryGetValue(key, out int persistentContactIndex) ||
            (uint)persistentContactIndex >=
            (uint)persistentContacts.Length)
            return -1;
        return persistentContactIndex;
    }

    internal static bool TryBuildPersistentContactAfterScheduledCheck(
        int persistentContactIndex,
        ContactConstraint pair,
        ushort nextCheckSubstep,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdSolverBodyState> stepStates,
        NativeArray<PersistentPredictiveContact> persistentContacts,
        out PersistentPredictiveContact contact)
    {
        if ((uint)persistentContactIndex >=
            (uint)persistentContacts.Length)
        {
            contact = default;
            return false;
        }

        contact = persistentContacts[persistentContactIndex];
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
        return true;
    }

    internal static bool TryBuildPersistentContactNextCheck(
        int persistentContactIndex,
        ushort nextCheckSubstep,
        NativeArray<PersistentPredictiveContact> persistentContacts,
        out PersistentPredictiveContact contact)
    {
        if ((uint)persistentContactIndex >=
            (uint)persistentContacts.Length)
        {
            contact = default;
            return false;
        }
        contact = persistentContacts[persistentContactIndex];
        contact.NextCheckSubstep = nextCheckSubstep;
        return true;
    }

    internal static bool TryBuildExpiredPersistentContact(
        int persistentContactIndex,
        NativeArray<PersistentPredictiveContact> persistentContacts,
        out PersistentPredictiveContact contact)
    {
        if ((uint)persistentContactIndex >=
            (uint)persistentContacts.Length)
        {
            contact = default;
            return false;
        }
        contact = persistentContacts[persistentContactIndex];
        contact.Lifecycle = PersistentContactLifecycle.Expired;
        contact.NextCheckSubstep = ushort.MaxValue;
        return true;
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

        float3 relativeStart =
            bodyBStep.SolvedPosition - bodyAStep.SolvedPosition;
        float3 relativeDisplacement =
            (bodyBEvidence.BaselineEnd - bodyBStep.SolvedPosition) -
            (bodyAEvidence.BaselineEnd - bodyAStep.SolvedPosition);
        relativeStart.y = 0f;
        relativeDisplacement.y = 0f;
        float relativeLengthSq = math.lengthsq(relativeDisplacement);
        float closestTime = relativeLengthSq > 0.0000001f
            ? math.clamp(
                -math.dot(relativeStart, relativeDisplacement) /
                relativeLengthSq,
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
                        bodyAIndex,
                        bodyBIndex))
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
