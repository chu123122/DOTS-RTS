using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public struct PersistentPairClassificationResult
{
    public BodyPair RawPair;
    public PersistentPredictiveContact Contact;
    public byte WasReclassified;
}

public struct PersistentClassificationPhaseState
{
    public uint Timestep;
    public uint ClassificationEpoch;
    public byte NeedsCommit;
}

#if RTS_CONTACT_DIAGNOSTICS
public struct PersistentClassificationTelemetryState
{
    public long BuildStartTimestamp;
    public long ClassificationStartTimestamp;
}
#endif

/// <summary>
/// P5B/P5C support for the staged Jacobi pipeline.
///
/// P5B keeps a persistent cell -> proxy membership view. It is rebuilt only
/// when guarded-proxy topology changes and is used to query dirty bodies without
/// scanning every persistent proxy. Capacity failure invalidates the view and
/// falls back to the authoritative full scan.
///
/// P5C separates persistent-pair classification into a serial prepare phase,
/// a pair-exclusive parallel evaluation phase and a deterministic serial commit.
/// </summary>
[BurstCompile]
internal struct EvaluatePersistentPairClassificationsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
    [ReadOnly] public NativeArray<CrowdMotionIntent> MotionIntents;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
    [ReadOnly] public NativeArray<BodyPair> RawPairs;
    [ReadOnly] public NativeArray<PersistentSweptProxy> PersistentProxies;
    [ReadOnly] public NativeArray<int> PersistentProxyIndexByBody;
    [ReadOnly] public NativeArray<PersistentPredictiveContact> PreviousContacts;
    [ReadOnly] public NativeParallelHashMap<StableEntityPairKey, int>
        PreviousContactIndex;
    [ReadOnly] public NativeArray<byte> DirtyFlagsByBody;
    [ReadOnly] public NativeReference<PersistentClassificationPhaseState> PhaseState;
    public NativeArray<PersistentPairClassificationResult> Results;

    public float PredictiveSkin;
    public float TimestepContactMargin;
    public float SoftAvoidanceShell;
    public float SoftAvoidanceResponseRate;
    public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
    public float RvoTimeHorizon;
    public byte EnablePredictivePairGeneration;
    public byte EnablePredictiveContacts;
    public int SubstepCount;
    public int ScheduleStartSubstep;

    public void Execute(int pairIndex)
    {
        PersistentClassificationPhaseState phase = PhaseState.Value;
        BodyPair rawPair = RawPairs[pairIndex];
        CrowdBodySnapshot bodyASnapshot = Bodies[rawPair.BodyA];
        CrowdNavigationState bodyANavigation = NavigationStates[rawPair.BodyA];
        CrowdMotionIntent bodyAIntent = MotionIntents[rawPair.BodyA];
        CrowdMotionEvidence bodyAEvidence = MotionEvidence[rawPair.BodyA];
        CrowdSolverBodyState bodyAStep = StepStates[rawPair.BodyA];
        CrowdBodySnapshot bodyBSnapshot = Bodies[rawPair.BodyB];
        CrowdNavigationState bodyBNavigation = NavigationStates[rawPair.BodyB];
        CrowdMotionIntent bodyBIntent = MotionIntents[rawPair.BodyB];
        CrowdMotionEvidence bodyBEvidence = MotionEvidence[rawPair.BodyB];
        CrowdSolverBodyState bodyBStep = StepStates[rawPair.BodyB];
        StableEntityPairKey key = StableEntityPairKey.Create(
            bodyASnapshot.Entity,
            bodyBSnapshot.Entity);

        bool hasProxyA = PersistentClassificationDataFlow.TryFindPersistentProxy(
            PersistentProxies,
            PersistentProxyIndexByBody,
            rawPair.BodyA,
            out PersistentSweptProxy proxyA);
        bool hasProxyB = PersistentClassificationDataFlow.TryFindPersistentProxy(
            PersistentProxies,
            PersistentProxyIndexByBody,
            rawPair.BodyB,
            out PersistentSweptProxy proxyB);
        bool hasPrevious = PersistentClassificationDataFlow.TryFindPersistentContact(
            PreviousContacts,
            PreviousContactIndex,
            key,
            out PersistentPredictiveContact previous);
        bool dirtyEndpoint =
            (IncrementalBodyDirtyFlags)DirtyFlagsByBody[rawPair.BodyA] !=
                IncrementalBodyDirtyFlags.None ||
            (IncrementalBodyDirtyFlags)DirtyFlagsByBody[rawPair.BodyB] !=
                IncrementalBodyDirtyFlags.None;
        bool canReuse = !dirtyEndpoint && hasPrevious &&
                        hasProxyA && hasProxyB &&
                        previous.ClassificationEpoch == phase.ClassificationEpoch &&
                        previous.MotionVersionA == proxyA.MotionVersion &&
                        previous.MotionVersionB == proxyB.MotionVersion;

        PersistentPairClassificationResult result = new PersistentPairClassificationResult
        {
            RawPair = rawPair,
            WasReclassified = (byte)(canReuse ? 0 : 1)
        };
        if (canReuse)
        {
            previous.LastSeenTimestep = phase.Timestep;
            result.Contact = previous;
        }
        else
        {
            result.Contact = PersistentClassificationDataFlow.ClassifyPersistentPair(
                key,
                rawPair,
                bodyASnapshot,
                bodyAEvidence,
                bodyAStep,
                bodyBSnapshot,
                bodyBEvidence,
                bodyBStep,
                proxyA,
                proxyB,
                phase.Timestep,
                phase.ClassificationEpoch,
                ScheduleStartSubstep,
                SubstepCount,
                PredictiveSkin,
                TimestepContactMargin,
                SoftAvoidanceShell,
                SoftAvoidanceResponseRate,
                SoftAvoidanceVelocitySolver,
                RvoTimeHorizon,
                EnablePredictivePairGeneration != 0,
                EnablePredictiveContacts != 0);
        }
        Results[pairIndex] = result;
    }
}

internal static partial class PersistentClassificationDataFlow
{
    internal static void PreparePersistentClassification(
        NativeReference<ContactPipelineExecutionState> runtimeState,
        ContactPipelineConfiguration configuration,
        NativeReference<byte> fullSweepPrepared,
        NativeList<ContactConstraint> previousTimestepContactPairs,
        NativeList<BodyPair> timestepInteractionPairs,
        NativeList<BodyPair> classificationBodyPairs,
        NativeReference<IncrementalContactCacheState> incrementalCacheState,
        NativeList<PersistentPairClassificationResult> classificationResults,
        NativeReference<PersistentClassificationPhaseState> classificationState
#if RTS_CONTACT_DIAGNOSTICS
        , NativeReference<PersistentClassificationTelemetryState> telemetryState
#endif
    )
    {
        PersistentClassificationPhaseState phase = default;
#if RTS_CONTACT_DIAGNOSTICS
        PersistentClassificationTelemetryState telemetry =
            new PersistentClassificationTelemetryState
            {
                BuildStartTimestamp = ProfilerUnsafeUtility.Timestamp,
                ClassificationStartTimestamp = ProfilerUnsafeUtility.Timestamp
            };
        telemetryState.Value = telemetry;
#endif
        classificationResults.Clear();
        if (runtimeState.Value.IsValid == 0 ||
            !configuration.EnableTimestepContactSetCache ||
            fullSweepPrepared.Value == 0)
        {
            classificationState.Value = phase;
            return;
        }

        previousTimestepContactPairs.Clear();
        classificationBodyPairs.Clear();
        classificationBodyPairs.AddRange(timestepInteractionPairs.AsArray());
        classificationResults.ResizeUninitialized(classificationBodyPairs.Length);
        phase.Timestep =
            incrementalCacheState.Value.Timestep;
        phase.ClassificationEpoch =
            ContactClassificationEpoch.Calculate(configuration);
        phase.NeedsCommit = 1;
        classificationState.Value = phase;
    }

    internal static bool TryFindPersistentProxy(
        NativeArray<PersistentSweptProxy> proxies,
        NativeArray<int> proxyIndexByBody,
        int bodyIndex,
        out PersistentSweptProxy proxy)
    {
        if ((uint)bodyIndex < (uint)proxyIndexByBody.Length)
        {
            int proxyIndex = proxyIndexByBody[bodyIndex];
            if ((uint)proxyIndex < (uint)proxies.Length)
            {
                proxy = proxies[proxyIndex];
                return proxy.IsValid != 0 &&
                       proxy.BodyIndex == bodyIndex;
            }
        }
        proxy = default;
        return false;
    }

    internal static bool TryFindPersistentContact(
        NativeArray<PersistentPredictiveContact> contacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex,
        StableEntityPairKey key,
        out PersistentPredictiveContact contact)
    {
        if (contactIndex.IsCreated &&
            contactIndex.TryGetValue(key, out int index) &&
            (uint)index < (uint)contacts.Length)
        {
            contact = contacts[index];
            return contact.Key.Equals(key);
        }
        contact = default;
        return false;
    }

    internal static PersistentPredictiveContact ClassifyPersistentPair(
        StableEntityPairKey key,
        BodyPair rawPair,
        CrowdBodySnapshot bodyA,
        CrowdMotionEvidence evidenceA,
        CrowdSolverBodyState stepA,
        CrowdBodySnapshot bodyB,
        CrowdMotionEvidence evidenceB,
        CrowdSolverBodyState stepB,
        PersistentSweptProxy proxyA,
        PersistentSweptProxy proxyB,
        uint timestep,
        uint classificationEpoch,
        int scheduleStartSubstep,
        int substepCount,
        float predictiveSkin,
        float timestepContactMargin,
        float softAvoidanceShell,
        float softAvoidanceResponseRate,
        SoftAvoidanceVelocitySolverMode softAvoidanceVelocitySolver,
        float rvoTimeHorizon,
        bool enablePredictivePairGeneration,
        bool enablePredictiveContacts)
    {
        float radiusSum = bodyA.Radius + bodyB.Radius;
        float3 relativeStart =
            evidenceB.TrajectoryStart - evidenceA.TrajectoryStart;
        float3 relativeDisplacement =
            (evidenceB.BaselineEnd - evidenceB.TrajectoryStart) -
            (evidenceA.BaselineEnd - evidenceA.TrajectoryStart);
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
        float candidateDistance = radiusSum + math.max(0f, predictiveSkin);
        float retainedDistance = candidateDistance +
                                 math.max(0f, timestepContactMargin) * 2f;
        float startDistanceSq = math.lengthsq(relativeStart);
        float3 endDelta =
            evidenceB.BaselineEnd - evidenceA.BaselineEnd;
        endDelta.y = 0f;
        float endDistanceSq = math.lengthsq(endDelta);
        float radiusSumSq = radiusSum * radiusSum;

        PersistentContactLifecycle lifecycle;
        ContactConstraintMode contactMode = ContactConstraintMode.Regular;
        if (minDistanceSq > retainedDistance * retainedDistance ||
            (startDistanceSq > radiusSumSq && !enablePredictivePairGeneration))
        {
            lifecycle = PersistentContactLifecycle.Expired;
        }
        else if (startDistanceSq <= radiusSumSq)
        {
            lifecycle = PersistentContactLifecycle.Actual;
        }
        else if (minDistanceSq > candidateDistance * candidateDistance)
        {
            lifecycle = PersistentContactLifecycle.Dormant;
        }
        else
        {
            bool preventSideExchange =
                endDistanceSq >= radiusSumSq &&
                minDistanceSq <= radiusSumSq;
            lifecycle = preventSideExchange && enablePredictiveContacts
                ? PersistentContactLifecycle.Predictive
                : PersistentContactLifecycle.Approaching;
            contactMode = lifecycle == PersistentContactLifecycle.Predictive
                ? ContactConstraintMode.Predictive
                : ContactConstraintMode.Regular;
        }

        float3 stableNormal = evidenceA.TrajectoryStart -
                              evidenceB.TrajectoryStart;
        stableNormal.y = 0f;
        stableNormal = math.normalizesafe(
            stableNormal,
            ContactPipelineMath.DeterministicFallbackNormal(rawPair.BodyA, rawPair.BodyB));

        ushort firstPossibleSubstep = 0;
        if (lifecycle == PersistentContactLifecycle.Dormant)
        {
            int totalSubstepCount = math.max(1, substepCount);
            if (relativeLengthSq <= 0.0000001f ||
                scheduleStartSubstep >= totalSubstepCount)
            {
                firstPossibleSubstep = ushort.MaxValue;
            }
            else
            {
                int remaining = math.max(
                    1,
                    totalSubstepCount - scheduleStartSubstep);
                int closestOffset = math.clamp(
                    (int)math.floor(closestTime * remaining),
                    0,
                    remaining - 1);
                firstPossibleSubstep = (ushort)(scheduleStartSubstep +
                    math.max(0, closestOffset - 1));
            }
        }

        return new PersistentPredictiveContact
        {
            Key = key,
            StableNormal = stableNormal,
            Lifecycle = lifecycle,
            ContactMode = contactMode,
            FixedSide = contactMode == ContactConstraintMode.Predictive
                ? (sbyte)1
                : (sbyte)0,
            SoftAvoidanceCandidate = (byte)(CouldEnterSoftRange(
                bodyA,
                evidenceA,
                stepA,
                bodyB,
                evidenceB,
                stepB,
                softAvoidanceShell,
                softAvoidanceResponseRate,
                softAvoidanceVelocitySolver,
                rvoTimeHorizon) ? 1 : 0),
            FirstPossibleSubstep = firstPossibleSubstep,
            NextCheckSubstep = firstPossibleSubstep,
            ClosestTime = closestTime,
            LastSeenTimestep = timestep,
            MotionVersionA = proxyA.MotionVersion,
            MotionVersionB = proxyB.MotionVersion,
            ClassificationEpoch = classificationEpoch
        };
    }

    internal static bool CouldEnterSoftRange(
        CrowdBodySnapshot bodyA,
        CrowdMotionEvidence evidenceA,
        CrowdSolverBodyState stepA,
        CrowdBodySnapshot bodyB,
        CrowdMotionEvidence evidenceB,
        CrowdSolverBodyState stepB,
        float softShell,
        float responseRate,
        SoftAvoidanceVelocitySolverMode solverMode,
        float rvoTimeHorizon)
    {
        if (softShell <= 0f || responseRate <= 0f)
            return false;
        float maxDistance = bodyA.Radius + bodyB.Radius + math.max(0f, softShell);
        float3 relativeStart = evidenceB.TrajectoryStart -
                               evidenceA.TrajectoryStart;
        float3 relativeTimestepDisplacement =
            (evidenceB.BaselineEnd - evidenceB.TrajectoryStart) -
            (evidenceA.BaselineEnd - evidenceA.TrajectoryStart);
        relativeStart.y = 0f;
        relativeTimestepDisplacement.y = 0f;
        if (CouldPersistentRelativePathApproach(
                relativeStart,
                relativeTimestepDisplacement,
                maxDistance))
            return true;
        if (solverMode != SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle)
            return false;
        float3 relativeHorizonDisplacement =
            (stepB.BaseVelocity - stepA.BaseVelocity) *
            math.max(0f, rvoTimeHorizon);
        relativeHorizonDisplacement.y = 0f;
        return CouldPersistentRelativePathApproach(
            relativeStart,
            relativeHorizonDisplacement,
            maxDistance);
    }

    internal static bool CouldPersistentRelativePathApproach(
        float3 relativeStart,
        float3 relativeDisplacement,
        float maxDistance)
    {
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
        return minDistanceSq <= maxDistance * maxDistance;
    }
}
}
