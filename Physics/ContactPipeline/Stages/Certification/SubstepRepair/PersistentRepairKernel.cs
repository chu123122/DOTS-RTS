using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;
using Unity.Entities;

namespace RTS.Unit.FlowField.Jobs
{
internal static partial class SubstepRepairDataFlow
{




    internal static void PrepareSubstepRepairBuffers(
        NativeReference<ContactPipelineExecutionState> runtimeState,
        ContactPipelineConfiguration configuration,
        NativeReference<byte> fullSweepPrepared,
        NativeList<ContactConstraint> previousTimestepContactPairs,
        NativeList<ContactConstraint> timestepContactPairs,
        NativeList<BodyPair> timestepInteractionPairs,
        NativeList<BodyPair> classificationBodyPairs,
        NativeList<ContactConstraint> pairs,
        NativeList<IncrementalDirtyBody> dirtyBodies,
        NativeReference<IncrementalContactCacheState> incrementalCacheState,
        NativeList<PersistentPairClassificationResult> classificationResults,
        NativeReference<PersistentClassificationPhaseState> classificationState
#if RTS_CONTACT_DIAGNOSTICS
        , NativeReference<PersistentClassificationTelemetryState> telemetryState
#endif
    )
    {
        previousTimestepContactPairs.Clear();
        classificationBodyPairs.Clear();
        pairs.Clear();
        classificationResults.Clear();
        classificationState.Value = default;
        if (runtimeState.Value.IsValid == 0)
            return;
        bool requireDirtyBodies =
            configuration.EnableTimestepContactSetCache;
        if ((requireDirtyBodies &&
             dirtyBodies.Length == 0) ||
            fullSweepPrepared.Value == 0)
            return;

#if RTS_CONTACT_DIAGNOSTICS
        long now = ProfilerUnsafeUtility.Timestamp;
        telemetryState.Value =
            new PersistentClassificationTelemetryState
            {
                BuildStartTimestamp = now,
                ClassificationStartTimestamp = now
            };
#endif
        if (configuration.EnableTimestepContactSetCache)
        {
            previousTimestepContactPairs.ResizeUninitialized(
                timestepContactPairs.Length);
        }
        classificationBodyPairs.ResizeUninitialized(
            timestepInteractionPairs.Length);
        classificationResults.ResizeUninitialized(
            classificationBodyPairs.Length);
        pairs.ResizeUninitialized(classificationBodyPairs.Length);

        classificationState.Value =
            new PersistentClassificationPhaseState
            {
                Timestep = incrementalCacheState.Value.Timestep,
                ClassificationEpoch = ContactClassificationEpoch.Calculate(
                    configuration),
                NeedsCommit = 2
            };
    }

    internal static void FinalizePreparedSubstep(
        int substepIndex,
        NativeReference<ContactPipelineExecutionState> runtimeState,
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
        NativeList<IncrementalDirtyBody> dirtyBodies,
        NativeReference<InteractionCertificate> interactionCertificate,
        NativeList<InteractionCertificateViolation> certificateViolations
#if RTS_CONTACT_DIAGNOSTICS
        , NativeReference<PredictiveDiscContactStatistics> statisticsState,
        NativeReference<IncrementalContactPipelineStatistics>
            incrementalStatisticsState
#endif
    )
    {
        ContactPipelineExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
#if RTS_CONTACT_DIAGNOSTICS
        PredictiveDiscContactStatistics statistics = statisticsState.Value;
        IncrementalContactPipelineStatistics incremental =
            incrementalStatisticsState.Value;
#else
        PredictiveDiscContactStatistics statistics = default;
        IncrementalContactPipelineStatistics incremental = default;
#endif
        int substepCount = math.max(1, configuration.SubstepCount);

        incremental.CorrectedEscapeBodyCount += dirtyBodies.Length;

        PredictiveContactActivationKernel.ActivateScheduledPredictiveContactsForSubstep(
            configuration.EnableTimestepContactSetCache ? substepIndex : 0,
            configuration.EnableTimestepContactSetCache ? substepCount : 1,
            ref incremental,
            configuration,
            bodies,
            motionEvidence,
            stepStates,
            currentBodyIndexByEntity,
            timestepInteractionPairs,
            softAvoidancePairs,
            timestepContactPairs,
            persistentNeighborPairs,
            persistentContacts,
            contactIndex,
            schedule,
            scheduleScratch,
            scheduleCursor,
            cacheState,
            interactionCertificate,
            certificateViolations);
        statistics.TimestepContactSetSubstepUseCount++;
#if RTS_CONTACT_DIAGNOSTICS
        runtime.IterationStartTimestamp = ProfilerUnsafeUtility.Timestamp;
        runtime.IterationAccountedStartNanoseconds =
            ContactPipelineDiagnosticsMath.AccountedCandidateNanoseconds(
                incremental);
#endif
#if RTS_CONTACT_DIAGNOSTICS
        statisticsState.Value = statistics;
        incrementalStatisticsState.Value = incremental;
#endif
        runtimeState.Value = runtime;
    }




}
}
