using Unity.Collections;

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
        long now = Unity.Profiling.LowLevel.Unsafe.ProfilerUnsafeUtility.Timestamp;
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
}
}
