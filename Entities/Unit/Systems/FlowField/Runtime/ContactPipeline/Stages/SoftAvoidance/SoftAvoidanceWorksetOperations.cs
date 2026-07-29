using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;
namespace RTS.Unit.FlowField.Jobs
{
public partial struct SoftAvoidanceJob
{
    private void EnsureSoftIncidentIndex()
    {
        ActiveIncidentIndexState state = ActiveIncidentIndexState.Value;
        if (state.SoftIsValid != 0 &&
            state.SoftPairCount == SoftAvoidancePairs.Length &&
            state.SoftBodyCount == Bodies.Length)
            return;

        BuildSoftIncidentIndex();
        state = ActiveIncidentIndexState.Value;
        state.SoftPairCount = SoftAvoidancePairs.Length;
        state.SoftBodyCount = Bodies.Length;
        state.SoftIsValid = 1;
        ActiveIncidentIndexState.Value = state;
    }

    private void BuildSoftIncidentIndex()
    {
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
            SoftIncidentWriteCursors[bodyIndex] = 0;
        for (int pairIndex = 0; pairIndex < SoftAvoidancePairs.Length; pairIndex++)
        {
            BodyPair pair = SoftAvoidancePairs[pairIndex];
            SoftIncidentWriteCursors[pair.BodyA]++;
            SoftIncidentWriteCursors[pair.BodyB]++;
        }
        int entries = 0;
        SoftIncidentOffsets[0] = 0;
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            entries += SoftIncidentWriteCursors[bodyIndex];
            SoftIncidentOffsets[bodyIndex + 1] = entries;
            SoftIncidentWriteCursors[bodyIndex] = SoftIncidentOffsets[bodyIndex];
        }
        SoftIncidentPairIndices.ResizeUninitialized(entries);
        for (int pairIndex = 0; pairIndex < SoftAvoidancePairs.Length; pairIndex++)
        {
            BodyPair pair = SoftAvoidancePairs[pairIndex];
            SoftIncidentPairIndices[SoftIncidentWriteCursors[pair.BodyA]++] = pairIndex;
            SoftIncidentPairIndices[SoftIncidentWriteCursors[pair.BodyB]++] = pairIndex;
        }
    }

    private void PrepareSoftWorkset(
        NativeReference<ContactPipelineExecutionState> runtimeState
#if RTS_CONTACT_DIAGNOSTICS
        , NativeList<JacobiBlockTelemetry> blockStatistics
#endif
        )
    {
        // Deferred jobs capture list lengths at schedule time and must see the
        // correct Capacity before this job begins. Resize unconditionally so that
        // the downstream EvaluateSoftAvoidancePairsJob never writes into a
        // zero-length array even when the runtime state is not yet valid.
        EnsureSoftIncidentIndex();
        SoftPairContributions.ResizeUninitialized(SoftAvoidancePairs.Length);

        ContactPipelineExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
#if RTS_CONTACT_DIAGNOSTICS
        blockStatistics.ResizeUninitialized(
            (SoftAvoidancePairs.Length + CrowdContactPipelineScheduler.SoftPairBatchSize - 1) / CrowdContactPipelineScheduler.SoftPairBatchSize);
        runtime.IterationStartTimestamp = ProfilerUnsafeUtility.Timestamp;
#endif
        runtimeState.Value = runtime;
    }

#if RTS_CONTACT_DIAGNOSTICS
    private void FinalizeSoftAvoidance(
        NativeReference<ContactPipelineExecutionState> runtimeState,
        NativeList<JacobiBlockTelemetry> blocks,
        NativeArray<int> escapeCountsByBlock,
        int escapeBlockCount)
    {
        ContactPipelineExecutionState runtime = runtimeState.Value;
        if (runtime.IsValid == 0)
            return;
        PredictiveDiscContactStatistics statistics = LoadContactStatistics();
        IncrementalContactPipelineStatistics incremental = LoadIncrementalStatistics();
        int activated = 0;
        for (int i = 0; i < blocks.Length; i++)
            activated += blocks[i].NewlyActivatedPairCount;
        int escaped = 0;
        for (int blockIndex = 0; blockIndex < escapeBlockCount; blockIndex++)
            escaped += escapeCountsByBlock[blockIndex];
        if (EnablePersistentContactCache &&
            SoftAvoidanceShell > 0f && SoftAvoidanceResponseRate > 0f)
            statistics.SoftAvoidanceFatAabbUseCount++;
        statistics.SoftAvoidanceCandidatePairCount += SoftAvoidancePairs.Length;
        statistics.SoftAvoidanceActivatedPairCount += activated;
        statistics.SoftAvoidanceEvaluationCount++;
        statistics.SoftAvoidanceNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - runtime.IterationStartTimestamp);
        incremental.SoftAvoidancePairEvaluationCount += SoftAvoidancePairs.Length;
        incremental.InteractionEnvelopeEscapeCount += escaped;
        StoreContactStatistics(statistics);
        StoreIncrementalStatistics(incremental);
    }
#endif



}
}
