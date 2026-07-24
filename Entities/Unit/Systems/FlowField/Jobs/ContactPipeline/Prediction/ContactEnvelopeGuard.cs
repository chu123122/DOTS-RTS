using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Safety boundary for every stage that can invalidate the current
/// InteractionSet/contact view. Each check has a distinct mutation source;
/// failures converge on one incremental-repair/full-rebuild path.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private void ClampSoftOutputToInteractionEnvelope(
        float substepDeltaTime,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        if (!EnableTimestepContactSetCache || substepDeltaTime <= 0f)
            return;

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid ||
                IsSoftOutputInsideInteractionEnvelope(
                    state,
                    state.SoftAvoidanceVelocity,
                    substepDeltaTime))
                continue;

            float3 requestedAvoidance = state.SoftAvoidanceVelocity;
            float lowerScale = 0f;
            float upperScale = 1f;
            if (IsSoftOutputInsideInteractionEnvelope(
                    state,
                    float3.zero,
                    substepDeltaTime))
            {
                // Preserve as much of the avoidance response as the
                // already-proven InteractionSet envelope can contain.
                for (int iteration = 0; iteration < 8; iteration++)
                {
                    float middleScale = (lowerScale + upperScale) * 0.5f;
                    if (IsSoftOutputInsideInteractionEnvelope(
                            state,
                            requestedAvoidance * middleScale,
                            substepDeltaTime))
                        lowerScale = middleScale;
                    else
                        upperScale = middleScale;
                }
            }

            state.SoftAvoidanceVelocity = requestedAvoidance * lowerScale;
            States[bodyIndex] = state;
            incrementalStatistics.InteractionEnvelopeEscapeCount++;
        }
    }

    private bool IsSoftOutputInsideInteractionEnvelope(
        FlowMovementFrameState state,
        float3 avoidanceVelocity,
        float substepDeltaTime)
    {
        float responseRate = math.max(0f, SoftAvoidanceResponseRate);
        if (state.IsSettled)
            responseRate *= math.max(0f, SettledSoftAvoidanceMultiplier);

        float3 velocity = SoftAvoidanceMath.ApplyVelocityBuffer(
            state.BasePredictedVelocity,
            avoidanceVelocity,
            responseRate,
            substepDeltaTime,
            state.MoveSpeed);
        if (state.IsSettled)
            velocity *= math.pow(0.8f, substepDeltaTime * 60f);
        if (math.lengthsq(velocity) > state.MoveSpeed * state.MoveSpeed)
            velocity = math.normalizesafe(velocity) * state.MoveSpeed;

        float3 predictedEnd = state.PredictedPosition +
                              velocity * substepDeltaTime;
        float contactPadding = math.max(0f, PredictiveSkin) +
                               math.max(0f, TimestepContactMargin);
        float avoidancePadding = math.max(0f, SoftAvoidanceShell) * 0.5f;
        float extent = math.max(0f, state.Radius) +
                       math.max(contactPadding, avoidancePadding);
        return AabbContains(
            state.TimestepInteractionEnvelopeMin,
            state.TimestepInteractionEnvelopeMax,
            predictedEnd.xz - extent,
            predictedEnd.xz + extent);
    }

    private bool ValidateSolverCorrectionContactEnvelope(
        int substepIndex,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        bool allInside = true;
        ClearIncrementalDirtyBodySet();
        float skin = math.max(0f, PredictiveSkin);
        for (int correctedIndex = 0; correctedIndex < CorrectedBodyIndices.Length; correctedIndex++)
        {
            int bodyIndex = CorrectedBodyIndices[correctedIndex];
            FlowMovementFrameState state = States[bodyIndex];
            float extent = math.max(0f, state.Radius) + skin;
            float2 currentMin = state.PredictedPosition.xz - extent;
            float2 currentMax = state.PredictedPosition.xz + extent;
            if (AabbContains(
                    state.TimestepEnvelopeMin,
                    state.TimestepEnvelopeMax,
                    currentMin,
                    currentMax))
                continue;

            allInside = false;
            SetIncrementalDirtyFlags(
                bodyIndex,
                IncrementalBodyDirtyFlags.Motion |
                IncrementalBodyDirtyFlags.CorrectedEscape);
            if (state.TimestepEscaped == 0)
            {
                state.TimestepEscaped = 1;
                statistics.TimestepContactSetEscapeBodyCount++;
                if (statistics.TimestepContactSetFirstEscapeSubstep < 0)
                    statistics.TimestepContactSetFirstEscapeSubstep = substepIndex;
                States[bodyIndex] = state;
            }
        }
        incrementalStatistics.CorrectedEscapeBodyCount += IncrementalDirtyBodies.Length;
        return allInside;
    }

    private bool ValidatePredictedContactEnvelope(
        int substepIndex,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        ClearIncrementalDirtyBodySet();
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;
            float extent = math.max(0f, state.Radius) +
                           math.max(0f, PredictiveSkin);
            float2 currentMin = state.PredictedPosition.xz - extent;
            float2 currentMax = state.PredictedPosition.xz + extent;
            if (AabbContains(
                    state.TimestepEnvelopeMin,
                    state.TimestepEnvelopeMax,
                    currentMin,
                    currentMax))
                continue;
            MarkContactEnvelopeEscape(
                bodyIndex,
                substepIndex,
                IncrementalBodyDirtyFlags.Motion,
                ref statistics);
        }
        incrementalStatistics.CorrectedEscapeBodyCount +=
            IncrementalDirtyBodies.Length;
        return IncrementalDirtyBodies.Length == 0;
    }

    private bool ValidateBaseMotionInteractionEnvelope(
        int substepIndex,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        ClearIncrementalDirtyBodySet();
        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid)
                continue;
            CalculateIncrementalValidationBounds(
                state,
                out float2 currentMin,
                out float2 currentMax);
            if (AabbContains(
                    state.TimestepInteractionEnvelopeMin,
                    state.TimestepInteractionEnvelopeMax,
                    currentMin,
                    currentMax))
                continue;
            MarkContactEnvelopeEscape(
                bodyIndex,
                substepIndex,
                IncrementalBodyDirtyFlags.Motion,
                ref statistics);
        }
        incrementalStatistics.InteractionEnvelopeEscapeCount +=
            IncrementalDirtyBodies.Length;
        return IncrementalDirtyBodies.Length == 0;
    }

    private void MarkContactEnvelopeEscape(
        int bodyIndex,
        int substepIndex,
        IncrementalBodyDirtyFlags source,
        ref PredictiveDiscContactStatistics statistics)
    {
        SetIncrementalDirtyFlags(
            bodyIndex,
            IncrementalBodyDirtyFlags.Motion | source);
        FlowMovementFrameState state = States[bodyIndex];
        if (state.TimestepEscaped != 0)
            return;
        state.TimestepEscaped = 1;
        statistics.TimestepContactSetEscapeBodyCount++;
        if (statistics.TimestepContactSetFirstEscapeSubstep < 0)
            statistics.TimestepContactSetFirstEscapeSubstep = substepIndex;
        States[bodyIndex] = state;
    }

    private void RepairOrRebuildContactViewForRemainingTime(
        int substepIndex,
        int substepCount,
        float substepDeltaTime,
        bool persistAcrossSubsteps,
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        bool activationAlreadyPassed = true)
    {
        float remainingDuration = persistAcrossSubsteps
            ? math.max(
                substepDeltaTime,
                (substepCount - substepIndex) * substepDeltaTime)
            : 0f;
        PrepareTimestepContactPrediction(remainingDuration, true);
        int scheduleStartSubstep = substepIndex +
                                   (activationAlreadyPassed ? 1 : 0);
        if (EnablePersistentContactCache &&
            TryIncrementallyRepairEscapedContactSet(
                substepIndex,
                scheduleStartSubstep,
                ref statistics,
                ref incrementalStatistics))
        {
            return;
        }

        BuildTimestepContactSet(
            ref statistics,
            ref incrementalStatistics,
            true,
            true,
            scheduleStartSubstep);
    }
}
}
