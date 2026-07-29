using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 为每个可能越出当前证书作用域的变更来源提供权威越界证据。
/// 本层不决定缓存有效性；所有失败都会汇聚到交互认证器的修复/重建路径。
/// </summary>
internal partial struct CertificationStageKernel
{
    private void ClampSoftOutputToInteractionEnvelope(
        float substepDeltaTime,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        if (!Configuration.EnableTimestepContactSetCache || substepDeltaTime <= 0f)
            return;

        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0) ||
                IsSoftOutputInsideInteractionEnvelope(
                    stateSnapshot,
                    stateNavigation,
                    stateEvidence,
                    stateStep,
                    stateStep.SoftAvoidanceVelocity,
                    substepDeltaTime))
                continue;

            float3 requestedAvoidance = stateStep.SoftAvoidanceVelocity;
            float lowerScale = 0f;
            float upperScale = 1f;
            if (IsSoftOutputInsideInteractionEnvelope(
                    stateSnapshot,
                    stateNavigation,
                    stateEvidence,
                    stateStep,
                    float3.zero,
                    substepDeltaTime))
            {
                // 在已认证的 InteractionSet 包络内尽量保留避让响应。
                for (int iteration = 0; iteration < 8; iteration++)
                {
                    float middleScale = (lowerScale + upperScale) * 0.5f;
                    if (IsSoftOutputInsideInteractionEnvelope(
                            stateSnapshot,
                            stateNavigation,
                            stateEvidence,
                            stateStep,
                            requestedAvoidance * middleScale,
                            substepDeltaTime))
                        lowerScale = middleScale;
                    else
                        upperScale = middleScale;
                }
            }

            stateStep.SoftAvoidanceVelocity = requestedAvoidance * lowerScale;
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
            incrementalStatistics.InteractionEnvelopeEscapeCount++;
        }
    }

    private bool IsSoftOutputInsideInteractionEnvelope(
        CrowdBodySnapshot stateSnapshot,
        CrowdNavigationState stateNavigation,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        float3 avoidanceVelocity,
        float substepDeltaTime)
    {
        float responseRate = math.max(0f, Configuration.SoftAvoidanceResponseRate);
        if ((stateNavigation.IsSettled != 0))
            responseRate *= math.max(0f, Configuration.SettledSoftAvoidanceMultiplier);

        float3 velocity = SoftAvoidanceMath.ApplyVelocityBuffer(
            stateStep.BaseVelocity,
            avoidanceVelocity,
            responseRate,
            substepDeltaTime,
            stateSnapshot.MoveSpeed);
        if ((stateNavigation.IsSettled != 0))
            velocity *= math.pow(0.8f, substepDeltaTime * 60f);
        if (math.lengthsq(velocity) > stateSnapshot.MoveSpeed * stateSnapshot.MoveSpeed)
            velocity = math.normalizesafe(velocity) * stateSnapshot.MoveSpeed;

        float3 predictedEnd = stateStep.SolvedPosition +
                              velocity * substepDeltaTime;
        float contactPadding = math.max(0f, Configuration.PredictiveSkin) +
                               math.max(0f, Configuration.TimestepContactMargin);
        float avoidancePadding = math.max(0f, Configuration.SoftAvoidanceShell) * 0.5f;
        float extent = math.max(0f, stateSnapshot.Radius) +
                       math.max(contactPadding, avoidancePadding);
        return ContactPipelineShared.AabbContains(
            stateEvidence.InteractionEnvelopeMin,
            stateEvidence.InteractionEnvelopeMax,
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
        float skin = math.max(0f, Configuration.PredictiveSkin);
        for (int correctedIndex = 0; correctedIndex < CorrectedBodyIndices.Length; correctedIndex++)
        {
            int bodyIndex = CorrectedBodyIndices[correctedIndex];
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            float extent = math.max(0f, stateSnapshot.Radius) + skin;
            float2 currentMin = stateStep.SolvedPosition.xz - extent;
            float2 currentMax = stateStep.SolvedPosition.xz + extent;
            if (ContactPipelineShared.AabbContains(
                    stateEvidence.ContactEnvelopeMin,
                    stateEvidence.ContactEnvelopeMax,
                    currentMin,
                    currentMax))
                continue;

            allInside = false;
            RevokeInteractionCertificate(
                bodyIndex,
                substepIndex,
                InteractionCertificateViolationReason.SolverCorrectionEnvelopeEscape,
                currentMin,
                currentMax);
            SetIncrementalDirtyFlags(
                bodyIndex,
                IncrementalBodyDirtyFlags.Motion |
                IncrementalBodyDirtyFlags.CorrectedEscape);
            if (stateEvidence.EnvelopeEscaped == 0)
            {
                stateEvidence.EnvelopeEscaped = 1;
                statistics.TimestepContactSetEscapeBodyCount++;
                if (statistics.TimestepContactSetFirstEscapeSubstep < 0)
                    statistics.TimestepContactSetFirstEscapeSubstep = substepIndex;
                Bodies[bodyIndex] = stateSnapshot;
                NavigationStates[bodyIndex] = stateNavigation;
                MotionIntents[bodyIndex] = stateIntent;
                MotionEvidence[bodyIndex] = stateEvidence;
                StepStates[bodyIndex] = stateStep;
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
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
                continue;
            float extent = math.max(0f, stateSnapshot.Radius) +
                           math.max(0f, Configuration.PredictiveSkin);
            float2 currentMin = stateStep.SolvedPosition.xz - extent;
            float2 currentMax = stateStep.SolvedPosition.xz + extent;
            if (ContactPipelineShared.AabbContains(
                    stateEvidence.ContactEnvelopeMin,
                    stateEvidence.ContactEnvelopeMax,
                    currentMin,
                    currentMax))
                continue;
            MarkContactEnvelopeEscape(
                bodyIndex,
                substepIndex,
                IncrementalBodyDirtyFlags.Motion,
                InteractionCertificateViolationReason.PredictedContactEnvelopeEscape,
                currentMin,
                currentMax,
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
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
                continue;
            CalculateIncrementalValidationBounds(
                stateSnapshot,
                stateEvidence,
                stateStep,
                out float2 currentMin,
                out float2 currentMax);
            if (ContactPipelineShared.AabbContains(
                    stateEvidence.InteractionEnvelopeMin,
                    stateEvidence.InteractionEnvelopeMax,
                    currentMin,
                    currentMax))
                continue;
            MarkContactEnvelopeEscape(
                bodyIndex,
                substepIndex,
                IncrementalBodyDirtyFlags.Motion,
                InteractionCertificateViolationReason.BaseMotionEnvelopeEscape,
                currentMin,
                currentMax,
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
        InteractionCertificateViolationReason reason,
        float2 observedMin,
        float2 observedMax,
        ref PredictiveDiscContactStatistics statistics)
    {
        RevokeInteractionCertificate(
            bodyIndex,
            substepIndex,
            reason,
            observedMin,
            observedMax);
        SetIncrementalDirtyFlags(
            bodyIndex,
            IncrementalBodyDirtyFlags.Motion | source);
        CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
        CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
        CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
        CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
        CrowdBodyStepState stateStep = StepStates[bodyIndex];
        if (stateEvidence.EnvelopeEscaped != 0)
            return;
        stateEvidence.EnvelopeEscaped = 1;
        statistics.TimestepContactSetEscapeBodyCount++;
        if (statistics.TimestepContactSetFirstEscapeSubstep < 0)
            statistics.TimestepContactSetFirstEscapeSubstep = substepIndex;
        Bodies[bodyIndex] = stateSnapshot;
        NavigationStates[bodyIndex] = stateNavigation;
        MotionIntents[bodyIndex] = stateIntent;
        MotionEvidence[bodyIndex] = stateEvidence;
        StepStates[bodyIndex] = stateStep;
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
        if (Configuration.EnablePersistentContactCache &&
            TryIncrementallyRepairEscapedContactSet(
                substepIndex,
                scheduleStartSubstep,
                ref statistics,
                ref incrementalStatistics))
        {
            IssueInteractionCertificate(
                new ContactViewBuildResult
                {
                    SourceMode = ContactInteractionSourceMode.PersistentRepair,
                    PersistentViewReady = 1,
                    UsedFullRebuild = 0,
                    RepairedBodyCount = IncrementalDirtyBodies.Length,
                    InteractionPairCount = PersistentNeighborPairs.Length
                },
                scheduleStartSubstep);
            return;
        }

        BuildOrRefreshTimestepContactViews(
            ref statistics,
            ref incrementalStatistics,
            true,
            true,
            scheduleStartSubstep);
    }
}
}
