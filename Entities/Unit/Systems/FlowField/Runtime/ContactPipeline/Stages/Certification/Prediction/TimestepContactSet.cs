using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal enum ContactInteractionSourceMode : byte
{
    FullSweep,
    PersistentReuse,
    PersistentRepair,
    PersistentFullRebuild
}

internal struct ContactViewBuildResult
{
    public ContactInteractionSourceMode SourceMode;
    public byte PersistentViewReady;
    public byte UsedFullRebuild;
    public int RepairedBodyCount;
    public int InteractionPairCount;
}

public partial struct InteractionCertificationJob
{
    private void PrepareTimestepContactPrediction(float duration, bool fromSolvedPosition)
    {
        duration = math.max(0f, duration);
        float margin = math.max(0f, TimestepContactMargin);
        float skin = math.max(0f, PredictiveSkin);

        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
                continue;

            float3 start = fromSolvedPosition
                ? stateStep.SolvedPosition
                : stateSnapshot.Position;
            float3 velocity = fromSolvedPosition
                ? stateStep.BaseVelocity
                : ContactPipelineMath.CalculateBaseVelocityForSubstep(
                    Grid,
                    EnvironmentGeometry,
                    stateSnapshot, stateNavigation, stateIntent, stateStep, duration);
            if ((stateNavigation.IsSettled != 0))
                velocity *= math.pow(0.8f, duration * 60f);
            if (math.lengthsq(velocity) > stateSnapshot.MoveSpeed * stateSnapshot.MoveSpeed)
                velocity = math.normalizesafe(velocity) * stateSnapshot.MoveSpeed;

            float3 end = start + velocity * duration;
            end.y = stateSnapshot.Position.y;
            float extent = math.max(0f, stateSnapshot.Radius) + skin + margin;
            stateEvidence.TrajectoryStart = start;
            stateEvidence.BaselineEnd = end;
            stateStep.BaseVelocity = velocity;
            stateEvidence.ContactEnvelopeMin = math.min(start.xz, end.xz) - extent;
            stateEvidence.ContactEnvelopeMax = math.max(start.xz, end.xz) + extent;
            CalculateIncrementalTightSweptBounds(
                stateSnapshot,
                stateEvidence,
                stateStep,
                out stateEvidence.InteractionEnvelopeMin,
                out stateEvidence.InteractionEnvelopeMax);
            if (!fromSolvedPosition)
                stateEvidence.EnvelopeEscaped = 0;
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
        }
    }

    private void PrepareSubstepContactPrediction()
    {
        float margin = math.max(0f, TimestepContactMargin);
        float skin = math.max(0f, PredictiveSkin);

        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0))
                continue;

            float3 start = stateStep.SubstepStartPosition;
            float3 end = stateStep.SolvedPosition;
            float extent = math.max(0f, stateSnapshot.Radius) + skin + margin;
            stateEvidence.TrajectoryStart = start;
            stateEvidence.BaselineEnd = end;
            stateEvidence.ContactEnvelopeMin = math.min(start.xz, end.xz) - extent;
            stateEvidence.ContactEnvelopeMax = math.max(start.xz, end.xz) + extent;
            CalculateIncrementalTightSweptBounds(
                stateSnapshot,
                stateEvidence,
                stateStep,
                out stateEvidence.InteractionEnvelopeMin,
                out stateEvidence.InteractionEnvelopeMax);
            stateEvidence.EnvelopeEscaped = 0;
            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
        }
    }

    private void BuildSubstepInteractionAndSoftViews(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long startTimestamp = ProfilerUnsafeUtility.Timestamp;
        BuildSweptInteractionPairs(ref statistics);
        BuildSoftAvoidancePairViewFromInteractions(ref incrementalStatistics);
        long elapsed = ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - startTimestamp);
        incrementalStatistics.FullSweepSourceNanoseconds += elapsed;
        incrementalStatistics.CurrentInteractionPairCount =
            TimestepInteractionPairs.Length;
        statistics.TimestepContactSetBuildNanoseconds += elapsed;

        // B0 仍在 Soft Avoidance 读取紧凑对视图前发布显式证书；下方会在未约束预测分类后重认证接触约束。
        IssueFullSweepSubstepCertificate();
    }

    private void BuildSubstepContactView(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        long startTimestamp = ProfilerUnsafeUtility.Timestamp;
        PreviousTimestepContactPairs.Clear();
        ClassifyTimestepContacts(
            ref statistics,
            ref incrementalStatistics,
            0);
        CommitTimestepContactViews(
            ref statistics,
            ref incrementalStatistics,
            false);
        ValidateIncrementalContactSetAgainstQuadraticOracle(
            ref incrementalStatistics);
        statistics.TimestepContactSetBuildNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - startTimestamp);
    }

    private ContactViewBuildResult BuildOrRefreshTimestepContactViews(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        bool forceFullBroadPhase,
        bool fallback,
        int scheduleStartSubstep = 0)
    {
        long startTimestamp = ProfilerUnsafeUtility.Timestamp;
        PreviousTimestepContactPairs.Clear();
        if (fallback)
            PreviousTimestepContactPairs.AddRange(TimestepContactPairs.AsArray());

        ContactViewBuildResult result = ResolveInteractionSource(
            ref statistics,
            ref incrementalStatistics,
            forceFullBroadPhase,
            scheduleStartSubstep);
        ObserveContactViewBuildResult(result, ref incrementalStatistics);

        if (result.PersistentViewReady != 0)
        {
            ValidateSoftAvoidancePairViewAgainstQuadraticOracle(
                ref incrementalStatistics);
        }
        else
        {
            BuildSoftAvoidancePairViewFromInteractions(ref incrementalStatistics);
            ClassifyTimestepContacts(
                ref statistics,
                ref incrementalStatistics,
                scheduleStartSubstep);
        }

        CommitTimestepContactViews(
            ref statistics,
            ref incrementalStatistics,
            fallback,
            scheduleStartSubstep);
        ValidateIncrementalContactSetAgainstQuadraticOracle(
            ref incrementalStatistics);

        long elapsed = ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - startTimestamp);
        statistics.TimestepContactSetBuildNanoseconds += elapsed;
        if (fallback)
        {
            statistics.TimestepContactSetFullRebuildCount++;
            statistics.TimestepContactSetFallbackNanoseconds += elapsed;
        }
        return result;
    }

    private ContactViewBuildResult ResolveInteractionSource(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        bool forceFullBroadPhase,
        int scheduleStartSubstep)
    {
        int repairCountBefore = incrementalStatistics.IncrementalRepairCount;
        int fullRebuildCountBefore = incrementalStatistics.FullRebuildCount;
        bool sourcedFromIncrementalCache = false;
        bool persistentViewReady = false;

        if (EnablePersistentContactCache)
        {
            sourcedFromIncrementalCache = BuildContactPairsFromPersistentNeighborSet(
                ref statistics,
                ref incrementalStatistics,
                forceFullBroadPhase,
                scheduleStartSubstep,
                out persistentViewReady);
        }
        else
        {
            long fullSweepStart = ProfilerUnsafeUtility.Timestamp;
            BuildSweptInteractionPairs(ref statistics);
            incrementalStatistics.FullSweepSourceNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                ProfilerUnsafeUtility.Timestamp - fullSweepStart);
        }

        int interactionPairCount = persistentViewReady
            ? PersistentNeighborPairs.Length
            : TimestepInteractionPairs.Length;
        ContactInteractionSourceMode sourceMode = ContactInteractionSourceMode.FullSweep;
        if (persistentViewReady)
        {
            if (incrementalStatistics.FullRebuildCount > fullRebuildCountBefore)
                sourceMode = ContactInteractionSourceMode.PersistentFullRebuild;
            else if (incrementalStatistics.IncrementalRepairCount > repairCountBefore)
                sourceMode = ContactInteractionSourceMode.PersistentRepair;
            else
                sourceMode = ContactInteractionSourceMode.PersistentReuse;
        }

        return new ContactViewBuildResult
        {
            SourceMode = sourceMode,
            PersistentViewReady = (byte)(persistentViewReady ? 1 : 0),
            UsedFullRebuild = (byte)(
                sourceMode == ContactInteractionSourceMode.PersistentFullRebuild ||
                (!sourcedFromIncrementalCache && forceFullBroadPhase)
                    ? 1
                    : 0),
            RepairedBodyCount = sourceMode == ContactInteractionSourceMode.PersistentRepair
                ? math.max(0, incrementalStatistics.TopologyDirtyBodyCount)
                : 0,
            InteractionPairCount = interactionPairCount
        };
    }

    private static void ObserveContactViewBuildResult(
        ContactViewBuildResult result,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        incrementalStatistics.CurrentInteractionPairCount = result.InteractionPairCount;
        if (result.UsedFullRebuild != 0)
            incrementalStatistics.UsedFullRebuild = 1;
    }

    private void ClassifyTimestepContacts(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int scheduleStartSubstep)
    {
        Pairs.Clear();
        ContactPipelineShared.AppendBodyPairsAsConstraints(TimestepInteractionPairs.AsArray(), Pairs);
        int classificationCandidateCount = Pairs.Length;
        statistics.CandidatePairCount += classificationCandidateCount;
        incrementalStatistics.ReclassifiedPairEvaluationCount +=
            classificationCandidateCount;
        incrementalStatistics.SweptClassificationEvaluationCount +=
            classificationCandidateCount;
        long classificationStart = ProfilerUnsafeUtility.Timestamp;
        FilterAndClassifyPairs(ref statistics, math.max(0f, PredictiveSkin));
        incrementalStatistics.SweptClassificationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - classificationStart);

        long scheduleStart = ProfilerUnsafeUtility.Timestamp;
        BuildTimestepPredictiveSchedule(
            ref incrementalStatistics,
            scheduleStartSubstep);
        incrementalStatistics.ContactActivationNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
            ProfilerUnsafeUtility.Timestamp - scheduleStart);
    }

    private void CommitTimestepContactViews(
        ref PredictiveDiscContactStatistics statistics,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        bool fallback,
        int scheduleStartSubstep = 0)
    {
        TimestepContactPairs.Clear();
        TimestepContactPairs.AddRange(Pairs.AsArray());
        statistics.TimestepContactSetBuildCount++;
        statistics.TimestepContactSetClassificationPassCount++;
        statistics.TimestepContactSetUniquePairCount = TimestepContactPairs.Length;
        statistics.TimestepContactSetDormantPairCount =
            incrementalStatistics.CurrentDormantPairCount;
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            ContactConstraint pair = TimestepContactPairs[pairIndex];
            if (!fallback)
                continue;

            int previousIndex = FindPairIndex(
                PreviousTimestepContactPairs,
                pair.BodyA,
                pair.BodyB);
            if (previousIndex >= 0)
            {
                ContactConstraint previous = PreviousTimestepContactPairs[previousIndex];
                pair.WasActivatedThisTimestep = previous.WasActivatedThisTimestep;
                pair.WasCorrectedThisTimestep = previous.WasCorrectedThisTimestep;
                pair.FirstActivatedSubstep = previous.FirstActivatedSubstep;
                pair.ActivatedSubstepCount = previous.ActivatedSubstepCount;
                pair.WasAddedByFallback = previous.WasAddedByFallback;
            }
            else
            {
                pair.WasAddedByFallback = 1;
                statistics.TimestepContactSetFallbackAddedPairCount++;
            }
            TimestepContactPairs[pairIndex] = pair;
        }

        // 这是串行与分阶段 Jacobi 共享的唯一消费者视图提交边界；候选缓存来源在证书背后，对下层是不可见的。
        IssueCertificateForCommittedViews(
            incrementalStatistics,
            scheduleStartSubstep);
    }

    private void ResetTimestepContactSetForSubstep()
    {
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            ContactConstraint pair = TimestepContactPairs[pairIndex];
            pair.Lambda = 0f;
            pair.WasActivated = 0;
            TimestepContactPairs[pairIndex] = pair;
        }
    }

    private static int FindPairIndex(
        Unity.Collections.NativeList<BodyPair> pairs,
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

    private static int FindPairIndex(
        Unity.Collections.NativeList<ContactConstraint> pairs,
        int bodyA,
        int bodyB)
    {
        int low = 0;
        int high = pairs.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            ContactConstraint candidate = pairs[middle];
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



}
}
