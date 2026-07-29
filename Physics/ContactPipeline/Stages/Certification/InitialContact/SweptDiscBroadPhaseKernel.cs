using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
internal partial struct CertificationStageKernel
{
    private void BuildSweptInteractionPairs(
        ref PredictiveDiscContactStatistics statistics)
    {
        if (FullSweepPrepared.IsCreated && FullSweepPrepared.Value != 0)
        {
            FullSweepPrepared.Value = 0;
            return;
        }

        // Scheduler 必须先完成 FullSweepBroadPhaseStageJobs。这里不再保留
        // 串行 body/cell/pair fallback，缺失前置产物时以空视图显式失败。
        SweptCellEntries.Clear();
        Pairs.Clear();
        TimestepInteractionPairs.Clear();
    }

    private void FilterAndClassifyPairs(
        ref PredictiveDiscContactStatistics statistics,
        float skin)
    {
        int writeIndex = 0;

        for (int readIndex = 0; readIndex < Pairs.Length; readIndex++)
        {
            ContactConstraint pair = Pairs[readIndex];
            CrowdBodySnapshot bodyASnapshot = Bodies[pair.BodyA];
            CrowdNavigationState bodyANavigation = NavigationStates[pair.BodyA];
            CrowdMotionIntent bodyAIntent = MotionIntents[pair.BodyA];
            CrowdMotionEvidence bodyAEvidence = MotionEvidence[pair.BodyA];
            CrowdBodyStepState bodyAStep = StepStates[pair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = Bodies[pair.BodyB];
            CrowdNavigationState bodyBNavigation = NavigationStates[pair.BodyB];
            CrowdMotionIntent bodyBIntent = MotionIntents[pair.BodyB];
            CrowdMotionEvidence bodyBEvidence = MotionEvidence[pair.BodyB];
            CrowdBodyStepState bodyBStep = StepStates[pair.BodyB];
            float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;

            float3 r0 = bodyBEvidence.TrajectoryStart - bodyAEvidence.TrajectoryStart;
            float3 relativeDisplacement =
                (bodyBEvidence.BaselineEnd - bodyBEvidence.TrajectoryStart) -
                (bodyAEvidence.BaselineEnd - bodyAEvidence.TrajectoryStart);
            r0.y = 0;
            relativeDisplacement.y = 0;

            float relativeLengthSq = math.lengthsq(relativeDisplacement);
            float closestTime = relativeLengthSq > 0.0000001f
                ? math.clamp(-math.dot(r0, relativeDisplacement) / relativeLengthSq, 0f, 1f)
                : 0f;
            float minDistanceSq = math.lengthsq(r0 + closestTime * relativeDisplacement);
            float candidateDistance = radiusSum + skin;
            float retainedDistance = candidateDistance +
                                     math.max(0f, Configuration.TimestepContactMargin) * 2f;
            if (minDistanceSq > retainedDistance * retainedDistance)
            {
#if RTS_CONTACT_DIAGNOSTICS
                if (Configuration.EnableDiagnostics)
                {
                    AddSelectedPairDiagnostic(
                        pair,
                        ContactDiagnosticPairKind.BroadPhaseRejected,
                        closestTime,
                        math.sqrt(minDistanceSq),
                        radiusSum,
                        0);
                }
#endif
                continue;
            }

            float startDistanceSq = math.lengthsq(r0);
            float3 endDelta = bodyBEvidence.BaselineEnd - bodyAEvidence.BaselineEnd;
            endDelta.y = 0;
            float endDistanceSq = math.lengthsq(endDelta);
            float radiusSumSq = radiusSum * radiusSum;

            bool isActualGeneratedPair = startDistanceSq <= radiusSumSq;
            if (!isActualGeneratedPair && !Configuration.EnablePredictivePairGeneration)
                continue;

            bool isDormant = minDistanceSq > candidateDistance * candidateDistance;

            if (isActualGeneratedPair)
                statistics.ActualGeneratedPairCount++;
            else if (!isDormant)
                statistics.PredictiveGeneratedPairCount++;

            // 生成来源只用于统计。求解模式仍按原边界：只有起终点均分离、但 swept 路径穿过接触半径的 Pair，才用初始分离平面防止换侧。
            bool shouldPreventSideExchange =
                !isActualGeneratedPair &&
                !isDormant &&
                endDistanceSq >= radiusSumSq &&
                minDistanceSq <= radiusSumSq;

            if (shouldPreventSideExchange)
                statistics.PotentialPredictivePairCount++;

            pair.Lambda = 0f;
            pair.WasActivated = 0;
            pair.WasActivatedThisTimestep = 0;
            pair.WasCorrectedThisTimestep = 0;
            pair.IsDormant = (byte)(isDormant ? 1 : 0);
            pair.WasAddedByFallback = 0;
            pair.FirstActivatedSubstep = -1;
            pair.ActivatedSubstepCount = 0;
            pair.ContactMode = shouldPreventSideExchange && Configuration.EnablePredictiveContacts
                ? ContactConstraintMode.Predictive
                : ContactConstraintMode.Regular;
            float3 predictiveNormal = bodyAEvidence.TrajectoryStart - bodyBEvidence.TrajectoryStart;
            predictiveNormal.y = 0f;
            pair.PredictiveNormal = math.normalizesafe(
                predictiveNormal,
                ContactPipelineMath.DeterministicFallbackNormal(pair.BodyA, pair.BodyB));
            Pairs[writeIndex++] = pair;

            if (isDormant)
                statistics.TimestepContactSetDormantPairCount++;

            if (pair.ContactMode == ContactConstraintMode.Predictive)
                statistics.PredictivePairCount++;
        }

        Pairs.ResizeUninitialized(writeIndex);
        statistics.ContactPairCount += writeIndex;
    }
}
}
