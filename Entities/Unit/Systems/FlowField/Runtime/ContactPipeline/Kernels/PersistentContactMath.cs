using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 持久（P1P6）接触分类路径的纯值辅助：约束装配、统计累加、活动表盘追踪、对轨迹查询、相邻对去重。
/// 不含 Job 状态、无实例字段，全部输入以参数传入。
/// </summary>
internal static class PersistentContactMath
{
    /// <summary>
    /// 由缓存预测接触的稳定法线与模式构造一条确定性（较小 body 索引在前）接触约束。
    /// </summary>
    internal static ContactConstraint BuildConstraintFromPersistentContact(
        int firstBodyIndex,
        int secondBodyIndex,
        PersistentPredictiveContact contact) =>
        new ContactConstraint
        {
            BodyA = math.min(firstBodyIndex, secondBodyIndex),
            BodyB = math.max(firstBodyIndex, secondBodyIndex),
            PredictiveNormal = contact.StableNormal,
            ContactMode = contact.ContactMode,
            FirstActivatedSubstep = -1
        };

    /// <summary>
    /// 将一条持久接触的生命周期累加进面向求解器的统计计数器（actual / predictive / approaching / dormant）。
    /// </summary>
    internal static void AccumulateClassificationStatistics(
        PersistentPredictiveContact contact,
        ref PredictiveDiscContactStatistics statistics)
    {
        switch (contact.Lifecycle)
        {
            case PersistentContactLifecycle.Actual:
                statistics.ActualGeneratedPairCount++;
                break;
            case PersistentContactLifecycle.Predictive:
                statistics.PredictiveGeneratedPairCount++;
                statistics.PredictivePairCount++;
                statistics.PotentialPredictivePairCount++;
                break;
            case PersistentContactLifecycle.Approaching:
                statistics.PredictiveGeneratedPairCount++;
                break;
            case PersistentContactLifecycle.Dormant:
                statistics.TimestepContactSetDormantPairCount++;
                break;
        }
    }

    /// <summary>
    /// 更新当前活动约束数及其运行峰值。
    /// </summary>
    internal static void UpdateActiveConstraintGauges(
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int currentActiveConstraintCount)
    {
        incrementalStatistics.CurrentActiveConstraintCount =
            math.max(0, currentActiveConstraintCount);
        incrementalStatistics.PeakActiveConstraintCount = math.max(
            incrementalStatistics.PeakActiveConstraintCount,
            incrementalStatistics.CurrentActiveConstraintCount);
    }

    /// <summary>
    /// 由预测接触 scratch 重算各生命周期的当前接触表盘（actual / predictive / approaching / dormant），
    /// 并更新活动约束表盘。dormant/approaching/predictive/actual 计数含义同 AccumulateClassificationStatistics，
    /// 但反映当前视图而非累计生成计数。
    /// </summary>
    internal static void RefreshCurrentContactStateGauges(
        NativeList<PersistentPredictiveContact> predictiveContactScratch,
        ref IncrementalContactPipelineStatistics incrementalStatistics,
        int currentActiveConstraintCount)
    {
        incrementalStatistics.CurrentSweptContactCount =
            predictiveContactScratch.Length;
        incrementalStatistics.CurrentDormantPairCount = 0;
        incrementalStatistics.CurrentApproachingPairCount = 0;
        incrementalStatistics.CurrentPredictivePairCount = 0;
        incrementalStatistics.CurrentActualPairCount = 0;

        for (int contactIndex = 0;
             contactIndex < predictiveContactScratch.Length;
             contactIndex++)
        {
            switch (predictiveContactScratch[contactIndex].Lifecycle)
            {
                case PersistentContactLifecycle.Dormant:
                    incrementalStatistics.CurrentDormantPairCount++;
                    break;
                case PersistentContactLifecycle.Approaching:
                    incrementalStatistics.CurrentApproachingPairCount++;
                    break;
                case PersistentContactLifecycle.Predictive:
                    incrementalStatistics.CurrentPredictivePairCount++;
                    break;
                case PersistentContactLifecycle.Actual:
                    incrementalStatistics.CurrentActualPairCount++;
                    break;
            }
        }

        UpdateActiveConstraintGauges(
            ref incrementalStatistics,
            currentActiveConstraintCount);
    }

    /// <summary>
    /// 两 swept-disc body 投影至 xz 平面后的最近接近时刻参数 t∈[0,1]。
    /// 相对位移可忽略（平行/静止）时返回 0。
    /// </summary>
    internal static float CalculatePairClosestTime(
        CrowdMotionEvidence bodyAEvidence,
        CrowdMotionEvidence bodyBEvidence)
    {
        float3 relativeStart =
            bodyBEvidence.TrajectoryStart - bodyAEvidence.TrajectoryStart;
        float3 relativeDisplacement =
            (bodyBEvidence.BaselineEnd - bodyBEvidence.TrajectoryStart) -
            (bodyAEvidence.BaselineEnd - bodyAEvidence.TrajectoryStart);
        relativeStart.y = 0f;
        relativeDisplacement.y = 0f;
        float relativeLengthSq = math.lengthsq(relativeDisplacement);
        return relativeLengthSq > 0.0000001f
            ? math.clamp(
                -math.dot(relativeStart, relativeDisplacement) / relativeLengthSq,
                0f,
                1f)
            : 0f;
    }

    /// <summary>
    /// 两个 body 在 timestep 内是否于 xz 平面上存在相对位移（超过 epsilon）。用来跳过静止对的最近时刻计算。
    /// </summary>
    internal static bool HasRelativeTimestepTrajectory(
        CrowdMotionEvidence bodyAEvidence,
        CrowdMotionEvidence bodyBEvidence)
    {
        float3 relativeDisplacement =
            (bodyBEvidence.BaselineEnd - bodyBEvidence.TrajectoryStart) -
            (bodyAEvidence.BaselineEnd - bodyAEvidence.TrajectoryStart);
        relativeDisplacement.y = 0f;
        return math.lengthsq(relativeDisplacement) > 0.0000001f;
    }

    /// <summary>
    /// 对持久相邻对列表就地排序并按 key 去重。按稳定 key 排序，使拓扑 diff 确定。
    /// </summary>
    internal static void SortAndDeduplicatePersistentNeighborPairs(
        NativeList<PersistentNeighborPair> pairs)
    {
        if (pairs.Length <= 1)
            return;

        pairs.AsArray().Sort(new PersistentNeighborPairComparer());
        int writeIndex = 1;
        PersistentNeighborPair previous = pairs[0];
        for (int readIndex = 1; readIndex < pairs.Length; readIndex++)
        {
            PersistentNeighborPair current = pairs[readIndex];
            if (current.Key.Equals(previous.Key))
                continue;
            pairs[writeIndex++] = current;
            previous = current;
        }
        pairs.ResizeUninitialized(writeIndex);
    }

    /// <summary>
    /// body 增量 proxy 的紧致 swept 边界：路径 AABB（轨迹起止与已解/未约束位置），
    /// 再按接触皮、双 timestep 余量、软避让壳的一半（取最大者）膨胀。
    /// </summary>
    internal static void CalculateIncrementalTightSweptBounds(
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        float predictiveSkin,
        float timestepContactMargin,
        float softAvoidanceShell,
        float rvoTimeHorizon,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        out float2 tightMin,
        out float2 tightMax)
    {
        float contactPadding = math.max(0f, predictiveSkin) +
                               math.max(0f, timestepContactMargin) * 2f;
        float avoidancePadding = math.max(0f, softAvoidanceShell) * 0.5f;
        float extent = math.max(0f, stateSnapshot.Radius) +
                       math.max(contactPadding, avoidancePadding);
        CalculateNeighborPathBounds(
            stateEvidence, stateStep, softSolverMode, softAvoidanceShell, rvoTimeHorizon,
            out float2 pathMin, out float2 pathMax);
        tightMin = pathMin - extent;
        tightMax = pathMax + extent;
    }

    /// <summary>
    /// body 增量 proxy 的校验边界：仅按当前接触/避让外扩的路径 AABB。
    /// 已存储的交互包络已含留存接触预算，校验不得重复应用，否则每个未变 proxy 都会被误判为逃逸。
    /// </summary>
    internal static void CalculateIncrementalValidationBounds(
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        float predictiveSkin,
        float timestepContactMargin,
        float softAvoidanceShell,
        out float2 validationMin,
        out float2 validationMax)
    {
        float contactPadding = math.max(0f, predictiveSkin) +
                               math.max(0f, timestepContactMargin);
        float avoidancePadding = math.max(0f, softAvoidanceShell) * 0.5f;
        float extent = math.max(0f, stateSnapshot.Radius) +
                       math.max(contactPadding, avoidancePadding);
        // 校验边界不外推 RVO 视域：交互包络在构建时已固定。这里传非 RVO 模式，绕过视域投影。
        CalculateNeighborPathBounds(
            stateEvidence, stateStep, SoftAvoidanceVelocitySolverMode.SurfaceVelocityBuffer, 0f, 0f,
            out float2 pathMin, out float2 pathMax);
        validationMin = pathMin - extent;
        validationMax = pathMax + extent;
    }

    private static void CalculateNeighborPathBounds(
        CrowdMotionEvidence evidence,
        CrowdBodyStepState step,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float softAvoidanceShell,
        float rvoTimeHorizon,
        out float2 pathMin,
        out float2 pathMax)
    {
        pathMin = math.min(
            evidence.TrajectoryStart.xz,
            math.min(
                evidence.BaselineEnd.xz,
                math.min(step.UnconstrainedPosition.xz, step.SolvedPosition.xz)));
        pathMax = math.max(
            evidence.TrajectoryStart.xz,
            math.max(
                evidence.BaselineEnd.xz,
                math.max(step.UnconstrainedPosition.xz, step.SolvedPosition.xz)));
        if (softSolverMode !=
                SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle ||
            softAvoidanceShell <= 0f)
            return;

        float2 horizonEnd = step.SolvedPosition.xz +
                            step.BaseVelocity.xz * math.max(0f, rvoTimeHorizon);
        pathMin = math.min(pathMin, horizonEnd);
        pathMax = math.max(pathMax, horizonEnd);
    }
}
}
