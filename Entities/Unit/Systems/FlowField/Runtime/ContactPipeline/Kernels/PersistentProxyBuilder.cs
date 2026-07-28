using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 增量（P1P6）接触路径下的持久 swept-proxy 构建与 dirty 标记分类。纯值函数：无 Job 状态、无实例字段、
/// 所有输入以参数传入。被认证器 Job 共享，并可独立做单元测试。
/// </summary>
internal static class PersistentProxyBuilder
{
    /// <summary>
    /// 由 body 步状态构建 swept-proxy 包络。proxy 同时携带紧致交互边界与膨胀的 guard 带；
    /// 仅 RVO 求解器模式会外推避让视域终点。
    /// </summary>
    internal static PersistentSweptProxy BuildFromState(
        int bodyIndex,
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        float guardMargin,
        float softAvoidanceShell,
        float softAvoidanceResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon)
    {
        PersistentSweptProxy proxy = new PersistentSweptProxy
        {
            Entity = stateSnapshot.Entity,
            BodyIndex = bodyIndex,
            IsValid = (byte)((stateSnapshot.IsInsideSimulationDomain != 0) ? 1 : 0),
            Radius = math.max(0f, stateSnapshot.Radius)
        };
        if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            return proxy;
        proxy.TightMin = stateEvidence.InteractionEnvelopeMin;
        proxy.TightMax = stateEvidence.InteractionEnvelopeMax;
        proxy.GuardMin = proxy.TightMin - math.max(0f, guardMargin);
        proxy.GuardMax = proxy.TightMax + math.max(0f, guardMargin);
        proxy.TrajectoryStart = stateEvidence.TrajectoryStart.xz;
        proxy.TrajectoryEnd = stateEvidence.BaselineEnd.xz;
        // 位置圆 topology guard：仅基于当前帧位置+半径，与速度方向无关。
        // 必须在 proxy.TrajectoryStart 赋值之后计算。
        // 转向/加速只改变 InteractionEnvelope（TightMin/Max），不改变位置，
        // 因此不会触发 topology dirty，只有物理位移超出此边界才重建邻居拓扑。
        float topologyHalfExtent = math.max(0f, proxy.Radius) + math.max(0f, guardMargin);
        proxy.TopologyGuardMin = proxy.TrajectoryStart - topologyHalfExtent;
        proxy.TopologyGuardMax = proxy.TrajectoryStart + topologyHalfExtent;
        proxy.AvoidanceHorizonEnd =
            softSolverMode == SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle &&
            softAvoidanceShell > 0f && softAvoidanceResponseRate > 0f
                ? stateEvidence.TrajectoryStart.xz +
                  stateStep.BaseVelocity.xz * math.max(0f, rvoTimeHorizon)
                : stateEvidence.BaselineEnd.xz;
        proxy.MotionVersion = 1u;
        return proxy;
    }

    /// <summary>
    /// 基于 <paramref name="current"/> 轨迹/半径相较 <paramref name="previous"/> 是否变化，
    /// 为其分配单调的 motion 版本号。稳定轨迹沿用上一版本，运动则递增。
    /// </summary>
    internal static void AssignMotionVersion(
        ref PersistentSweptProxy current,
        PersistentSweptProxy previous)
    {
        bool same = math.all(current.TrajectoryStart == previous.TrajectoryStart) &&
                    math.all(current.TrajectoryEnd == previous.TrajectoryEnd) &&
                    math.all(current.AvoidanceHorizonEnd ==
                             previous.AvoidanceHorizonEnd) &&
                    current.Radius == previous.Radius;
        current.MotionVersion = same
            ? previous.MotionVersion
            : previous.MotionVersion == uint.MaxValue
                ? 1u
                : previous.MotionVersion + 1u;
    }

    /// <summary>
    /// 将 body 当前状态与缓存 proxy 比对，dirty 时把更新后的 proxy 写回持久存储。
    /// 返回 dirty 标记的并集（entity-set / topology / motion）。
    /// 缓存视图在其有效性或容量不一致时被视为不可读。
    /// </summary>
    internal static IncrementalBodyDirtyFlags ClassifyAndUpdateForBody(
        int bodyIndex,
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        NativeArray<PersistentSweptProxy> persistentProxies,
        NativeArray<int> proxyIndexByBody,
        IncrementalContactCacheState cacheState,
        float guardMargin,
        float softAvoidanceShell,
        float softAvoidanceResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon)
    {
        if (cacheState.IsValid == 0 ||
            proxyIndexByBody.Length != cacheState.BodyCount ||
            persistentProxies.Length != cacheState.BodyCount ||
            (uint)bodyIndex >= (uint)proxyIndexByBody.Length)
            return IncrementalBodyDirtyFlags.None;

        int proxyIndex = proxyIndexByBody[bodyIndex];
        if ((uint)proxyIndex >= (uint)persistentProxies.Length)
            return IncrementalBodyDirtyFlags.EntitySet |
                   IncrementalBodyDirtyFlags.Topology |
                   IncrementalBodyDirtyFlags.Motion;

        PersistentSweptProxy previous = persistentProxies[proxyIndex];
        if (previous.Entity != stateSnapshot.Entity)
            return IncrementalBodyDirtyFlags.EntitySet |
                   IncrementalBodyDirtyFlags.Topology |
                   IncrementalBodyDirtyFlags.Motion;

        PersistentSweptProxy current = BuildFromState(
            bodyIndex, stateSnapshot, stateEvidence, stateStep,
            guardMargin, softAvoidanceShell,
            softAvoidanceResponseRate, softSolverMode, rvoTimeHorizon);
        AssignMotionVersion(ref current, previous);
        // Topology dirty 判据：仅看物理位置圆是否逃出上次拓扑构建时的位置圆守护边界。
        // 速度方向变化（转向/加速）只改变 InteractionEnvelope，不改变位置，故不触发 topology rebuild，
        // 避免 AI 导航场景下 dirty 率雪崩（此前以 InteractionEnvelope 做判据时的核心缺陷）。
        float2 posMin = current.TrajectoryStart - current.Radius;
        float2 posMax = current.TrajectoryStart + current.Radius;
        bool topologyDirty = previous.IsValid != current.IsValid ||
                             previous.Radius != current.Radius ||
                             (current.IsValid != 0 && !ContactPipelineShared.AabbContains(
                                 previous.TopologyGuardMin, previous.TopologyGuardMax,
                                 posMin, posMax));
        bool motionDirty = topologyDirty || current.MotionVersion != previous.MotionVersion;
        if (!motionDirty)
            return IncrementalBodyDirtyFlags.None;
        if (!topologyDirty)
        {
            // 位置仍在 topology guard 内——冻结 topology guard，不更新（直到下次拓扑重建）。
            current.TopologyGuardMin = previous.TopologyGuardMin;
            current.TopologyGuardMax = previous.TopologyGuardMax;
            // 滑动 broadphase guard 以跟踪当前 InteractionEnvelope（供邻居对插入使用）。
            // 位移取自 TightMin 平移量；若 envelope 因转向/加速越出平移后的 guard，
            // 直接用当前帧 TightMin/Max 重建 guard——这不是 topology 变化，只是 guard 尺寸刷新。
            float2 displacement = current.TightMin - previous.TightMin;
            current.GuardMin = previous.GuardMin + displacement;
            current.GuardMax = previous.GuardMax + displacement;
            if (!ContactPipelineShared.AabbContains(
                    current.GuardMin, current.GuardMax,
                    current.TightMin, current.TightMax))
            {
                current.GuardMin = current.TightMin - math.max(0f, guardMargin);
                current.GuardMax = current.TightMax + math.max(0f, guardMargin);
            }
        }
        persistentProxies[proxyIndex] = current;
        return topologyDirty
            ? IncrementalBodyDirtyFlags.Motion | IncrementalBodyDirtyFlags.Topology
            : IncrementalBodyDirtyFlags.Motion;
    }
}
}
