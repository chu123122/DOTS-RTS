using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 持久（P1P6）接触缓存的结构化复用性检查与配置指纹。仅当已提交缓存状态仍与当前 body/求解器配置一致时，
/// 缓存方可增量修补；任何漂移都强制全量重建。纯值函数。
/// </summary>
internal static class PersistentCacheReusability
{
    /// <summary>
    /// 检查时刻所捕捉、决定缓存复用的配置轴快照。
    /// </summary>
    internal struct ConfigurationFingerprint
    {
        public float GuardMargin;
        public float PredictiveSkin;
        public float TimestepContactMargin;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public float RvoTimeHorizon;
        public int SubstepCount;
        public bool PredictivePairGenerationEnabled;
        public bool PredictiveContactsEnabled;
        public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
    }

    /// <summary>
    /// 缓存视图能否就地修补：状态必须有效、容量匹配当前 body 数，且每项配置轴都需匹配上次构建时记录的指纹。
    /// </summary>
    internal static bool IsStructurallyReusable(
        IncrementalContactCacheState state,
        int bodyCount,
        int persistentProxyCount,
        int proxyIndexByBodyCount,
        ConfigurationFingerprint config)
    {
        return state.IsValid != 0 &&
               state.BodyCount == bodyCount &&
               persistentProxyCount == bodyCount &&
               proxyIndexByBodyCount == bodyCount &&
               state.GuardMargin == math.max(0f, config.GuardMargin) &&
               state.PredictiveSkin == math.max(0f, config.PredictiveSkin) &&
               state.TimestepContactMargin == math.max(0f, config.TimestepContactMargin) &&
               state.SoftAvoidanceShell == math.max(0f, config.SoftAvoidanceShell) &&
               state.SoftAvoidanceResponseRate == math.max(0f, config.SoftAvoidanceResponseRate) &&
               state.RvoTimeHorizon == math.max(0f, config.RvoTimeHorizon) &&
               state.SubstepCount == math.max(1, config.SubstepCount) &&
               state.PredictivePairGenerationEnabled == (byte)(config.PredictivePairGenerationEnabled ? 1 : 0) &&
               state.PredictiveContactsEnabled == (byte)(config.PredictiveContactsEnabled ? 1 : 0) &&
               state.SoftAvoidanceVelocitySolver == (byte)config.SoftAvoidanceVelocitySolver;
    }
}
}
