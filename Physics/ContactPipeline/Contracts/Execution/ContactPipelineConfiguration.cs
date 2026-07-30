using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Gameplay Adapter 在 step 开始时冻结的纯 Crowd Physics 配置。
/// 执行期间任何 UI 或 ECS 设置变化都不会影响当前 step。
/// </summary>
public struct CrowdPhysicsSettings
{
    public uint ObstacleVersion;
    public int SubstepCount;
    public int IterationCount;
    public ContactPositionSolverMode ContactPositionSolver;
    public float Compliance;
    public float PredictiveSkin;
    public float SoftAvoidanceResponseRate;
    public float SoftAvoidanceShell;
    public float SettledSoftAvoidanceMultiplier;
    public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
    public float RvoTimeHorizon;
    public bool EnablePredictivePairGeneration;
    public bool EnablePredictiveContacts;
    public bool EnableDiagnostics;
    public bool EnablePersistentContactCache;
    public bool EnableTimestepContactSetCache;
    public float GuardEnvelopeMargin;
    public float TimestepContactMargin;
}

/// <summary>
/// 单次接触管线调用的不可变配置快照。遗留名称在 BaseFlowMovementSystem 边界处翻译；
/// 生产模块只消费这个同步快照。
/// </summary>
public struct ContactPipelineConfiguration
{
    // 身份属于当前仿真步，而非持久缓存寿命。
    public ulong WorldId;
    public uint SimulationStepId;
    public uint ObstacleVersion;

    public float DeltaTime;
    public int SubstepCount;
    public int IterationCount;
    public ContactPositionSolverMode ContactPositionSolver;
    public float Compliance;
    public float PredictiveSkin;
    public float SoftAvoidanceResponseRate;
    public float SoftAvoidanceShell;
    public float SettledSoftAvoidanceMultiplier;
    public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
    public float RvoTimeHorizon;
    public bool EnablePredictivePairGeneration;
    public bool EnablePredictiveContacts;
#if RTS_CONTACT_DIAGNOSTICS
    public bool EnableDiagnostics;
#else
    // 刻意用属性而非 false 字段：Burst 独立编译每个 Job，无法假定所有配置都来自 Create()。
    // 常量 getter 让每个诊断判断成为编译期分支。
    public bool EnableDiagnostics
    {
        get => false;
        set { }
    }
#endif
    public bool EnablePersistentContactCache;
    public bool EnableTimestepContactSetCache;
    public float GuardEnvelopeMargin;
    public float TimestepContactMargin;

    /// <summary>
    /// 作为认证证据之一的输入指纹。单独不够：签发证书前仍需校验实体映射与 guard 包含关系。
    /// </summary>
    public uint CalculateCertificationFingerprint()
    {
        uint flags = 0u;
        flags |= EnablePredictivePairGeneration ? 1u << 0 : 0u;
        flags |= EnablePredictiveContacts ? 1u << 1 : 0u;
        flags |= EnablePersistentContactCache ? 1u << 2 : 0u;
        flags |= EnableTimestepContactSetCache ? 1u << 3 : 0u;
        flags |= (uint)ContactPositionSolver << 8;
        flags |= (uint)SoftAvoidanceVelocitySolver << 16;

        uint first = math.hash(new uint4(
            math.asuint(DeltaTime),
            (uint)math.max(1, SubstepCount),
            (uint)math.max(1, IterationCount),
            math.asuint(Compliance)));
        uint second = math.hash(new uint4(
            math.asuint(PredictiveSkin),
            math.asuint(GuardEnvelopeMargin),
            math.asuint(TimestepContactMargin),
            math.asuint(SoftAvoidanceShell)));
        uint third = math.hash(new uint4(
            math.asuint(SoftAvoidanceResponseRate),
            math.asuint(SettledSoftAvoidanceMultiplier),
            math.asuint(RvoTimeHorizon),
            flags));
        return math.hash(new uint4(first, second, third, ObstacleVersion));
    }

    public static ContactPipelineConfiguration Create(
        ulong worldId,
        uint simulationStepId,
        float deltaTime,
        CrowdPhysicsSettings settings)
    {
        return new ContactPipelineConfiguration
        {
            WorldId = worldId,
            SimulationStepId = simulationStepId,
            ObstacleVersion = settings.ObstacleVersion,
            DeltaTime = deltaTime,
            SubstepCount = settings.SubstepCount,
            IterationCount = settings.IterationCount,
            ContactPositionSolver = settings.ContactPositionSolver,
            Compliance = settings.Compliance,
            PredictiveSkin = settings.PredictiveSkin,
            SoftAvoidanceResponseRate = settings.SoftAvoidanceResponseRate,
            SoftAvoidanceShell = settings.SoftAvoidanceShell,
            SettledSoftAvoidanceMultiplier =
                settings.SettledSoftAvoidanceMultiplier,
            SoftAvoidanceVelocitySolver =
                settings.SoftAvoidanceVelocitySolver,
            RvoTimeHorizon = settings.RvoTimeHorizon,
            EnablePredictivePairGeneration =
                settings.EnablePredictivePairGeneration,
            EnablePredictiveContacts = settings.EnablePredictiveContacts,
            EnableDiagnostics =
#if RTS_CONTACT_DIAGNOSTICS
                settings.EnableDiagnostics,
#else
                false,
#endif
            EnablePersistentContactCache =
                settings.EnablePersistentContactCache,
            EnableTimestepContactSetCache =
                settings.EnableTimestepContactSetCache,
            GuardEnvelopeMargin = settings.GuardEnvelopeMargin,
            TimestepContactMargin = settings.TimestepContactMargin
        };
    }
}
}
