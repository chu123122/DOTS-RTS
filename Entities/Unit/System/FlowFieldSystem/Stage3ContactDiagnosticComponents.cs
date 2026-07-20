using Unity.Entities;
using Unity.Mathematics;

public enum Stage3ContactDiagnosticPairKind : byte
{
    BroadPhaseRejected,
    Regular,
    Predictive,
    PredictiveDisabled
}

/// <summary>
/// 中键选择只写入该诊断状态，不改变游戏使用的 UnitSelected。
/// </summary>
public struct Stage3ContactDiagnosticSelection : IComponentData
{
    public Entity SelectedEntity;
}

/// <summary>
/// 每轮 XPBD 结束后的约束残差和本轮投影量。
/// </summary>
public struct Stage3ContactIterationDiagnostic : IBufferElementData
{
    public int SubstepIndex;
    public int IterationIndex;
    public int ActiveConstraintCount;
    public int PredictiveActivatedCount;
    public float MaxConstraintViolation;
    public float AverageConstraintViolation;
    public float MaxRadialPenetration;
    public float AverageRadialPenetration;
    public float TotalPositionCorrection;
    public float MaxPositionCorrection;
    public float TotalWallPositionCorrection;
    public float MaxWallPositionCorrection;
}

/// <summary>
/// 当前诊断单位在最后一个 substep 中遇到的 Broad/Narrow Phase Pair。
/// </summary>
public struct Stage3ContactPairDiagnostic : IBufferElementData
{
    public Entity OtherEntity;
    public Stage3ContactDiagnosticPairKind Kind;
    public byte WasActivated;
    public float ClosestTime;
    public float MinimumDistance;
    public float RadiusSum;
    public float OtherRadius;
    public float3 OtherStartPosition;
    public float3 OtherPredictedPosition;
    public float3 SelectedClosestPosition;
    public float3 OtherClosestPosition;
}

/// <summary>
/// 当前诊断单位最后一个 substep 的轨迹和求解结果。
/// </summary>
public struct Stage3SelectedBodyDiagnostic : IComponentData
{
    public byte IsValid;
    public int SubstepIndex;
    public float Radius;
    public float Skin;
    public float3 StartPosition;
    public float3 UnconstrainedPredictedPosition;
    public float3 SolvedPosition;
    public float3 ContactCorrection;
    public float3 WallCorrection;
    public float3 VelocityBeforeContact;
    public float3 VelocityAfterContact;
}
