using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Jobs;
using RTS.Unit.FlowField.Systems;

namespace RTS.Unit.FlowField.Diagnostics
{

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
/// 每轮 XPBD 求解前后的约束残差和本轮投影量。
/// </summary>
public struct Stage3ContactIterationDiagnostic : IBufferElementData
{
    public int SubstepIndex;
    public int IterationIndex;
    public int ActiveConstraintCount;
    public int PredictiveActivatedCount;
    public float MaxConstraintViolationBeforeSolve;
    public float AverageConstraintViolationBeforeSolve;
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
/// 当前诊断单位在本 timestep ContactSet 中保留的 Pair 及其激活生命周期。
/// </summary>
public struct Stage3ContactPairDiagnostic : IBufferElementData
{
    public Entity OtherEntity;
    public Stage3ContactDiagnosticPairKind Kind;
    public byte WasActivated;
    public byte WasAddedByFallback;
    public int FirstActivatedSubstep;
    public int ActivatedSubstepCount;
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
/// 当前诊断单位的 substep 求解结果，以及当前 ContactSet 对应的 timestep 轨迹和包围盒。
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
    public float3 TimestepStartPosition;
    public float3 TimestepPredictedPosition;
    public float2 TimestepEnvelopeMin;
    public float2 TimestepEnvelopeMax;
    public byte TimestepEscaped;
    public float3 TimestepContactCorrection;
    public float3 TimestepWallCorrection;
    public byte ShadowReferenceAvailable;
    public byte ShadowEscaped;
    public float2 ShadowFatMin;
    public float2 ShadowFatMax;
}

/// <summary>
/// 常规热力图使用的每单位 timestep 汇总；不携带完整 Pair 列表。
/// </summary>
public struct Stage3ContactHeatSample : IBufferElementData
{
    public Entity Entity;
    public float3 Position;
    public int ContactPairDegree;
    public int ActivePairDegree;
    public int PredictivePairDegree;
    public float ContactCorrection;
    public byte Escaped;
    public byte HasFallbackPair;
}

public static class Stage3ContactDiagnosticReadback
{
    public static bool Required(UnitContactSolverSettings settings)
    {
        return settings.EnableDiagnostics || settings.VisualizeContactHeatmap;
    }
}
}
