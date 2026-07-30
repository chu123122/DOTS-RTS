using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Diagnostics
{

public enum ContactDiagnosticPairKind : byte
{
    BroadPhaseRejected,
    Regular,
    Predictive,
    PredictiveDisabled
}

/// <summary>
/// 中键选择只写入该诊断状态，不改变游戏使用的 UnitSelected。
/// </summary>
public struct ContactDiagnosticSelection : IComponentData
{
#if RTS_CONTACT_DIAGNOSTICS
    public Entity SelectedEntity;
#else
    private byte _disabledStorage;
    public Entity SelectedEntity { get => Entity.Null; set { } }
#endif
}

/// <summary>
/// 每轮 XPBD 求解前后的约束残差和本轮投影量。
/// Gameplay-only builds retain a one-byte buffer element so existing job and
/// DynamicBuffer signatures remain source compatible without allocating the
/// full diagnostic record per iteration.
/// </summary>
public struct ContactIterationDiagnostic : IBufferElementData
{
#if RTS_CONTACT_DIAGNOSTICS
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
#else
    private byte _disabledStorage;
    public int SubstepIndex { get => default; set { } }
    public int IterationIndex { get => default; set { } }
    public int ActiveConstraintCount { get => default; set { } }
    public int PredictiveActivatedCount { get => default; set { } }
    public float MaxConstraintViolationBeforeSolve { get => default; set { } }
    public float AverageConstraintViolationBeforeSolve { get => default; set { } }
    public float MaxConstraintViolation { get => default; set { } }
    public float AverageConstraintViolation { get => default; set { } }
    public float MaxRadialPenetration { get => default; set { } }
    public float AverageRadialPenetration { get => default; set { } }
    public float TotalPositionCorrection { get => default; set { } }
    public float MaxPositionCorrection { get => default; set { } }
    public float TotalWallPositionCorrection { get => default; set { } }
    public float MaxWallPositionCorrection { get => default; set { } }
#endif
}

/// <summary>
/// 当前诊断单位在本 timestep ContactSet 中保留的 Pair 及其激活生命周期。
/// </summary>
public struct ContactPairDiagnostic : IBufferElementData
{
#if RTS_CONTACT_DIAGNOSTICS
    public Entity OtherEntity;
    public ContactDiagnosticPairKind Kind;
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
#else
    private byte _disabledStorage;
    public Entity OtherEntity { get => Entity.Null; set { } }
    public ContactDiagnosticPairKind Kind { get => default; set { } }
    public byte WasActivated { get => default; set { } }
    public byte WasAddedByFallback { get => default; set { } }
    public int FirstActivatedSubstep { get => default; set { } }
    public int ActivatedSubstepCount { get => default; set { } }
    public float ClosestTime { get => default; set { } }
    public float MinimumDistance { get => default; set { } }
    public float RadiusSum { get => default; set { } }
    public float OtherRadius { get => default; set { } }
    public float3 OtherStartPosition { get => default; set { } }
    public float3 OtherPredictedPosition { get => default; set { } }
    public float3 SelectedClosestPosition { get => default; set { } }
    public float3 OtherClosestPosition { get => default; set { } }
#endif
}

/// <summary>
/// 当前诊断单位的 substep 求解结果，以及当前 ContactSet 对应的 timestep 轨迹和包围盒。
/// </summary>
public struct SelectedBodyContactDiagnostic : IComponentData
{
#if RTS_CONTACT_DIAGNOSTICS
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
#else
    private byte _disabledStorage;
    public byte IsValid { get => default; set { } }
    public int SubstepIndex { get => default; set { } }
    public float Radius { get => default; set { } }
    public float Skin { get => default; set { } }
    public float3 StartPosition { get => default; set { } }
    public float3 UnconstrainedPredictedPosition { get => default; set { } }
    public float3 SolvedPosition { get => default; set { } }
    public float3 ContactCorrection { get => default; set { } }
    public float3 WallCorrection { get => default; set { } }
    public float3 VelocityBeforeContact { get => default; set { } }
    public float3 VelocityAfterContact { get => default; set { } }
    public float3 TimestepStartPosition { get => default; set { } }
    public float3 TimestepPredictedPosition { get => default; set { } }
    public float2 TimestepEnvelopeMin { get => default; set { } }
    public float2 TimestepEnvelopeMax { get => default; set { } }
    public byte TimestepEscaped { get => default; set { } }
    public float3 TimestepContactCorrection { get => default; set { } }
    public float3 TimestepWallCorrection { get => default; set { } }
    public byte ShadowReferenceAvailable { get => default; set { } }
    public byte ShadowEscaped { get => default; set { } }
    public float2 ShadowFatMin { get => default; set { } }
    public float2 ShadowFatMax { get => default; set { } }
#endif
}

/// <summary>
/// 常规热力图使用的每单位 timestep 汇总；不携带完整 Pair 列表。
/// </summary>
public struct ContactHeatSample : IBufferElementData
{
#if RTS_CONTACT_DIAGNOSTICS
    public Entity Entity;
    public float3 Position;
    public int ContactPairDegree;
    public int ActivePairDegree;
    public int PredictivePairDegree;
    public float ContactCorrection;
    public byte Escaped;
    public byte HasFallbackPair;
#else
    private byte _disabledStorage;
    public Entity Entity { get => Entity.Null; set { } }
    public float3 Position { get => default; set { } }
    public int ContactPairDegree { get => default; set { } }
    public int ActivePairDegree { get => default; set { } }
    public int PredictivePairDegree { get => default; set { } }
    public float ContactCorrection { get => default; set { } }
    public byte Escaped { get => default; set { } }
    public byte HasFallbackPair { get => default; set { } }
#endif
}

public static class ContactDiagnosticReadback
{
    public static bool Required(UnitContactSolverSettings settings)
    {
#if RTS_CONTACT_DIAGNOSTICS
        return settings.EnableDiagnostics || settings.VisualizeContactHeatmap;
#else
        return false;
#endif
    }
}
}
