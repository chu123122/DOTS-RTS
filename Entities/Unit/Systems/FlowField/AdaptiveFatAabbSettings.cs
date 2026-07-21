using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace RTS.Unit.FlowField
{
/// <summary>
/// Adaptive Fat AABB 的运行参数。
/// 没有 Authoring 时 BaseFlowMovementSystem 会使用 <see cref="Default"/>，
/// 因而不会要求所有场景都补挂配置组件。
/// </summary>
public struct AdaptiveFatAabbSettings : IComponentData
{
    public byte Enabled;
    public byte DrawDebug;
    public int DetectionCellSpan;
    public int MinimumUnitsPerCell;
    public int MinimumUnitsPerRegion;
    public int HaloCellCount;
    public float EnableScore;
    public float DisableScore;
    public int EnableFrames;
    public int DisableFrames;
    public float DensityWeight;
    public float PersistenceWeight;
    public float PressureWeight;
    public float EscapeRiskWeight;
    public float CorrectionReference;
    public float MaximumCacheableSpeed;
    public float ScoreSmoothing;
    public float DebugHeight;

    public static AdaptiveFatAabbSettings Default => new AdaptiveFatAabbSettings
    {
        Enabled = 1,
        DrawDebug = 0,
        DetectionCellSpan = 3,
        MinimumUnitsPerCell = 6,
        MinimumUnitsPerRegion = 14,
        HaloCellCount = 1,
        EnableScore = 0.68f,
        DisableScore = 0.42f,
        EnableFrames = 4,
        DisableFrames = 12,
        DensityWeight = 0.55f,
        PersistenceWeight = 0.20f,
        PressureWeight = 0.20f,
        EscapeRiskWeight = 0.25f,
        CorrectionReference = 0.08f,
        MaximumCacheableSpeed = 4f,
        ScoreSmoothing = 0.25f,
        DebugHeight = 0.15f
    };

    public AdaptiveFatAabbSettings Sanitized()
    {
        AdaptiveFatAabbSettings value = this;
        value.DetectionCellSpan = math.max(1, value.DetectionCellSpan);
        value.MinimumUnitsPerCell = math.max(1, value.MinimumUnitsPerCell);
        value.MinimumUnitsPerRegion = math.max(1, value.MinimumUnitsPerRegion);
        value.HaloCellCount = math.max(0, value.HaloCellCount);
        value.EnableScore = math.clamp(value.EnableScore, 0f, 1f);
        value.DisableScore = math.clamp(value.DisableScore, 0f, value.EnableScore);
        value.EnableFrames = math.max(1, value.EnableFrames);
        value.DisableFrames = math.max(1, value.DisableFrames);
        value.DensityWeight = math.max(0f, value.DensityWeight);
        value.PersistenceWeight = math.max(0f, value.PersistenceWeight);
        value.PressureWeight = math.max(0f, value.PressureWeight);
        value.EscapeRiskWeight = math.max(0f, value.EscapeRiskWeight);
        value.CorrectionReference = math.max(0.0001f, value.CorrectionReference);
        value.MaximumCacheableSpeed = math.max(0.0001f, value.MaximumCacheableSpeed);
        value.ScoreSmoothing = math.clamp(value.ScoreSmoothing, 0.01f, 1f);
        value.DebugHeight = math.max(0f, value.DebugHeight);
        return value;
    }
}

/// <summary>
/// 可选的场景配置入口。只允许一个实例被当作 singleton 使用。
/// </summary>
public sealed class AdaptiveFatAabbAuthoring : MonoBehaviour
{
    [Header("Adaptive Fat AABB")]
    public bool Enabled = true;
    public bool DrawDebug = true;

    [Header("Hotspot Grid")]
    [Min(1)] public int DetectionCellSpan = 3;
    [Min(1)] public int MinimumUnitsPerCell = 6;
    [Min(1)] public int MinimumUnitsPerRegion = 14;
    [Min(0)] public int HaloCellCount = 1;

    [Header("Confidence")]
    [Range(0f, 1f)] public float EnableScore = 0.68f;
    [Range(0f, 1f)] public float DisableScore = 0.42f;
    [Min(1)] public int EnableFrames = 4;
    [Min(1)] public int DisableFrames = 12;
    [Min(0f)] public float DensityWeight = 0.55f;
    [Min(0f)] public float PersistenceWeight = 0.20f;
    [Min(0f)] public float PressureWeight = 0.20f;
    [Min(0f)] public float EscapeRiskWeight = 0.25f;
    [Min(0.0001f)] public float CorrectionReference = 0.08f;
    [Min(0.0001f)] public float MaximumCacheableSpeed = 4f;
    [Range(0.01f, 1f)] public float ScoreSmoothing = 0.25f;

    [Header("Debug")]
    [Min(0f)] public float DebugHeight = 0.15f;

    private sealed class Baker : Baker<AdaptiveFatAabbAuthoring>
    {
        public override void Bake(AdaptiveFatAabbAuthoring authoring)
        {
            Entity entity = GetEntity(TransformUsageFlags.None);
            AddComponent(entity, new AdaptiveFatAabbSettings
            {
                Enabled = (byte)(authoring.Enabled ? 1 : 0),
                DrawDebug = (byte)(authoring.DrawDebug ? 1 : 0),
                DetectionCellSpan = authoring.DetectionCellSpan,
                MinimumUnitsPerCell = authoring.MinimumUnitsPerCell,
                MinimumUnitsPerRegion = authoring.MinimumUnitsPerRegion,
                HaloCellCount = authoring.HaloCellCount,
                EnableScore = authoring.EnableScore,
                DisableScore = authoring.DisableScore,
                EnableFrames = authoring.EnableFrames,
                DisableFrames = authoring.DisableFrames,
                DensityWeight = authoring.DensityWeight,
                PersistenceWeight = authoring.PersistenceWeight,
                PressureWeight = authoring.PressureWeight,
                EscapeRiskWeight = authoring.EscapeRiskWeight,
                CorrectionReference = authoring.CorrectionReference,
                MaximumCacheableSpeed = authoring.MaximumCacheableSpeed,
                ScoreSmoothing = authoring.ScoreSmoothing,
                DebugHeight = authoring.DebugHeight
            }.Sanitized());
        }
    }
}
}
