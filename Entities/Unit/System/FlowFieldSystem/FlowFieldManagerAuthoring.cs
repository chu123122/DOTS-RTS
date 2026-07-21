using Entities.Unit.System.FlowFieldSystem;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;

public class FlowFieldManagerAuthoring : MonoBehaviour
{
    public float cellRadius = 0.5f; 
    public int2 gridSize = new int2(100, 100);
    public float3 gridOrigin;

    [Header("Flow Field Visualization")]
    public bool showGrid = true;
    public bool showCost = true;
    public bool showDirections = true;
    [Range(4, 16)] public int pixelsPerCell = 8;
    [Range(0f, 1f)] public float visualizationOpacity = 0.65f;
    public float visualizationHeightOffset = 0.05f;

    [Header("Soft Avoidance")]
    [Tooltip("软避让速度缓冲的每秒响应率；0 表示不把避让速度写入预测速度。")]
    [FormerlySerializedAs("softAvoidanceWeight")]
    [Min(0f)] public float softAvoidanceResponseRate = 4f;
    [Tooltip("单位表面之间开始软避让的额外距离；实际激活距离为双方半径之和加该值。")]
    [Min(0f)] public float softAvoidanceShell = 0.2f;
    [Min(0f)] public float settledSoftAvoidanceMultiplier = 1.5f;
    [Tooltip("Surface 保留距离缓冲；RVO 使用相对速度和时间窗口求速度修正。")]
    public SoftAvoidanceVelocitySolverMode softAvoidanceVelocitySolver =
        SoftAvoidanceVelocitySolverMode.SurfaceVelocityBuffer;
    [Tooltip("RVO 预测未来碰撞的时间窗口（秒），不等同于 Fat AABB 缓存 TTL。")]
    [Min(0.01f)] public float rvoTimeHorizon = 0.5f;

    [Header("Unit Contact XPBD")]
    [Min(1)] public int contactSubsteps = 2;
    [Min(1)] public int contactIterations = 4;
    [Min(0f)] public float contactCompliance;
    [Min(0f)] public float predictiveContactSkin = 0.05f;
    [Tooltip("关闭时只生成 substep 起点已经接触的实际 Pair，不再使用 swept path 提前生成 Pair。")]
    public bool enablePredictivePairGeneration = true;
    [Tooltip("关闭时仍生成 swept candidate，但不会启用防换侧 Predictive 约束。")]
    public bool enablePredictiveContacts = true;

    [Header("Stage 3 Contact Diagnostic")]
    [Tooltip("开启逐 iteration 残差、位置修正、速度变化和选中单位 Pair 采集。")]
    public bool enableContactDiagnostics;
    [Tooltip("显示中键选中单位的 swept capsule、AABB 和候选 Pair。")]
    public bool visualizeSelectedContacts = true;
    [Tooltip("按 F6 后自动采集并写出 JSON 的持续时间（秒）。")]
    [Min(0.5f)] public float diagnosticCaptureDuration = 10f;
    [Tooltip("JSON 采样间隔（秒），不会逐帧写磁盘。")]
    [Min(0.05f)] public float diagnosticCaptureInterval = 0.1f;

    [Header("Fat AABB Neighbor Cache")]
    [Tooltip("启用后 Fat AABB 缓存会替代重复的 Broad Phase 候选发现；每个子步仍重新执行 Narrow Phase。")]
    [FormerlySerializedAs("enableShadowNeighborCacheTest")]
    public bool enableFatAabbCache;
    [Tooltip("Fat AABB 在圆盘半径和 Predictive Skin 之外保留的复用余量。")]
    [FormerlySerializedAs("shadowCacheMargin")]
    [Min(0f)] public float fatAabbCacheMargin = 0.25f;

    public class Baker : Baker<FlowFieldManagerAuthoring>
    {
        public override void Bake(FlowFieldManagerAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);

            AddComponent(entity, new FlowFieldSettings
            {
                GridDimensions = authoring.gridSize,
                CellRadius = authoring.cellRadius,
                GridOrigin = authoring.gridOrigin,
                SoftAvoidanceResponseRate = math.max(
                    0f,
                    authoring.softAvoidanceResponseRate),
                SoftAvoidanceShell = math.max(0f, authoring.softAvoidanceShell),
                SettledSoftAvoidanceMultiplier = math.max(
                    0f,
                    authoring.settledSoftAvoidanceMultiplier),
                SoftAvoidanceVelocitySolver = authoring.softAvoidanceVelocitySolver,
                RvoTimeHorizon = math.max(0.01f, authoring.rvoTimeHorizon)
            });
            AddComponent(entity, new FlowFieldGlobalTarget { TargetPosition = float3.zero });
            AddComponent(entity, new MoveOrder());
            SetComponentEnabled<MoveOrder>(entity, false);
            AddComponent(entity, new UnitContactSolverSettings
            {
                SubstepCount = math.max(1, authoring.contactSubsteps),
                IterationCount = math.max(1, authoring.contactIterations),
                Compliance = math.max(0f, authoring.contactCompliance),
                PredictiveSkin = math.max(0f, authoring.predictiveContactSkin),
                EnablePredictivePairGeneration = authoring.enablePredictivePairGeneration,
                EnablePredictiveContacts = authoring.enablePredictiveContacts,
                EnableDiagnostics = authoring.enableContactDiagnostics,
                VisualizeSelectedContacts = authoring.visualizeSelectedContacts,
                DiagnosticCaptureDuration = math.max(0.5f, authoring.diagnosticCaptureDuration),
                DiagnosticCaptureInterval = math.max(0.05f, authoring.diagnosticCaptureInterval),
                EnableFatAabbCache = authoring.enableFatAabbCache,
                FatAabbCacheMargin = math.max(0f, authoring.fatAabbCacheMargin)
            });
            AddComponent(entity, new PredictiveDiscContactStatistics());
            AddComponent(entity, new ShadowNeighborCacheStatistics());
            AddComponent(entity, new Stage3ContactDiagnosticSelection
            {
                SelectedEntity = Entity.Null
            });
            AddComponent(entity, new Stage3SelectedBodyDiagnostic());
            AddBuffer<Stage3ContactIterationDiagnostic>(entity);
            AddBuffer<Stage3ContactPairDiagnostic>(entity);
            AddComponent(entity, new FlowFieldRuntimeState());
            AddComponent(entity, new FlowFieldCostState { IsDirty = true });
            AddComponent(entity, new RecalculateFlowFieldTag());
            SetComponentEnabled<RecalculateFlowFieldTag>(entity, false);
            AddComponent(entity, new FlowFieldVisualizationSettings
            {
                Visible = authoring.showGrid,
                ShowCost = authoring.showCost,
                ShowDirections = authoring.showDirections,
                PixelsPerCell = (byte)math.clamp(authoring.pixelsPerCell, 4, 16),
                HeightOffset = authoring.visualizationHeightOffset,
                Opacity = math.saturate(authoring.visualizationOpacity)
            });
        }
    }
}
