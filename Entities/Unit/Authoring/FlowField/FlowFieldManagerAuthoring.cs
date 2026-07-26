using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Systems;

namespace RTS.Unit.Authoring.FlowField
{

public class FlowFieldManagerAuthoring : MonoBehaviour
{
    [Header("诊断总开关")]
    [Tooltip("开启后 Profiler 计时、统计累积、快照发布、O(N²) Oracle 全部运行；关闭后热点路径零开销。")]
    public bool enableContactDiagnostics;

    [Header("流场网格")]
    [Tooltip("每个格子的边长（世界单位）。")]
    public float cellRadius = 0.5f;
    [Tooltip("网格尺寸（列 × 行）。")]
    public int2 gridSize = new int2(100, 100);
    [Tooltip("网格左下角的世界坐标原点。")]
    public float3 gridOrigin;

    [Header("流场可视化")]
    public bool showGrid = true;
    public bool showCost = true;
    [Tooltip("在格子中心绘制 8 方向最佳路径箭头。")]
    public bool showDirections = true;
    [Range(4, 16)] public int pixelsPerCell = 8;
    [Tooltip("可视化平面的不透明度。")]
    [Range(0f, 1f)] public float visualizationOpacity = 0.65f;
    [Tooltip("可视化平面在世界 Y 轴上的高度偏移。")]
    public float visualizationHeightOffset = 0.05f;

    [Header("软避让")]
    [Tooltip("软避让速度缓冲的每秒响应率；0 表示不把避让速度写入预测速度。")]
    [FormerlySerializedAs("softAvoidanceWeight")]
    [Min(0f)] public float softAvoidanceResponseRate = 4f;
    [Tooltip("单位表面之间开始软避让的额外距离；实际激活距离为双方半径之和加该值。")]
    [Min(0f)] public float softAvoidanceShell = 0.2f;
    [Tooltip("已到达目标单位的软避让强度倍率；>1 时更用力推开周围单位保持间距。")]
    [Min(0f)] public float settledSoftAvoidanceMultiplier = 1.5f;
    [Tooltip("预测引导 = 纯位置排斥；RVO 互惠避让 = 速度感知避障。")]
    public SoftAvoidanceVelocitySolverMode softAvoidanceVelocitySolver =
        SoftAvoidanceVelocitySolverMode.SurfaceVelocityBuffer;
    [Tooltip("RVO 预测未来碰撞的时间窗口（秒），选 RVO 时生效。")]
    [Min(0.01f)] public float rvoTimeHorizon = 0.5f;

    [Header("XPBD 接触求解")]
    [Tooltip("每帧拆分的物理子步数。")]
    [Min(1)] public int contactSubsteps = 2;
    [Tooltip("每个子步内 XPBD 约束迭代次数。")]
    [Min(1)] public int contactIterations = 4;
    [Tooltip("XPBD 柔度参数；0 = 硬约束，越大越软。")]
    [Min(0f)] public float contactCompliance;
    [Tooltip("接触位置求解器：Gauss-Seidel = 串行逐对修正收敛快；Jacobi = 并行批量修正吞吐高。")]
    public ContactPositionSolverMode contactPositionSolver = ContactPositionSolverMode.GaussSeidel;
    [Tooltip("预测碰撞检测时在圆盘半径外的膨胀厚度。")]
    [Min(0f)] public float predictiveContactSkin = 0.05f;
    [Tooltip("关闭时只生成子步起点已实际接触的 Pair，不沿 swept path 提前发现潜在碰撞。")]
    public bool enablePredictivePairGeneration = true;
    [Tooltip("关闭时仍生成 swept candidate Pair，但使用实时法线而非预计算的轨迹法线求解。")]
    public bool enablePredictiveContacts = true;

    [Header("接触缓存")]
    [Tooltip("在一个 timestep 的全部子步间复用中层 InteractionSet 接触视图。")]
    public bool enableTimestepContactSetCache = true;
    [Tooltip("启用后跨帧持久邻居拓扑复用 guarded proxy 证书，避免每帧全量候选发现。")]
    [FormerlySerializedAs("enableShadowNeighborCacheTest")]
    [FormerlySerializedAs("enableFatAabbCache")]
    public bool enablePersistentContactCache;
    [Tooltip("持久 Guard Envelope 在圆盘半径和 Predictive Skin 之外保留的复用余量。")]
    [FormerlySerializedAs("shadowCacheMargin")]
    [FormerlySerializedAs("fatAabbCacheMargin")]
    [Min(0f)] public float persistentGuardEnvelopeMargin = 0.25f;
    [Tooltip("整个 timestep ContactSet 预测轨迹之外的安全边界；逃出后执行完整回退重建。")]
    [Min(0f)] public float timestepContactMargin = 0.25f;

    [Header("调试显示")]
    [Tooltip("显示中键选中单位的 swept capsule、AABB 和候选 Pair。")]
    public bool visualizeSelectedContacts = true;
    [Tooltip("常规视图显示接触负载热力图；中键选中后仍显示单位细节。")]
    public bool visualizeContactHeatmap = true;
    [Tooltip("热力图着色维度。")]
    public ContactHeatmapMode contactHeatmapMode = ContactHeatmapMode.ContactLoad;
    [Tooltip("中键选中诊断单位时使用的默认时间倍率。")]
    [Range(0.025f, 1f)] public float diagnosticSlowMotionScale = 0.2f;
    [Tooltip("按 F6 后自动采集并写出 JSON 的持续时间（秒）。")]
    [Min(0.5f)] public float diagnosticCaptureDuration = 10f;
    [Tooltip("JSON 采样间隔（秒），不会逐帧写磁盘。")]
    [Min(0.05f)] public float diagnosticCaptureInterval = 0.1f;

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
            AddBuffer<MoveOrderSelectionElement>(entity);
            SetComponentEnabled<MoveOrder>(entity, false);
            AddComponent(entity, new UnitContactSolverSettings
            {
                SubstepCount = math.max(1, authoring.contactSubsteps),
                IterationCount = math.max(1, authoring.contactIterations),
                ContactPositionSolver = authoring.contactPositionSolver,
                Compliance = math.max(0f, authoring.contactCompliance),
                PredictiveSkin = math.max(0f, authoring.predictiveContactSkin),
                EnablePredictivePairGeneration = authoring.enablePredictivePairGeneration,
                EnablePredictiveContacts = authoring.enablePredictiveContacts,
                EnableDiagnostics = authoring.enableContactDiagnostics,
                VisualizeSelectedContacts = authoring.visualizeSelectedContacts,
                DiagnosticCaptureDuration = math.max(0.5f, authoring.diagnosticCaptureDuration),
                DiagnosticCaptureInterval = math.max(0.05f, authoring.diagnosticCaptureInterval),
                EnableTimestepContactSetCache =
                    authoring.enableTimestepContactSetCache,
                EnablePersistentContactCache = authoring.enablePersistentContactCache,
                PersistentGuardEnvelopeMargin = math.max(0f, authoring.persistentGuardEnvelopeMargin),
                TimestepContactMargin = math.max(0f, authoring.timestepContactMargin),
                VisualizeContactHeatmap = authoring.visualizeContactHeatmap,
                ContactHeatmapMode = authoring.contactHeatmapMode,
                DiagnosticSlowMotionScale = math.clamp(
                    authoring.diagnosticSlowMotionScale,
                    0.025f,
                    1f)
            });
            AddComponent(entity, new FlowFieldRuntimeState());
            AddComponent(entity, new FlowFieldCostState { IsDirty = true });
            // 启动时先烘焙一次 Cost/Integration/Vector Field。
            // RtsCommandSystem 会保留首条 MoveOrder，直到 ActiveVersion 发布，
            // 避免 FlowFieldGrid 尚未创建时形成循环等待。
            AddComponent(entity, new RecalculateFlowFieldTag { RequestVersion = 1 });
            SetComponentEnabled<RecalculateFlowFieldTag>(entity, true);
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
}
