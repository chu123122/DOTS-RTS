using Entities.Unit.System.FlowFieldSystem;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

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

    [Header("Unit Contact XPBD")]
    [Min(1)] public int contactSubsteps = 2;
    [Min(1)] public int contactIterations = 4;
    [Min(0f)] public float contactCompliance;
    [Min(0f)] public float predictiveContactSkin = 0.05f;
    [Tooltip("关闭时仍生成 swept candidate，但不会启用防换侧 Predictive 约束。")]
    public bool enablePredictiveContacts = true;

    [Header("Stage 3 Contact Diagnostic")]
    [Tooltip("开启逐 iteration 残差、位置修正、速度变化和选中单位 Pair 采集。")]
    public bool enableContactDiagnostics;
    [Tooltip("显示中键选中单位的 swept capsule、AABB 和候选 Pair。")]
    public bool visualizeSelectedContacts = true;

    [Header("Shadow Neighbor Cache Test")]
    [Tooltip("只旁路评估 Fat Swept AABB 邻居表；不会替换当前权威 Broad/Narrow Phase。")]
    public bool enableShadowNeighborCacheTest;
    [Min(0f)] public float shadowCacheMargin = 0.25f;

    public class Baker : Baker<FlowFieldManagerAuthoring>
    {
        public override void Bake(FlowFieldManagerAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);

            AddComponent(entity, new FlowFieldSettings
            {
                GridDimensions = authoring.gridSize,
                CellRadius = authoring.cellRadius,
                GridOrigin = authoring.gridOrigin
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
                EnablePredictiveContacts = authoring.enablePredictiveContacts,
                EnableDiagnostics = authoring.enableContactDiagnostics,
                VisualizeSelectedContacts = authoring.visualizeSelectedContacts,
                EnableShadowNeighborCacheTest = authoring.enableShadowNeighborCacheTest,
                ShadowCacheMargin = math.max(0f, authoring.shadowCacheMargin)
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
