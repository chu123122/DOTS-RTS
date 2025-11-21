using _RePlaySystem.Base;
using Unity.Entities;
using Unity.Transforms;

[WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation)]
public partial class LocalUnitFlowMovementSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireForUpdate<FlowFieldGrid>();
        RequireForUpdate<UnitSpatialMap>();
        RequireForUpdate<LocalInstance>();
    }

    protected override void OnUpdate()
    {
        var gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();
        var spatialMap = SystemAPI.GetSingleton<UnitSpatialMap>();
        
        if (!gridComponent.Grid.IsCreated) return;

        // 1. 创建 Lookup 用于查询邻居位置
        var transformLookup = SystemAPI.GetComponentLookup<LocalTransform>(isReadOnly: true);
        transformLookup.Update(this);

        var moveJob = new MoveAlongFlowFieldJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            
            SpatialMap = spatialMap.Map,
            TransformLookup = transformLookup, // 传入 Lookup
            
            SeparationWeight = 5.0f, // 调大一点能明显看到排斥
            SeparationRadius = 1.0f  // 感知半径
        };

        Dependency = moveJob.ScheduleParallel(Dependency);
    }
}