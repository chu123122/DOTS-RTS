using Unity.Burst;
using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Jobs;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{

[DisableAutoCreation]
public partial class UnitSpatialPartitionSystem : SystemBase
{
    protected override void OnCreate()
    {
        // 依赖：Grid 配置（用于计算格子索引）。
        RequireForUpdate<FlowFieldSettings>();
    }

    protected override void OnUpdate()
    {
        var manager = World.EntityManager;
        var settings = SystemAPI.GetSingleton<FlowFieldSettings>();
        var query = SystemAPI.QueryBuilder().WithAll<UnitSelected, LocalTransform>().Build(); 
        int unitCount = query.CalculateEntityCount();

        Entity singletonEntity = SystemAPI.GetSingletonEntity<FlowFieldSettings>(); 

        // 还没初始化空间映射组件。
        if (!SystemAPI.HasComponent<UnitSpatialMap>(singletonEntity))
        {
            var map = new NativeParallelMultiHashMap<int, Entity>(unitCount * 2, Allocator.Persistent);
            manager.AddComponentData(singletonEntity, new UnitSpatialMap { Map = map });
        }

        var mapComp = SystemAPI.GetSingletonRW<UnitSpatialMap>();
        if (unitCount > mapComp.ValueRO.Map.Capacity)
        {
            mapComp.ValueRW.Map.Capacity = unitCount * 2;
        }
        Dependency.Complete();
        mapComp.ValueRW.Map.Clear();

        // 调度空间映射 Job。
        var job = new BuildSpatialMapJob
        {
            Map = mapComp.ValueRW.Map.AsParallelWriter(), 
            GridDimensions = settings.GridDimensions,
            GridOrigin = settings.GridOrigin,
            CellRadius = settings.CellRadius
        };
        Dependency = job.ScheduleParallel(Dependency);
    }

    protected override void OnDestroy()
    {
        if (SystemAPI.TryGetSingleton<UnitSpatialMap>(out var mapComp))
            if (mapComp.Map.IsCreated) mapComp.Map.Dispose();
    }
}

[BurstCompile]
public partial struct BuildSpatialMapJob : IJobEntity
{
    public NativeParallelMultiHashMap<int, Entity>.ParallelWriter Map;
    public int2 GridDimensions;
    public float3 GridOrigin;
    public float CellRadius;

    public void Execute(Entity entity, in LocalTransform transform, in BasicUnitTag tag) 
    {
        int2 cellPos = FlowFieldUtils.WorldToCell(transform.Position, GridOrigin, CellRadius);

        if (cellPos.x >= 0 && cellPos.x < GridDimensions.x &&
            cellPos.y >= 0 && cellPos.y < GridDimensions.y)
        {
            int flatIndex = FlowFieldUtils.GetFlatIndex(cellPos, GridDimensions);
            Map.Add(flatIndex, entity);
        }
    }
}
}
