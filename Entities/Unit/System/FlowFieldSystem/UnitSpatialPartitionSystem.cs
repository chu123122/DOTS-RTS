using Unity.Burst;
using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Jobs;
using Unity.NetCode; // 必须引用
using 通用; // 引用 FlowFieldUtils 所在的命名空间

[UpdateInGroup(typeof(SimulationSystemGroup), OrderFirst = true)]
public partial class UnitSpatialPartitionSystem : SystemBase
{
    protected override void OnCreate()
    {
        // 依赖设置 (Grid配置用于计算格子索引)
        RequireForUpdate<FlowFieldSettings>();
    }

    protected override void OnUpdate()
    {
        var manager = World.EntityManager;
        var settings = SystemAPI.GetSingleton<FlowFieldSettings>();
        var query = SystemAPI.QueryBuilder().WithAll<UnitSelected, LocalTransform>().Build(); 
        int unitCount = query.CalculateEntityCount();

        Entity singletonEntity = SystemAPI.GetSingletonEntity<FlowFieldSettings>(); 

        // 如果还没初始化Map组件
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

        //调动Job
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