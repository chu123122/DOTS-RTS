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
        // 1. 管理 Map 的生命周期 (创建/扩容)
        var manager = World.EntityManager;
        var settings = SystemAPI.GetSingleton<FlowFieldSettings>();
        var query = SystemAPI.QueryBuilder().WithAll<UnitSelected, LocalTransform>().Build(); // 假设你的单位都有 UnitSelected 或其他 Tag
        int unitCount = query.CalculateEntityCount();

        Entity singletonEntity = SystemAPI.GetSingletonEntity<FlowFieldSettings>(); // 我们把 Map 挂在 Settings 的实体上

        // 如果还没初始化 Map 组件
        if (!SystemAPI.HasComponent<UnitSpatialMap>(singletonEntity))
        {
            var map = new NativeParallelMultiHashMap<int, Entity>(unitCount * 2, Allocator.Persistent);
            manager.AddComponentData(singletonEntity, new UnitSpatialMap { Map = map });
        }

        var mapComp = SystemAPI.GetSingletonRW<UnitSpatialMap>();
        
        // 动态扩容检查 (如果单位数量激增)
        if (unitCount > mapComp.ValueRO.Map.Capacity)
        {
            mapComp.ValueRW.Map.Capacity = unitCount * 2;
        }
        Dependency.Complete();
        // 2. 清空上一帧的数据
        mapComp.ValueRW.Map.Clear();

        // 3. 调度并行 Job：将所有单位填入 Map
        var job = new BuildSpatialMapJob
        {
            Map = mapComp.ValueRW.Map.AsParallelWriter(), // 并行写入器
            GridDimensions = settings.GridDimensions,
            GridOrigin = settings.GridOrigin,
            CellRadius = settings.CellRadius
        };

        // ScheduleParallel 自动分配
        Dependency = job.ScheduleParallel(Dependency);
    }

    protected override void OnDestroy()
    {
        // 别忘了释放内存！
        if (SystemAPI.TryGetSingleton<UnitSpatialMap>(out var mapComp))
        {
            if (mapComp.Map.IsCreated) mapComp.Map.Dispose();
        }
    }
}

[BurstCompile]
public partial struct BuildSpatialMapJob : IJobEntity
{
    // 并行写入必须用 ParallelWriter
    public NativeParallelMultiHashMap<int, Entity>.ParallelWriter Map;
    public int2 GridDimensions;
    public float3 GridOrigin;
    public float CellRadius;

    // 只查询基本的单位组件
    public void Execute(Entity entity, in LocalTransform transform, in BasicUnitTag tag) // 确保你有 BasicUnitTag 或类似的标识
    {
        // 算出单位在哪个格子
        int2 cellPos = FlowFieldUtils.WorldToCell(transform.Position, GridOrigin, CellRadius);

        // 边界检查
        if (cellPos.x >= 0 && cellPos.x < GridDimensions.x &&
            cellPos.y >= 0 && cellPos.y < GridDimensions.y)
        {
            int flatIndex = FlowFieldUtils.GetFlatIndex(cellPos, GridDimensions);
            // 写入：Key = 格子索引, Value = Entity ID
            Map.Add(flatIndex, entity);
        }
    }
}