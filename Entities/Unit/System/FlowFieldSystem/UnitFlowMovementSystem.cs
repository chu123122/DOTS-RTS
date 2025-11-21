using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Transforms;
using 通用;

[UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
[UpdateAfter(typeof(UnitSpatialPartitionSystem))] // 必须在 Map 构建后运行
public partial class UnitFlowMovementSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireForUpdate<FlowFieldGrid>();
        RequireForUpdate<UnitSpatialMap>();
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

[BurstCompile]
public partial struct MoveAlongFlowFieldJob : IJobEntity
{
    public float DeltaTime;
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    
    // Boids 专用
    [ReadOnly] public NativeParallelMultiHashMap<int, Entity> SpatialMap;
    [ReadOnly][NativeDisableContainerSafetyRestriction] public ComponentLookup<LocalTransform> TransformLookup;
    public float SeparationWeight;
    public float SeparationRadius;

    public void Execute(
        Entity entity, 
        ref LocalTransform transform, 
        ref Velocity velocity, 
        in UnitMoveSpeed speed,
        in UnitMovementSettings settings)
    {
        // --- 1. Flow Field 力 ---
        int2 cellPos = FlowFieldUtils.WorldToCell(transform.Position, GridOrigin, CellRadius);
        // (省略：边界检查代码同上...)
        if (cellPos.x < 0 || cellPos.x >= GridDimensions.x || cellPos.y < 0 || cellPos.y >= GridDimensions.y) return;

        int flatIndex = FlowFieldUtils.GetFlatIndex(cellPos, GridDimensions);
        FlowFieldCell cell = Grid[flatIndex];
        
        float3 desiredDir = float3.zero;
        if (cell.BestDirectionIndex != 0xFF)
        {
            int2 dirOffset = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
            desiredDir = math.normalize(new float3(dirOffset.x, 0, dirOffset.y));
        }
        
        // 基础流场驱动力
        float3 moveForce = (desiredDir * speed.Value) - velocity.Value;

        // --- 2. Separation 力 (关键部分) ---
        float3 separationForce = float3.zero;
        int neighborCount = 0;

        // 搜索 9 宫格
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                int2 checkCell = cellPos + new int2(x, y);
                if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                    checkCell.y < 0 || checkCell.y >= GridDimensions.y) continue;

                int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);

                if (SpatialMap.TryGetFirstValue(checkIndex, out Entity neighborEntity, out var it))
                {
                    do
                    {
                        if (neighborEntity == entity) continue;
                        
                        // 使用 Lookup 获取邻居位置
                        if (!TransformLookup.HasComponent(neighborEntity)) continue;
                        float3 neighborPos = TransformLookup[neighborEntity].Position;

                        float dist = math.distance(transform.Position, neighborPos);
                        
                        // 只有靠得足够近才排斥
                        if (dist < SeparationRadius && dist > 0.001f)
                        {
                            float3 pushDir = math.normalize(transform.Position - neighborPos);
                            // 距离越近，力越大 (1 / dist)
                            separationForce += pushDir / dist; 
                            neighborCount++;
                        }

                    } while (SpatialMap.TryGetNextValue(out neighborEntity, ref it));
                }
            }
        }

        if (neighborCount > 0)
        {
            separationForce /= neighborCount; // 平均一下
            separationForce *= SeparationWeight;
        }

        // --- 3. 合成与应用 ---
        // 这里的魔法是：FlowField 负责"走"，Separation 负责"推"
        float3 totalForce = moveForce + separationForce;
        
        // 限制最大力
        float maxForce = settings.MaxForce; 
        if (math.length(totalForce) > maxForce) 
            totalForce = math.normalize(totalForce) * maxForce;

        velocity.Value += totalForce * DeltaTime;
        
        // 限制最大速度
        if (math.length(velocity.Value) > speed.Value)
            velocity.Value = math.normalize(velocity.Value) * speed.Value;

        // 应用位移
        transform.Position += velocity.Value * DeltaTime;
        
        // 更新旋转 (只看速度方向)
        if (math.lengthsq(velocity.Value) > 0.01f)
            transform.Rotation = quaternion.LookRotationSafe(math.normalize(velocity.Value), math.up());
    }
}