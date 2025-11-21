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
        // (边界检查省略...)
        if (cellPos.x < 0 || cellPos.x >= GridDimensions.x || cellPos.y < 0 || cellPos.y >= GridDimensions.y) return;

        int flatIndex = FlowFieldUtils.GetFlatIndex(cellPos, GridDimensions);
        FlowFieldCell cell = Grid[flatIndex];
        
        float3 desiredDir = float3.zero;
        bool isAtDestination = (cell.BestDirectionIndex == 0xFF); // 0xFF 代表到了终点或无效

        if (!isAtDestination)
        {
            int2 dirOffset = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
            desiredDir = math.normalize(new float3(dirOffset.x, 0, dirOffset.y));
        }
        
        // 【关键修改 A】到达终点时的行为差异
        // 如果没到终点：FlowForce 负责驱动
        // 如果到了终点：FlowForce 消失，且我们要施加"刹车"
        float3 moveForce = float3.zero;
        
        if (!isAtDestination)
        {
            moveForce = (desiredDir * speed.Value) - velocity.Value;
        }
        else
        {
            // 到达终点后，期望速度是 0，所以 moveForce 会变成纯粹的"减速力"
            // 但我们不希望它太强硬，给一个阻尼系数
            moveForce = (float3.zero - velocity.Value) * 2.0f; // 2.0f 是刹车强度
        }

        // --- 2. Separation 力 ---
        float3 separationForce = float3.zero;
        int neighborCount = 0;
        
        // ... (这中间的 9宫格 搜索代码保持不变) ...
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
            separationForce /= neighborCount;
            separationForce *= SeparationWeight;
        }

        // --- 3. 合成与应用 (物理层修复) ---
        float3 totalForce = moveForce + separationForce;
        
        // 限制最大力
        float maxForce = settings.MaxForce; 
        if (math.length(totalForce) > maxForce) 
            totalForce = math.normalize(totalForce) * maxForce;

        velocity.Value += totalForce * DeltaTime;
        
        // 【关键修改 B】全局阻尼 (Damping) - 防止溜冰和抖动
        // 每一帧都让速度衰减一点点 (比如 0.95)，模拟空气阻力或地面摩擦
        // 这能有效吸收微小的抖动力
        velocity.Value *= 0.95f;

        // 限制最大速度
        if (math.length(velocity.Value) > speed.Value)
            velocity.Value = math.normalize(velocity.Value) * speed.Value;

        // 【关键修改 C】微小速度过滤 (Deadzone)
        // 如果速度太小（比如只是被轻微挤压），就视为静止，不要更新位置，也不要更新旋转
        if (math.lengthsq(velocity.Value) < 0.01f) 
        {
            velocity.Value = float3.zero;
        }
        else
        {
            transform.Position += velocity.Value * DeltaTime;
        }
        
        // --- 4. 视觉层修复：平滑旋转 (Slerp) ---
        // 只有当速度超过一定阈值才旋转，且使用 Slerp 缓慢转向
        if (math.lengthsq(velocity.Value) > 0.1f) // 阈值调高一点，防止原地抽搐
        {
            quaternion targetRotation = quaternion.LookRotationSafe(math.normalize(velocity.Value), math.up());
            
            // settings.RotationSpeed 建议设为 10.0f - 15.0f
            // Math.slerp 负责平滑插值，不再是瞬间 snap
            transform.Rotation = math.slerp(transform.Rotation, targetRotation, DeltaTime * settings.RotationSpeed);
        }
    }
}