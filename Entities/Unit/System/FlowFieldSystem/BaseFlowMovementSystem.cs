using Unity.Burst;
using Unity.Entities;
using Unity.Transforms;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using 通用; 

public abstract partial class BaseFlowMovementSystem : SystemBase
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
            TransformLookup = transformLookup,
            
            // 可以在这里微调分离参数
            SeparationWeight = 7f,
            SeparationRadius = 0.6f 
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

    [ReadOnly] public NativeParallelMultiHashMap<int, Entity> SpatialMap;
    [ReadOnly] [NativeDisableContainerSafetyRestriction]
    public ComponentLookup<LocalTransform> TransformLookup;

    public float SeparationWeight;
    public float SeparationRadius;

   public void Execute(
        Entity entity,
        ref LocalTransform transform,
        ref Velocity velocity,
        in UnitMoveSpeed speed,
        in UnitMovementSettings settings)
    {
        // 1. 环境感知
        int2 cellPos = FlowFieldUtils.WorldToCell(transform.Position, GridOrigin, CellRadius);
        if (cellPos.x < 0 || cellPos.x >= GridDimensions.x || 
            cellPos.y < 0 || cellPos.y >= GridDimensions.y) 
        {
            velocity.Value = float3.zero; 
            return;
        }

        int flatIndex = FlowFieldUtils.GetFlatIndex(cellPos, GridDimensions);
        FlowFieldCell cell = Grid[flatIndex];

        // 缓冲区定义
        int arrivalDistance = 3; 
        float flowWeight = 1.0f;
        
        if (cell.IntegrationValue != ushort.MaxValue && cell.IntegrationValue <= arrivalDistance)
        {
            flowWeight = (float)cell.IntegrationValue / (float)arrivalDistance;
        }

        bool isAtDestination = (cell.BestDirectionIndex == 0xFF) || (cell.IntegrationValue == 0);

        // 2. Move Force (流场力)
        float3 moveForce = float3.zero;
        if (!isAtDestination)
        {
            int2 dirOffset = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
            float3 desiredDir = math.normalize(new float3(dirOffset.x, 0, dirOffset.y));
            moveForce = (desiredDir * speed.Value * flowWeight) - velocity.Value;
        }
        // 【改动 1】到了终点不主动施加反向刹车力 (moveForce)，而是完全交给后面的阻尼控制
        // 这样可以避免刹车力对抗分离力

        // 3. Separation Force (分离力 - 核心升级)
        float3 separationForce = float3.zero;
        int neighborCount = 0;
        bool isOverlapping = false; // 标记：是否发生了严重穿模

        // 假设胶囊体半径是 0.5 (直径1.0)
        // 我们把"硬碰撞"半径设为 0.5 * 2 = 1.0 (即两个圆心距离 1.0 时刚好接触)
        // SeparationRadius 建议设为 1.2 (稍微大一点点作为缓冲)
        float hardRadius = 0.5f; // 硬半径 (物理接触界限)

        // 9 宫格搜索
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
                        if (!TransformLookup.HasComponent(neighborEntity)) continue;

                        float3 neighborPos = TransformLookup[neighborEntity].Position;
                        float3 diff = transform.Position - neighborPos;
                        diff.y = 0; // 2D 锁定

                        float distSq = math.lengthsq(diff);
                        float detectRadiusSq = SeparationRadius * SeparationRadius;

                        if (distSq < detectRadiusSq && distSq > 0.00001f)
                        {
                            float dist = math.sqrt(distSq);
                            float3 pushDir = diff / dist; // normalize

                            // 【核心改动 2：分段力场】
                            // 如果距离 < 硬半径 (真的撞上了)，力呈指数级爆炸
                            // 如果距离 > 硬半径 (只是靠近)，力是温柔的线性
                            
                            float forceMagnitude = 0f;
                            
                            if (dist < hardRadius) 
                            {
                                isOverlapping = true; // 标记为严重拥挤
                                // 指数推力：距离越近，力大得越夸张
                                // (hardRadius - dist) / dist 这是一个经典的避障公式
                                forceMagnitude = (hardRadius - dist) / dist * 3.0f; 
                            }
                            else
                            {
                                // 线性推力：温柔维持社交距离
                                forceMagnitude = 1.0f - (dist / SeparationRadius);
                            }

                            separationForce += pushDir * forceMagnitude * speed.Value;
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

        // 4. 合成
        float3 totalForce = moveForce + separationForce;
        float maxForce = settings.MaxForce;
        
        // 【改动 3：如果严重穿模，允许突破最大力限制】
        // 紧急避险时，力气大一点没关系
        if (isOverlapping) maxForce *= 2.0f; 
        
        if (math.length(totalForce) > maxForce)
            totalForce = math.normalize(totalForce) * maxForce;

        velocity.Value += totalForce * DeltaTime;

        // 5. 智能阻尼 (条件刹车)
        // 【核心改动 4：如果正在穿模 (isOverlapping)，禁止任何阻尼！】
        // 只有当你没被挤着的时候，才允许刹车停下来
        if (!isOverlapping)
        {
            if (flowWeight < 0.99f || isAtDestination)
            {
                float damping = isAtDestination ? 0.85f : 0.95f;
                velocity.Value *= math.pow(damping, DeltaTime * 60f);
            }
        }
        else
        {
            // 如果穿模了，不仅不阻尼，甚至可以稍微加速让它滑出去 (可选)
            // velocity.Value *= 1.01f; 
        }

        // 6. 限制最大速度
        if (math.length(velocity.Value) > speed.Value)
            velocity.Value = math.normalize(velocity.Value) * speed.Value;

        // 7. 位移更新
        // 只要有分离力 (neighborCount > 0)，就允许移动，哪怕还没达到速度阈值
        // 这样能消除最后一点点的重叠卡顿
        bool shouldMove = math.lengthsq(velocity.Value) > 0.0001f || neighborCount > 0;

        if (shouldMove)
        {
            // 锁 Y 轴
            velocity.Value.y = 0;
            transform.Position += velocity.Value * DeltaTime;

            if (math.lengthsq(velocity.Value) > 0.01f)
            {
                quaternion targetRot = quaternion.LookRotationSafe(math.normalize(velocity.Value), math.up());
                transform.Rotation = math.slerp(transform.Rotation, targetRot, DeltaTime * 10.0f);
            }
        }
        else
        {
            if (isAtDestination && !isOverlapping) velocity.Value = float3.zero;
        }
    }
}
