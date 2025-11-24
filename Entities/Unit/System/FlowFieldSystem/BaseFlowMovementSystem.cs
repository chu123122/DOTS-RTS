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
            SeparationWeight = 4f,
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
        // ========================================================
        // Phase 1: 环境感知 & 流场计算 (Sensing & Macro Force)
        // ========================================================
        int2 cellPos = FlowFieldUtils.WorldToCell(transform.Position, GridOrigin, CellRadius);
        
        // 边界保护
        if (cellPos.x < 0 || cellPos.x >= GridDimensions.x || 
            cellPos.y < 0 || cellPos.y >= GridDimensions.y) 
        {
            velocity.Value = float3.zero; 
            return;
        }

        int flatIndex = FlowFieldUtils.GetFlatIndex(cellPos, GridDimensions);
        FlowFieldCell cell = Grid[flatIndex];

        // 缓冲区逻辑
        int arrivalDistance = 2; 
        float flowWeight = 1.0f;
        if (cell.IntegrationValue != ushort.MaxValue && cell.IntegrationValue <= arrivalDistance)
        {
            float linearT = (float)cell.IntegrationValue / (float)arrivalDistance;
            // 使用开方曲线，保持冲劲
            flowWeight = math.sqrt(linearT);
        }

        // 到达判定
        bool isAtDestination =  (cell.IntegrationValue == 0);

        // 流场驱动力
        float3 moveForce = float3.zero;
        if (!isAtDestination&& cell.Cost != 0)
        {
            int2 dirOffset = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
            float3 desiredDir = math.normalize(new float3(dirOffset.x, 0, dirOffset.y));
            moveForce = (desiredDir * speed.Value * flowWeight) - velocity.Value;
        }
        else if (isAtDestination)
        {
            moveForce = float3.zero; // 到了终点切断动力，交给阻尼和分离力
        }

        // ========================================================
        // Phase 2: 邻居搜索 (同时计算 "软力" 和 "硬修正")
        // ========================================================
        float3 separationForce = float3.zero;   // 软力：影响速度
        float3 positionCorrection = float3.zero; // 硬修正：直接修位置
        int neighborCount = 0;
        
        // 【关键参数】硬半径 = 单位直径 (1.0)
        // 小于这个距离代表物理穿模，必须修位置
        float hardRadius = 0.5f; 

        // 9 宫格循环
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                int2 checkCell = cellPos + new int2(x, y);
                if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                    checkCell.y < 0 || checkCell.y >= GridDimensions.y) continue;

                int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);
                if (Grid[checkIndex].Cost == 0)
                {
                    // 计算墙格子的世界中心
                    float3 wallPos = GridOrigin + new float3(
                        checkCell.x * CellRadius * 2 + CellRadius, 
                        transform.Position.y, 
                        checkCell.y * CellRadius * 2 + CellRadius
                    );

                    float3 diff = transform.Position - wallPos;
                    diff.y = 0;
                    
                    float distSq = math.lengthsq(diff);
                    // 墙壁的警戒半径：格子半径 + 单位半径 + 缓冲
                    // 假设格子 0.5*2=1.0，单位 0.5。警戒距离 1.0 比较合适
                    float wallCheckRadius = CellRadius + 0.6f; 
                    
                    if (distSq < wallCheckRadius * wallCheckRadius && distSq > 0.0001f)
                    {
                        float dist = math.sqrt(distSq);
                        float3 pushDir = diff / dist;
                        
                        // 墙壁斥力：非常强硬 (10倍权重)
                        float repelStrength = (wallCheckRadius - dist) / dist * 10.0f; 
                        separationForce += pushDir * repelStrength * speed.Value;
                        
                        // 墙壁硬修正：如果真的陷进去了，直接推出来
                        // 阈值：格子半径 + 单位半径 (0.5 + 0.5 = 1.0)
                        float wallHardRadius = CellRadius + 0.5f;
                        if (dist < wallHardRadius)
                        {
                            float penetration = wallHardRadius - dist;
                            positionCorrection += pushDir * (penetration * 0.5f);
                        }
                    }
                    continue; // 是墙就不用查 Entity HashMap 了
                }
                
                if (SpatialMap.TryGetFirstValue(checkIndex, out Entity neighborEntity, out var it))
                {
                    do
                    {
                        if (neighborEntity == entity) continue;
                        if (!TransformLookup.HasComponent(neighborEntity)) continue;

                        float3 neighborPos = TransformLookup[neighborEntity].Position;
                        float3 diff = transform.Position - neighborPos;
                        
                        // 【关键】强制 2D 锁定，防止被挤飞到天上
                        diff.y = 0; 

                        float distSq = math.lengthsq(diff);
                        float sepRadiusSq = SeparationRadius * SeparationRadius; // 软半径平方

                        // 只有在侦测范围内才处理
                        if (distSq < sepRadiusSq && distSq > 0.00001f)
                        {
                            float dist = math.sqrt(distSq);
                            float3 pushDir = diff / dist; // 标准化方向向量

                            // >>> A. 硬穿模修正 (PBD Logic) <<<
                            if (dist < hardRadius)
                            {
                                float penetration = hardRadius - dist;
                                // 累加修正向量。系数 0.4 意味着每帧修正 40% 的穿透量
                                // 两个单位各修 40%，合起来就是 80%，迅速分开且不震荡
                                positionCorrection += pushDir * (penetration * 0.4f); 
                            }
                            
                            // >>> B. 软分离力 (Velocity Logic) <<<
                            // 即使在修位置，也要给个速度斥力，保持队形松散
                            // 线性公式：(Radius - dist) / Radius
                            float softFactor = 1.0f - (dist / SeparationRadius);
                            separationForce += pushDir * softFactor * speed.Value;
                            
                            neighborCount++;
                        }

                    } while (SpatialMap.TryGetNextValue(out neighborEntity, ref it));
                }
            }
        }

        // 应用软力平均值
        if (neighborCount > 0)
        {
            separationForce /= neighborCount;
            // 终点时加大权重，防止挤成一团
            float currentSepWeight = isAtDestination ? SeparationWeight * 1.5f : SeparationWeight;
            separationForce *= currentSepWeight;
        }

        // ========================================================
        // Phase 3: 物理积分 (Velocity Update)
        // ========================================================
        float3 totalForce = moveForce + separationForce;
        
        if (cell.Cost == 0 && math.lengthsq(totalForce) < 0.1f)
        {
            // 简单的逃逸：沿当前格子中心向外推
            float3 cellCenter = GridOrigin + new float3(
                cellPos.x * CellRadius * 2 + CellRadius, 
                transform.Position.y, 
                cellPos.y * CellRadius * 2 + CellRadius
            );
            float3 escapeDir = math.normalize(transform.Position - cellCenter);
            if (math.lengthsq(escapeDir) < 0.001f) escapeDir = new float3(1,0,0); // 防止重合
            totalForce += escapeDir * speed.Value * 5.0f;
        }
        
        float maxForce = settings.MaxForce;
        if (math.length(totalForce) > maxForce)
            totalForce = math.normalize(totalForce) * maxForce;

        velocity.Value += totalForce * DeltaTime;

        // 阻尼逻辑：只有在 (到达终点) 且 (没有发生硬穿模) 时才刹车
        // 如果正在被挤 (positionCorrection > 0)，就不要刹车，顺滑滑开
        bool isHardColliding = math.lengthsq(positionCorrection) > 0.0001f;

        if (isAtDestination && !isHardColliding)
        {
            // 强阻尼停车
            velocity.Value *= math.pow(0.8f, DeltaTime * 60f);
        }
        else if (flowWeight < 0.99f)
        {
            // 缓冲区轻微减速
            velocity.Value *= math.pow(0.95f, DeltaTime * 60f);
        }

        // 限速
        if (math.length(velocity.Value) > speed.Value)
            velocity.Value = math.normalize(velocity.Value) * speed.Value;

        // ========================================================
        // Phase 4: 位置更新 & 硬修正应用 (Position Update)
        // ========================================================
        
        // 允许移动条件：速度够快 或 正在发生硬穿模
        bool shouldMove = math.lengthsq(velocity.Value) > 0.005f || isHardColliding;

        if (shouldMove)
        {
            // Step 1: 正常速度位移
            float3 newPos = transform.Position + velocity.Value * DeltaTime;

            // Step 2: 叠加硬修正 (解决穿模)
            if (isHardColliding)
            {
                // 限制单帧最大修正量 (防止瞬移穿墙)
                float maxCorrectionPerFrame = 0.15f; 
                if (math.lengthsq(positionCorrection) > maxCorrectionPerFrame * maxCorrectionPerFrame)
                {
                    positionCorrection = math.normalize(positionCorrection) * maxCorrectionPerFrame;
                }
                
                // 直接修改坐标！
                newPos += positionCorrection;
            }

            // 强制锁 Y 轴
            newPos.y = transform.Position.y; // 保持高度不变
            transform.Position = newPos;
            velocity.Value.y = 0;

            // Step 3: 平滑旋转
            if (math.lengthsq(velocity.Value) > 0.01f)
            {
                quaternion targetRot = quaternion.LookRotationSafe(math.normalize(velocity.Value), math.up());
                transform.Rotation = math.slerp(transform.Rotation, targetRot, DeltaTime * 10.0f);
            }
        }
        else
        {
            // 彻底静止
            if (isAtDestination && !isHardColliding) velocity.Value = float3.zero;
        }
    }
}
