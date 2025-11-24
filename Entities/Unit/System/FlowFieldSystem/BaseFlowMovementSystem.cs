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

        //移动Job
        var moveJob = new MoveAlongFlowFieldJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime, 
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            
            SpatialMap = spatialMap.Map,
            TransformLookup = transformLookup,
            
            SeparationWeight = 4f,//软分离力
            SeparationRadius = 0.6f //软距离半径，取实际0.5f的1.2倍
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
        // 1. 流场计算
        int2 cellPos = FlowFieldUtils.WorldToCell(transform.Position, GridOrigin, CellRadius);
        
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
        //缓冲权重
        float flowWeight = 1.0f;
        if (cell.IntegrationValue != ushort.MaxValue && cell.IntegrationValue <= arrivalDistance)
        {
            float linearT = (float)cell.IntegrationValue / (float)arrivalDistance;
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
            moveForce = float3.zero; // 单位到达终点时取消流域力
        }

        // 2。 排斥力计算 
        float3 separationForce = float3.zero;   // 分离力，软作用力
        float3 positionCorrection = float3.zero; // 偏移距离，硬作用力
        int neighborCount = 0;
        float hardRadius = 0.5f; 

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                int2 checkCell = cellPos + new int2(x, y);
                if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                    checkCell.y < 0 || checkCell.y >= GridDimensions.y)
                    continue;

                int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);
                
                //障碍物作用力
                if (Grid[checkIndex].Cost == 0)
                {
                    float3 wallPos = GridOrigin + new float3(
                        checkCell.x * CellRadius * 2 + CellRadius, 
                        transform.Position.y, 
                        checkCell.y * CellRadius * 2 + CellRadius
                    );

                    float3 diff = transform.Position - wallPos;
                    diff.y = 0;
                    
                    float distSq = math.lengthsq(diff);
                    float wallCheckRadius = CellRadius + 0.6f; 
                    
                    if (distSq < wallCheckRadius * wallCheckRadius && distSq > 0.0001f)
                    {
                        float dist = math.sqrt(distSq);
                        float3 pushDir = diff / dist;
                        
                        //排斥力
                        float repelStrength = (wallCheckRadius - dist) / dist * 10.0f; 
                        separationForce += pushDir * repelStrength * speed.Value;
                        
                        //直接位移，当物体与墙壁穿模时
                        float wallHardRadius = CellRadius + 0.5f;
                        if (dist < wallHardRadius)
                        {
                            float penetration = wallHardRadius - dist;
                            positionCorrection += pushDir * (penetration * 0.5f);
                        }
                    }
                    continue; 
                }
                
                //当前Grid存在邻居时，遍历计算
                if (SpatialMap.TryGetFirstValue(checkIndex, out Entity neighborEntity, out var it))
                {
                    do
                    {
                        if (neighborEntity == entity) continue;
                        if (!TransformLookup.HasComponent(neighborEntity)) continue;

                        float3 neighborPos = TransformLookup[neighborEntity].Position;
                        float3 diff = transform.Position - neighborPos;
                        diff.y = 0; 

                        float distSq = math.lengthsq(diff);
                        float sepRadiusSq = SeparationRadius * SeparationRadius; 

                        if (distSq < sepRadiusSq && distSq > 0.00001f)
                        {
                            float dist = math.sqrt(distSq);
                            float3 pushDir = diff / dist; 

                            // 硬穿模修正 
                            if (dist < hardRadius)
                            {
                                float penetration = hardRadius - dist;
                                positionCorrection += pushDir * (penetration * 0.4f); 
                            }
                            
                            //Biods分离力 
                            float softFactor = 1.0f - (dist / SeparationRadius);
                            separationForce += pushDir * softFactor * speed.Value;
                            
                            neighborCount++;
                        }

                    } while (SpatialMap.TryGetNextValue(out neighborEntity, ref it));
                }
            }
        }

        if (neighborCount > 0)
        {
            separationForce /= neighborCount;
            float currentSepWeight = isAtDestination ? SeparationWeight * 1.5f : SeparationWeight;
            separationForce *= currentSepWeight;
        }
        
        // 3. 合力作用
        float3 totalForce = moveForce + separationForce;//合力=流域力（Flow Field）+分离力（Biods）
        
        //被卡在障碍物里面时，强制推出
        if (cell.Cost == 0 && math.lengthsq(totalForce) < 0.1f)
        {
            float3 cellCenter = GridOrigin + new float3(
                cellPos.x * CellRadius * 2 + CellRadius, 
                transform.Position.y, 
                cellPos.y * CellRadius * 2 + CellRadius
            );
            float3 escapeDir = math.normalize(transform.Position - cellCenter);
            if (math.lengthsq(escapeDir) < 0.001f) escapeDir = new float3(1,0,0); 
            totalForce += escapeDir * speed.Value * 5.0f;
        }
        
        float maxForce = settings.MaxForce;
        if (math.length(totalForce) > maxForce)
            totalForce = math.normalize(totalForce) * maxForce;

        velocity.Value += totalForce * DeltaTime;

        // 发生穿模（硬碰撞）
        bool isHardColliding = math.lengthsq(positionCorrection) > 0.0001f;

        if (isAtDestination && !isHardColliding)
        {
            // 强阻尼力停车
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
        
        // 速度够快或正在发生硬穿模
        bool shouldMove = math.lengthsq(velocity.Value) > 0.005f || isHardColliding;
        if (shouldMove)
        {
            float3 newPos = transform.Position + velocity.Value * DeltaTime;
            
            //PBD修正
            if (isHardColliding)
            {
                float maxCorrectionPerFrame = 0.15f; 
                if (math.lengthsq(positionCorrection) > maxCorrectionPerFrame * maxCorrectionPerFrame)
                    positionCorrection = math.normalize(positionCorrection) * maxCorrectionPerFrame;
                
                newPos += positionCorrection;
            }

            newPos.y = transform.Position.y; 
            transform.Position = newPos;
            velocity.Value.y = 0;

            //旋转
            if (math.lengthsq(velocity.Value) > 0.01f)
            {
                quaternion targetRot = quaternion.LookRotationSafe(math.normalize(velocity.Value), math.up());
                transform.Rotation = math.slerp(transform.Rotation, targetRot, DeltaTime * 10.0f);
            }
        }
        else
        {
            if (isAtDestination && !isHardColliding) velocity.Value = float3.zero;
        }
    }
}