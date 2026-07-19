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
    private struct MovementContext
    {
        public int2 CellPosition;
        public FlowFieldCell Cell;
        public float FlowWeight;
        public bool IsAtDestination;
    }

    private struct IndependentForceResult
    {
        public float3 Force;
    }

    private struct SoftAvoidanceResult
    {
        public float3 Force;
        public int NeighborCount;
    }

    private struct ConstraintProjectionResult
    {
        public float3 PositionCorrection;
    }

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
        if (!TryBuildMovementContext(transform.Position, out MovementContext context))
        {
            velocity.Value = float3.zero; 
            return;
        }

        IndependentForceResult independentForce = CalculateIndependentForce(context, velocity.Value, speed.Value);
        SoftAvoidanceResult softAvoidance = default;
        ConstraintProjectionResult constraints = default;

        AccumulateLocalInteractions(
            entity,
            transform.Position,
            speed.Value,
            context.CellPosition,
            ref softAvoidance,
            ref constraints);

        FinalizeSoftAvoidance(ref softAvoidance, context.IsAtDestination);
        IntegrateProjectAndWriteBack(
            ref transform,
            ref velocity,
            speed.Value,
            settings,
            context,
            independentForce,
            softAvoidance,
            constraints);
    }

    private bool TryBuildMovementContext(float3 position, out MovementContext context)
    {
        int2 cellPos = FlowFieldUtils.WorldToCell(position, GridOrigin, CellRadius);
        if (cellPos.x < 0 || cellPos.x >= GridDimensions.x ||
            cellPos.y < 0 || cellPos.y >= GridDimensions.y)
        {
            context = default;
            return false;
        }

        int flatIndex = FlowFieldUtils.GetFlatIndex(cellPos, GridDimensions);
        FlowFieldCell cell = Grid[flatIndex];
        const int arrivalDistance = 2;
        float flowWeight = 1.0f;
        if (cell.IntegrationValue != ushort.MaxValue && cell.IntegrationValue <= arrivalDistance)
        {
            float linearT = (float)cell.IntegrationValue / arrivalDistance;
            flowWeight = math.sqrt(linearT);
        }

        context = new MovementContext
        {
            CellPosition = cellPos,
            Cell = cell,
            FlowWeight = flowWeight,
            IsAtDestination = cell.IntegrationValue == 0
        };
        return true;
    }

    private static IndependentForceResult CalculateIndependentForce(
        in MovementContext context,
        float3 currentVelocity,
        float moveSpeed)
    {
        float3 moveForce = float3.zero;
        if (!context.IsAtDestination && context.Cell.Cost != 0)
        {
            int2 dirOffset = FlowFieldUtils.GetDirectionOffset(context.Cell.BestDirectionIndex);
            float3 desiredDir = math.normalize(new float3(dirOffset.x, 0, dirOffset.y));
            moveForce = desiredDir * moveSpeed * context.FlowWeight - currentVelocity;
        }

        return new IndependentForceResult { Force = moveForce };
    }

    private void AccumulateLocalInteractions(
        Entity entity,
        float3 position,
        float moveSpeed,
        int2 cellPos,
        ref SoftAvoidanceResult softAvoidance,
        ref ConstraintProjectionResult constraints)
    {
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
                        position.y,
                        checkCell.y * CellRadius * 2 + CellRadius
                    );

                    AccumulateWallInteraction(
                        position,
                        wallPos,
                        moveSpeed,
                        ref softAvoidance,
                        ref constraints);
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
                        AccumulateUnitInteraction(
                            position,
                            neighborPos,
                            moveSpeed,
                            ref softAvoidance,
                            ref constraints);

                    } while (SpatialMap.TryGetNextValue(out neighborEntity, ref it));
                }
            }
        }
    }

    private void AccumulateWallInteraction(
        float3 position,
        float3 wallPosition,
        float moveSpeed,
        ref SoftAvoidanceResult softAvoidance,
        ref ConstraintProjectionResult constraints)
    {
        float3 diff = position - wallPosition;
        diff.y = 0;

        float distSq = math.lengthsq(diff);
        float wallCheckRadius = CellRadius + 0.6f;
        if (distSq >= wallCheckRadius * wallCheckRadius || distSq <= 0.0001f)
            return;

        float dist = math.sqrt(distSq);
        float3 pushDir = diff / dist;
        float repelStrength = (wallCheckRadius - dist) / dist * 10.0f;
        softAvoidance.Force += pushDir * repelStrength * moveSpeed;

        float wallHardRadius = CellRadius + 0.5f;
        if (dist < wallHardRadius)
        {
            float penetration = wallHardRadius - dist;
            constraints.PositionCorrection += pushDir * (penetration * 0.5f);
        }
    }

    private void AccumulateUnitInteraction(
        float3 position,
        float3 neighborPosition,
        float moveSpeed,
        ref SoftAvoidanceResult softAvoidance,
        ref ConstraintProjectionResult constraints)
    {
        float3 diff = position - neighborPosition;
        diff.y = 0;

        float distSq = math.lengthsq(diff);
        float sepRadiusSq = SeparationRadius * SeparationRadius;
        if (distSq >= sepRadiusSq || distSq <= 0.00001f)
            return;

        float dist = math.sqrt(distSq);
        float3 pushDir = diff / dist;
        const float hardRadius = 0.5f;
        if (dist < hardRadius)
        {
            float penetration = hardRadius - dist;
            constraints.PositionCorrection += pushDir * (penetration * 0.4f);
        }

        float softFactor = 1.0f - dist / SeparationRadius;
        softAvoidance.Force += pushDir * softFactor * moveSpeed;
        softAvoidance.NeighborCount++;
    }

    private void FinalizeSoftAvoidance(ref SoftAvoidanceResult softAvoidance, bool isAtDestination)
    {
        if (softAvoidance.NeighborCount <= 0)
            return;

        softAvoidance.Force /= softAvoidance.NeighborCount;
        float currentSepWeight = isAtDestination ? SeparationWeight * 1.5f : SeparationWeight;
        softAvoidance.Force *= currentSepWeight;
    }

    private void IntegrateProjectAndWriteBack(
        ref LocalTransform transform,
        ref Velocity velocity,
        float moveSpeed,
        in UnitMovementSettings settings,
        in MovementContext context,
        in IndependentForceResult independentForce,
        in SoftAvoidanceResult softAvoidance,
        in ConstraintProjectionResult constraints)
    {
        float3 totalForce = independentForce.Force + softAvoidance.Force;

        //被卡在障碍物里面时，强制推出
        if (context.Cell.Cost == 0 && math.lengthsq(totalForce) < 0.1f)
        {
            float3 cellCenter = GridOrigin + new float3(
                context.CellPosition.x * CellRadius * 2 + CellRadius,
                transform.Position.y, 
                context.CellPosition.y * CellRadius * 2 + CellRadius
            );
            float3 escapeDir = math.normalize(transform.Position - cellCenter);
            if (math.lengthsq(escapeDir) < 0.001f) escapeDir = new float3(1,0,0); 
            totalForce += escapeDir * moveSpeed * 5.0f;
        }
        
        float maxForce = settings.MaxForce;
        if (math.length(totalForce) > maxForce)
            totalForce = math.normalize(totalForce) * maxForce;

        velocity.Value += totalForce * DeltaTime;

        // 发生穿模（硬碰撞）
        float3 positionCorrection = constraints.PositionCorrection;
        bool isHardColliding = math.lengthsq(positionCorrection) > 0.0001f;

        if (context.IsAtDestination && !isHardColliding)
        {
            // 强阻尼力停车
            velocity.Value *= math.pow(0.8f, DeltaTime * 60f);
        }
        else if (context.FlowWeight < 0.99f)
        {
            // 缓冲区轻微减速
            velocity.Value *= math.pow(0.95f, DeltaTime * 60f);
        }

        // 限速
        if (math.length(velocity.Value) > moveSpeed)
            velocity.Value = math.normalize(velocity.Value) * moveSpeed;
        
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
            if (context.IsAtDestination && !isHardColliding) velocity.Value = float3.zero;
        }
    }
}
