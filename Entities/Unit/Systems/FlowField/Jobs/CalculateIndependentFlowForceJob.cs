using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{

/// <summary>
/// 计算不依赖其他单位的移动力，并初始化本帧单位状态。
/// 本阶段只读取单位自身状态和流场，不访问邻居。
/// </summary>
[BurstCompile]
public partial struct CalculateIndependentFlowForceJob : IJobEntity
{
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    [ReadOnly] public NativeArray<float2> CollisionFootprints;

    public NativeArray<FlowMovementFrameState> States;

    public void Execute(
        Entity entity,
        [EntityIndexInQuery] int entityIndex,
        in LocalTransform transform,
        in Velocity velocity,
        in UnitMoveSpeed speed,
        in UnitMovementSettings settings,
        in UnitContactBody contactBody,
        in UnitMoveDestination destination,
        ref FlowArrivalState arrivalState)
    {
        var state = new FlowMovementFrameState
        {
            Entity = entity,
            CurrentPosition = transform.Position,
            CurrentRotation = transform.Rotation,
            CurrentVelocity = velocity.Value,
            MoveSpeed = speed.Value,
            MaxForce = settings.MaxForce,
            InverseMass = math.max(0f, contactBody.InverseMass),
            Radius = math.cmax(CollisionFootprints[entityIndex]) * 0.5f
        };

        // 越界单位不参与本帧后续求解，并在最终阶段停止速度。
        int2 cellPos = FlowFieldUtils.WorldToCell(transform.Position, GridOrigin, CellRadius);
        if (cellPos.x < 0 || cellPos.x >= GridDimensions.x ||
            cellPos.y < 0 || cellPos.y >= GridDimensions.y)
        {
            state.IsInsideGrid = false;
            States[entityIndex] = state;
            return;
        }

        int flatIndex = FlowFieldUtils.GetFlatIndex(cellPos, GridDimensions);
        FlowFieldCell cell = Grid[flatIndex];

        bool hasActiveDestination = destination.IsActive != 0;
        float2 destinationDelta = destination.Position.xz - transform.Position.xz;
        float destinationDistance = math.length(destinationDelta);
        float arrivalEnterRadius = math.max(0.01f, destination.ArrivalRadius);
        float arrivalExitRadius = arrivalEnterRadius + math.max(0.05f, state.Radius * 0.5f);
        bool isSettled = !hasActiveDestination ||
                         (arrivalState.IsSettled
                             ? destinationDistance <= arrivalExitRadius
                             : destinationDistance <= arrivalEnterRadius);
        arrivalState.IsSettled = isSettled;

        float3 moveForce = float3.zero;
        if (hasActiveDestination && !isSettled && cell.Cost != 0)
        {
            bool useDirectApproach =
                cell.IntegrationValue != ushort.MaxValue &&
                cell.IntegrationValue <= destination.DirectApproachIntegrationDistance;
            float3 desiredVelocity;
            if (useDirectApproach)
            {
                float3 desiredDirection = math.normalizesafe(
                    new float3(destinationDelta.x, 0f, destinationDelta.y));
                float brakingDistance = math.max(
                    CellRadius * 2f,
                    math.max(state.Radius * 2f, arrivalExitRadius));
                float speedScale = math.saturate(destinationDistance / brakingDistance);
                desiredVelocity = desiredDirection * speed.Value * speedScale;
            }
            else
            {
                int2 dirOffset = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
                float3 desiredDirection = math.normalizesafe(
                    new float3(dirOffset.x, 0f, dirOffset.y));
                desiredVelocity = desiredDirection * speed.Value;
            }

            moveForce = desiredVelocity - velocity.Value;
        }

        state.CellPosition = cellPos;
        state.Cell = cell;
        state.IsSettled = isSettled;
        state.IsInsideGrid = true;
        state.IndependentForce = moveForce;
        States[entityIndex] = state;
    }
}
}
