using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using 通用;

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

    public NativeArray<FlowMovementFrameState> States;

    public void Execute(
        [EntityIndexInQuery] int entityIndex,
        in LocalTransform transform,
        in Velocity velocity,
        in UnitMoveSpeed speed,
        in UnitMovementSettings settings)
    {
        var state = new FlowMovementFrameState
        {
            CurrentPosition = transform.Position,
            CurrentRotation = transform.Rotation,
            CurrentVelocity = velocity.Value,
            MoveSpeed = speed.Value,
            MaxForce = settings.MaxForce
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

        // 靠近终点时逐步降低流场牵引，避免单位以最大速度冲过目标格。
        const int arrivalDistance = 2;
        float flowWeight = 1.0f;
        if (cell.IntegrationValue != ushort.MaxValue && cell.IntegrationValue <= arrivalDistance)
        {
            float linearT = (float)cell.IntegrationValue / arrivalDistance;
            flowWeight = math.sqrt(linearT);
        }

        bool isAtDestination = cell.IntegrationValue == 0;
        float3 moveForce = float3.zero;
        if (!isAtDestination && cell.Cost != 0)
        {
            // Steering 形式：期望速度与当前速度之差作为本阶段的力。
            int2 dirOffset = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
            float3 desiredDir = math.normalize(new float3(dirOffset.x, 0, dirOffset.y));
            moveForce = desiredDir * speed.Value * flowWeight - velocity.Value;
        }

        state.CellPosition = cellPos;
        state.Cell = cell;
        state.FlowWeight = flowWeight;
        state.IsAtDestination = isAtDestination;
        state.IsInsideGrid = true;
        state.IndependentForce = moveForce;
        States[entityIndex] = state;
    }
}
