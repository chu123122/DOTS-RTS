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

    [ReadOnly] public NativeReference<int> ArrivalEnterDistance;

    public NativeArray<FlowMovementFrameState> States;

    public void Execute(
        [EntityIndexInQuery] int entityIndex,
        in LocalTransform transform,
        in Velocity velocity,
        in UnitMoveSpeed speed,
        in UnitMovementSettings settings,
        ref FlowArrivalState arrivalState)
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

        bool isReachable = cell.Cost != 0 && cell.IntegrationValue != ushort.MaxValue;
        int integrationDistance = cell.IntegrationValue;
        int arrivalEnterDistance = ArrivalEnterDistance.Value;
        int arrivalExitDistance = arrivalEnterDistance + 1;

        // 已进入到达区域的单位只有越过更外层的退出边界才重新跟随流场。
        // 未进入的单位到达内层边界后停车，从而形成一格宽的滞回带。
        bool isSettled = arrivalState.IsSettled
            ? isReachable && integrationDistance <= arrivalExitDistance
            : isReachable && integrationDistance <= arrivalEnterDistance;
        arrivalState.IsSettled = isSettled;

        float3 moveForce = float3.zero;
        if (!isSettled && cell.Cost != 0)
        {
            // 到达区域外保持完整期望速度，不再设置固定格数的提前减速带。
            int2 dirOffset = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
            float3 desiredDir = math.normalize(new float3(dirOffset.x, 0, dirOffset.y));
            moveForce = desiredDir * speed.Value - velocity.Value;
        }

        state.CellPosition = cellPos;
        state.Cell = cell;
        state.IsSettled = isSettled;
        state.IsInsideGrid = true;
        state.IndependentForce = moveForce;
        States[entityIndex] = state;
    }
}
