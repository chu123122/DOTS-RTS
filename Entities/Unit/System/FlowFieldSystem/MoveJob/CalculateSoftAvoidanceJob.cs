using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using 通用;

/// <summary>
/// 使用当前帧位置计算墙壁和单位产生的软避让力。
/// 软力只改变后续预测速度，不直接修正位置，也不承担硬碰撞约束。
/// </summary>
[BurstCompile]
public partial struct CalculateSoftAvoidanceJob : IJobEntity
{
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    [ReadOnly] public NativeParallelMultiHashMap<int, Entity> SpatialMap;
    [ReadOnly] public ComponentLookup<LocalTransform> TransformLookup;

    public float SeparationWeight;
    public float SeparationRadius;
    public NativeArray<FlowMovementFrameState> States;

    public void Execute(Entity entity, [EntityIndexInQuery] int entityIndex)
    {
        FlowMovementFrameState state = States[entityIndex];
        if (!state.IsInsideGrid) return;

        float3 separationForce = float3.zero;
        int neighborCount = 0;

        // SpatialMap 将搜索限制在当前格周围 3x3 格，而不是遍历所有单位。
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                int2 checkCell = state.CellPosition + new int2(x, y);
                if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                    checkCell.y < 0 || checkCell.y >= GridDimensions.y)
                    continue;

                int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);
                if (Grid[checkIndex].Cost == 0)
                {
                    // 障碍格中心作为近似墙壁位置，产生连续的排斥力。
                    float3 wallPosition = GridOrigin + new float3(
                        checkCell.x * CellRadius * 2 + CellRadius,
                        state.CurrentPosition.y,
                        checkCell.y * CellRadius * 2 + CellRadius);

                    AccumulateWallSoftForce(state.CurrentPosition, wallPosition, state.MoveSpeed, ref separationForce);
                    continue;
                }

                if (!SpatialMap.TryGetFirstValue(checkIndex, out Entity neighborEntity, out var iterator))
                    continue;

                do
                {
                    if (neighborEntity == entity) continue;
                    if (!TransformLookup.HasComponent(neighborEntity)) continue;

                    // 此处有意读取当前帧位置；所有软力求完后才统一积分预测位置。
                    float3 neighborPosition = TransformLookup[neighborEntity].Position;
                    AccumulateUnitSoftForce(
                        state.CurrentPosition,
                        neighborPosition,
                        state.MoveSpeed,
                        ref separationForce,
                        ref neighborCount);
                } while (SpatialMap.TryGetNextValue(out neighborEntity, ref iterator));
            }
        }

        if (neighborCount > 0)
        {
            // 先平均再加权，避免邻居数量直接线性放大软避让力。
            separationForce /= neighborCount;
            float currentWeight = state.IsAtDestination ? SeparationWeight * 1.5f : SeparationWeight;
            separationForce *= currentWeight;
        }

        state.SoftAvoidanceForce = separationForce;
        States[entityIndex] = state;
    }

    private void AccumulateWallSoftForce(
        float3 position,
        float3 wallPosition,
        float moveSpeed,
        ref float3 separationForce)
    {
        float3 diff = position - wallPosition;
        diff.y = 0;

        float distSq = math.lengthsq(diff);
        float wallCheckRadius = CellRadius + 0.6f;
        if (distSq >= wallCheckRadius * wallCheckRadius || distSq <= 0.0001f)
            return;

        float dist = math.sqrt(distSq);
        float3 pushDirection = diff / dist;
        float repelStrength = (wallCheckRadius - dist) / dist * 10.0f;
        separationForce += pushDirection * repelStrength * moveSpeed;
    }

    private void AccumulateUnitSoftForce(
        float3 position,
        float3 neighborPosition,
        float moveSpeed,
        ref float3 separationForce,
        ref int neighborCount)
    {
        float3 diff = position - neighborPosition;
        diff.y = 0;

        float distSq = math.lengthsq(diff);
        float separationRadiusSq = SeparationRadius * SeparationRadius;
        if (distSq >= separationRadiusSq || distSq <= 0.00001f)
            return;

        float dist = math.sqrt(distSq);
        float3 pushDirection = diff / dist;
        float softFactor = 1.0f - dist / SeparationRadius;
        separationForce += pushDirection * softFactor * moveSpeed;
        neighborCount++;
    }
}
