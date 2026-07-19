using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using 通用;

/// <summary>
/// 在预测位置上检测墙壁/单位穿透，并累计位置空间修正量。
/// 当前实现每个单位独立求修正，尚未建立唯一碰撞对，也没有迭代投影。
/// </summary>
[BurstCompile]
public partial struct CalculateFlowConstraintsJob : IJobEntity
{
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;

    [ReadOnly] public NativeParallelMultiHashMap<int, Entity> SpatialMap;
    [ReadOnly] public NativeParallelHashMap<Entity, float3> PredictedPositions;

    public float SeparationRadius;
    public NativeArray<FlowMovementFrameState> States;

    public void Execute(Entity entity, [EntityIndexInQuery] int entityIndex)
    {
        FlowMovementFrameState state = States[entityIndex];
        if (!state.IsInsideGrid) return;

        float3 positionCorrection = float3.zero;

        // 当前 SpatialMap 只承担宽相候选筛选；它仍由当前帧位置构建。
        // 窄相距离判断则统一使用 PredictedPositions 中的预测位置。
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
                    float3 wallPosition = GridOrigin + new float3(
                        checkCell.x * CellRadius * 2 + CellRadius,
                        state.PredictedPosition.y,
                        checkCell.y * CellRadius * 2 + CellRadius);

                    AccumulateWallConstraint(state.PredictedPosition, wallPosition, ref positionCorrection);
                    continue;
                }

                if (!SpatialMap.TryGetFirstValue(checkIndex, out Entity neighborEntity, out var iterator))
                    continue;

                do
                {
                    if (neighborEntity == entity) continue;
                    if (!PredictedPositions.TryGetValue(neighborEntity, out float3 neighborPosition)) continue;

                    // 双方都读取同一预测快照，避免约束结果受 Job 执行先后顺序影响。
                    AccumulateUnitConstraint(state.PredictedPosition, neighborPosition, ref positionCorrection);
                } while (SpatialMap.TryGetNextValue(out neighborEntity, ref iterator));
            }
        }

        state.PositionCorrection = positionCorrection;
        States[entityIndex] = state;
    }

    private void AccumulateWallConstraint(
        float3 position,
        float3 wallPosition,
        ref float3 positionCorrection)
    {
        float3 diff = position - wallPosition;
        diff.y = 0;

        float distSq = math.lengthsq(diff);
        float wallCheckRadius = CellRadius + 0.6f;
        if (distSq >= wallCheckRadius * wallCheckRadius || distSq <= 0.0001f)
            return;

        float dist = math.sqrt(distSq);
        float wallHardRadius = CellRadius + 0.5f;
        if (dist >= wallHardRadius) return;

        float3 pushDirection = diff / dist;
        float penetration = wallHardRadius - dist;
        positionCorrection += pushDirection * (penetration * 0.5f);
    }

    private void AccumulateUnitConstraint(
        float3 position,
        float3 neighborPosition,
        ref float3 positionCorrection)
    {
        float3 diff = position - neighborPosition;
        diff.y = 0;

        float distSq = math.lengthsq(diff);
        float separationRadiusSq = SeparationRadius * SeparationRadius;
        if (distSq >= separationRadiusSq || distSq <= 0.00001f)
            return;

        float dist = math.sqrt(distSq);
        const float hardRadius = 0.5f;
        if (dist >= hardRadius) return;

        float3 pushDirection = diff / dist;
        float penetration = hardRadius - dist;

        // 当前双方会各自计算一次修正，0.4 是单侧应用的经验比例，并非完整 PBD 质量权重。
        positionCorrection += pushDirection * (penetration * 0.4f);
    }
}
