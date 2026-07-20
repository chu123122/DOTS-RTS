using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

/// <summary>
/// 当前帧唯一单位碰撞候选。索引指向同一帧的 FlowMovementFrameState 数组。
/// </summary>
public struct UnitCollisionPair
{
    public int BodyA;
    public int BodyB;
}

/// <summary>
/// 从当前 Spatial Hash 生成唯一单位 Pair 快照。
/// BodyA 始终小于 BodyB，因此同一 Pair 不会从双方视角重复写入。
/// </summary>
[BurstCompile]
public partial struct BuildUniqueUnitCollisionPairsJob : IJobEntity
{
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public int2 GridDimensions;

    [ReadOnly] public NativeParallelMultiHashMap<int, Entity> SpatialMap;
    [ReadOnly] public NativeParallelHashMap<Entity, int> EntityToIndex;
    [ReadOnly] public NativeArray<FlowMovementFrameState> States;

    public NativeStream.Writer PairWriter;

    public void Execute(Entity entity, [EntityIndexInQuery] int entityIndex)
    {
        PairWriter.BeginForEachIndex(entityIndex);

        FlowMovementFrameState state = States[entityIndex];
        if (!state.IsInsideGrid)
        {
            PairWriter.EndForEachIndex();
            return;
        }

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
                    continue;

                if (!SpatialMap.TryGetFirstValue(checkIndex, out Entity neighborEntity, out var iterator))
                    continue;

                do
                {
                    if (neighborEntity == entity) continue;
                    if (!EntityToIndex.TryGetValue(neighborEntity, out int neighborIndex)) continue;
                    if (entityIndex >= neighborIndex) continue;

                    PairWriter.Write(new UnitCollisionPair
                    {
                        BodyA = entityIndex,
                        BodyB = neighborIndex
                    });
                } while (SpatialMap.TryGetNextValue(out neighborEntity, ref iterator));
            }
        }

        PairWriter.EndForEachIndex();
    }
}

/// <summary>
/// 将并行生成的分段 Pair 流压平成连续数组，供后续求解阶段稳定遍历。
/// </summary>
[BurstCompile]
public struct CollectUnitCollisionPairsJob : IJob
{
    [ReadOnly] public NativeStream.Reader PairReader;
    public NativeList<UnitCollisionPair> Pairs;

    public void Execute()
    {
        for (int i = 0; i < PairReader.ForEachCount; i++)
        {
            int pairCount = PairReader.BeginForEachIndex(i);
            for (int pairIndex = 0; pairIndex < pairCount; pairIndex++)
                Pairs.Add(PairReader.Read<UnitCollisionPair>());
            PairReader.EndForEachIndex();
        }
    }
}

/// <summary>
/// 保留现有墙壁位置投影；单位接触已移交给唯一 Pair 求解 Job。
/// </summary>
[BurstCompile]
public partial struct CalculateWallConstraintsJob : IJobEntity
{
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    public NativeArray<FlowMovementFrameState> States;

    public void Execute([EntityIndexInQuery] int entityIndex)
    {
        FlowMovementFrameState state = States[entityIndex];
        if (!state.IsInsideGrid) return;

        float3 positionCorrection = float3.zero;
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                int2 checkCell = state.CellPosition + new int2(x, y);
                if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                    checkCell.y < 0 || checkCell.y >= GridDimensions.y)
                    continue;

                int checkIndex = FlowFieldUtils.GetFlatIndex(checkCell, GridDimensions);
                if (Grid[checkIndex].Cost != 0)
                    continue;

                float3 wallPosition = GridOrigin + new float3(
                    checkCell.x * CellRadius * 2 + CellRadius,
                    state.PredictedPosition.y,
                    checkCell.y * CellRadius * 2 + CellRadius);

                AccumulateWallConstraint(state.PredictedPosition, wallPosition, ref positionCorrection);
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
}

/// <summary>
/// 单轮 Jacobi 单位接触投影。
/// 所有约束都读取同一份预测位置，只向双方累计修正，因此结果不依赖 Pair 遍历顺序。
/// </summary>
[BurstCompile]
public struct SolveUnitCollisionPairsJob : IJob
{
    [ReadOnly] public NativeArray<UnitCollisionPair> Pairs;
    [ReadOnly] public NativeArray<float2> CollisionFootprints;
    public NativeArray<FlowMovementFrameState> States;

    public void Execute()
    {
        for (int i = 0; i < Pairs.Length; i++)
        {
            UnitCollisionPair pair = Pairs[i];
            FlowMovementFrameState bodyA = States[pair.BodyA];
            FlowMovementFrameState bodyB = States[pair.BodyB];

            float3 diff = bodyA.PredictedPosition - bodyB.PredictedPosition;
            diff.y = 0;

            float distSq = math.lengthsq(diff);
            if (distSq <= 0.00001f)
                continue;

            float radiusA = math.cmax(CollisionFootprints[pair.BodyA]) * 0.5f;
            float radiusB = math.cmax(CollisionFootprints[pair.BodyB]) * 0.5f;
            float targetDistance = radiusA + radiusB;
            if (distSq >= targetDistance * targetDistance)
                continue;

            float distance = math.sqrt(distSq);
            float3 normal = diff / distance;
            float penetration = targetDistance - distance;
            float3 correction = normal * (penetration * 0.4f);

            bodyA.PositionCorrection += correction;
            bodyB.PositionCorrection -= correction;
            States[pair.BodyA] = bodyA;
            States[pair.BodyB] = bodyB;
        }
    }
}
