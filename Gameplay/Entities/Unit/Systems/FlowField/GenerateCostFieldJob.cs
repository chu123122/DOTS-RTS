using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{

[BurstCompile]
public struct GenerateCostFieldJob : IJobParallelFor
{
    [ReadOnly] public CollisionWorld CollisionWorld;
    
    public NativeArray<FlowFieldCell> Grid;
    
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    
    public CollisionFilter ObstacleFilter;

    public void Execute(int index)
    {
        int2 cellPos = FlowFieldUtils.GetCellPosFromIndex(index, GridDimensions);
        
        float cellSize = CellRadius * 2;
        float3 worldPos = GridOrigin + new float3(
            cellPos.x * cellSize + CellRadius,
            1f, 
            cellPos.y * cellSize + CellRadius
        );
        PointDistanceInput input = new PointDistanceInput
        {
            Position = worldPos,
            MaxDistance = CellRadius , 
            Filter = ObstacleFilter
        };
        
        if (CollisionWorld.CalculateDistance(input, out DistanceHit hit))
        {
            var cell = Grid[index];
            cell.Cost = 0; 

            cell.IntegrationValue = ushort.MaxValue;
            cell.BestDirectionIndex = 0xFF;
            Grid[index] = cell;
        }
        else
        {
            var cell = Grid[index];
            cell.Cost = 1;
            Grid[index] = cell;
        }
    }
}

/// <summary>
/// 在 Unity Physics world 的明确读取点发布 Crowd 障碍语义。
/// 本 Job 不读取 FlowFieldCell，Navigation cost 与 Physics obstacle 是两个独立产品。
/// </summary>
[BurstCompile]
public struct GenerateCrowdObstacleFieldJob : IJobParallelFor
{
    [ReadOnly] public CollisionWorld CollisionWorld;
    public NativeArray<CrowdObstacleCell> ObstacleCells;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    public CollisionFilter ObstacleFilter;

    public void Execute(int index)
    {
        int2 cell = FlowFieldUtils.GetCellPosFromIndex(
            index,
            GridDimensions);
        float cellSize = CellRadius * 2f;
        float3 worldPosition = GridOrigin + new float3(
            cell.x * cellSize + CellRadius,
            1f,
            cell.y * cellSize + CellRadius);
        PointDistanceInput input = new PointDistanceInput
        {
            Position = worldPosition,
            MaxDistance = CellRadius,
            Filter = ObstacleFilter
        };
        ObstacleCells[index] = new CrowdObstacleCell
        {
            IsBlocked = (byte)(
                CollisionWorld.CalculateDistance(input, out DistanceHit _)
                    ? 1
                    : 0)
        };
    }
}
}
