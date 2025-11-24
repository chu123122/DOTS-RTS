using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using UnityEngine; // 必须引用
using 通用; // 引用 FlowFieldUtils

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
        Debug.Log($"index:{index},worldPos:{worldPos}");
        
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