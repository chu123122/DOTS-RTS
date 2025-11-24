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
    // 物理世界的只读副本，用于查询碰撞
    [ReadOnly] public CollisionWorld CollisionWorld;
    
    public NativeArray<FlowFieldCell> Grid;
    
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    
    // 过滤器：决定我们要检测哪一层 (Layer)
    public CollisionFilter ObstacleFilter;

    public void Execute(int index)
    {
        // 1. 算出当前格子的世界坐标中心
        int2 cellPos = FlowFieldUtils.GetCellPosFromIndex(index, GridDimensions);
        
        // 坐标转换：从 grid index -> world position
        // x = origin.x + cell.x * size + radius
        float cellSize = CellRadius * 2;
        float3 worldPos = GridOrigin + new float3(
            cellPos.x * cellSize + CellRadius,
            0.5f, 
            cellPos.y * cellSize + CellRadius
        );
        Debug.Log($"index:{index},worldPos:{worldPos}");

        // 2. 准备碰撞检测输入
        // 我们用一个比格子稍小的球体来检测，防止边缘误判
        PointDistanceInput input = new PointDistanceInput
        {
            Position = worldPos,
            MaxDistance = CellRadius * 0.9f, // 检测半径
            Filter = ObstacleFilter
        };

        // 3. 执行检测 (CalculateDistance 类似于 OverlapSphere)
        // 如果在这个范围内碰到了 Filter 指定的物体，返回 true
        if (CollisionWorld.CalculateDistance(input, out DistanceHit hit))
        {
            // 碰到障碍物 -> Cost = 0 (不可通行)
            var cell = Grid[index];
            cell.Cost = 0; 
            // 既然是障碍物，之前的 BFS 数据也要重置一下以防万一
            cell.IntegrationValue = ushort.MaxValue;
            cell.BestDirectionIndex = 0xFF;
            Grid[index] = cell;
        }
        else
        {
            // 没碰到 -> Cost = 1 (平地)
            var cell = Grid[index];
            cell.Cost = 1;
            Grid[index] = cell;
        }
    }
}