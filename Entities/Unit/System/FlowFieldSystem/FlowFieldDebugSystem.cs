using Entities.Unit.System.FlowFieldSystem;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Collections;
using UnityEngine; // 为了 Debug.DrawLine
using 通用;

[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateAfter(typeof(FlowFieldBakeSystem))]
public partial class FlowFieldDebugSystem : SystemBase
{
    // 开关：在 Editor 里随时可以关掉，防止卡顿
    public bool ShowVectors = true;
    public bool ShowCost = true; // 如果你想看阻挡格

    protected override void OnUpdate()
    {
        if (!ShowVectors && !ShowCost) return;

        // 只在 Editor 模式下运行，且只在有 Grid 时运行
        if (!SystemAPI.TryGetSingleton<FlowFieldGrid>(out var grid)) return;
        
        Dependency.Complete();
        
        if (!grid.Grid.IsCreated) return;

        var cells = grid.Grid;
        int2 dims = grid.GridDimensions;
        float3 origin = grid.GridOrigin;
        float radius = grid.CellRadius;
        float cellSize = radius * 2;

        // 遍历所有格子 (注意：Debug.DrawLine 只能在主线程，不能用 Job)
        // 为了性能，我们只画视口附近的，或者只画 1000 个？
        // 这里为了简单，直接全画 (如果格子太多可能会卡，小心)
        for (int x = 0; x < dims.x; x++)
        {
            for (int y = 0; y < dims.y; y++)
            {
                int index = FlowFieldUtils.GetFlatIndex(new int2(x, y), dims);
                var cell = cells[index];
                
                float3 cellCenter = origin + new float3(x * cellSize + radius, 0, y * cellSize + radius);

                // 1. 画障碍物 (红色方块)
                if (ShowCost && cell.Cost == 0)
                {
                    Debug.DrawRay(cellCenter, Vector3.up * 2, Color.red);
                }

                // 2. 画向量场 (白色箭头)
                if (ShowVectors && cell.BestDirectionIndex != 0xFF)
                {
                    int2 dirOffset = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
                    float3 dir = new float3(dirOffset.x, 0, dirOffset.y);
                    
                    // 画一条线指向目标方向
                    Debug.DrawRay(cellCenter, dir * radius, Color.white);
                }
                
                // 3.  画目标点高亮 (绿色)
                if (cell.IntegrationValue == 0)
                {
                    Debug.DrawRay(cellCenter, Vector3.up * 5, Color.green);
                }
            }
        }
    }
}