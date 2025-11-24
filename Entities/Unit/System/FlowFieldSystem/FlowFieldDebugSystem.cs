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
    public bool ShowVectors = true;
    public bool ShowCost = true;

    protected override void OnUpdate()
    {
        if (!ShowVectors && !ShowCost) return;
        if (!SystemAPI.TryGetSingleton<FlowFieldGrid>(out var grid)) return;
        
        Dependency.Complete();
        
        if (!grid.Grid.IsCreated) return;

        var cells = grid.Grid;
        int2 dims = grid.GridDimensions;
        float3 origin = grid.GridOrigin;
        float radius = grid.CellRadius;
        float cellSize = radius * 2;
        
        for (int x = 0; x < dims.x; x++)
        {
            for (int y = 0; y < dims.y; y++)
            {
                int index = FlowFieldUtils.GetFlatIndex(new int2(x, y), dims);
                var cell = cells[index];
                
                float3 cellCenter = origin + new float3(x * cellSize + radius, 0, y * cellSize + radius);

                if (ShowCost && cell.Cost == 0)
                    Debug.DrawRay(cellCenter, Vector3.up * 2, Color.red);
                
                if (ShowVectors && cell.BestDirectionIndex != 0xFF)
                {
                    int2 dirOffset = FlowFieldUtils.GetDirectionOffset(cell.BestDirectionIndex);
                    float3 dir = new float3(dirOffset.x, 0, dirOffset.y);
                    
                    Debug.DrawRay(cellCenter, dir * radius, Color.white);
                }
                if (cell.IntegrationValue == 0)
                    Debug.DrawRay(cellCenter, Vector3.up * 5, Color.green);
            }
        }
    }
}