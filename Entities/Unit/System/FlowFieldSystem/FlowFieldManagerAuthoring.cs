using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

public class FlowFieldManagerAuthoring : MonoBehaviour
{
    public float cellRadius = 0.5f; 
    public int2 gridSize = new int2(100, 100);

    public class Baker : Baker<FlowFieldManagerAuthoring>
    {
        public override void Bake(FlowFieldManagerAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);

            // 【关键修改】只添加配置，不添加运行时 Grid
            AddComponent(entity, new FlowFieldSettings
            {
                GridDimensions = authoring.gridSize,
                CellRadius = authoring.cellRadius,
                GridOrigin = float3.zero
            });
            
            // 添加全局目标
            AddComponent(entity, new FlowFieldGlobalTarget { TargetPosition = float3.zero });
        }
    }
}