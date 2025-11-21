using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;

namespace Entities.Unit.System.FlowFieldSystem
{
    
    public struct RecalculateFlowFieldTag : IComponentData {}
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
    public partial class FlowFieldBakeSystem : SystemBase
    {
        protected override void OnCreate()
        {
            RequireForUpdate<FlowFieldSettings>();
            RequireForUpdate<RecalculateFlowFieldTag>();
        }

        protected override void OnUpdate()
        {
            if (!SystemAPI.TryGetSingletonEntity<FlowFieldSettings>(out Entity managerEntity)) return;
            if (!EntityManager.HasComponent<FlowFieldGrid>(managerEntity))
            {
                var settings = EntityManager.GetComponentData<FlowFieldSettings>(managerEntity);
                int totalCells = settings.GridDimensions.x * settings.GridDimensions.y;
                Debug.Log($"totalCells: {totalCells}");
                // 动态创建运行时组件
                var runtimeGrid = new FlowFieldGrid
                {
                    GridDimensions = settings.GridDimensions,
                    CellRadius = settings.CellRadius,
                    GridOrigin = settings.GridOrigin,
                    // 在这里分配内存！Allocator.Persistent 意味着它会一直存在直到游戏结束
                    Grid = new NativeArray<FlowFieldCell>(totalCells, Allocator.Persistent)
                };

                // 添加到 Entity 上
                EntityManager.AddComponentData(managerEntity, runtimeGrid);
        
                // 顺便触发第一次计算
               // EntityManager.AddComponent<RecalculateFlowFieldTag>(managerEntity);
            }
            
            
            var gridEntity =SystemAPI.GetSingletonEntity<FlowFieldGrid>();
            var gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();
            
            if(!SystemAPI.HasComponent<FlowFieldGlobalTarget>(gridEntity))return;
            float3 targetPos = SystemAPI.GetComponent<FlowFieldGlobalTarget>(gridEntity).TargetPosition;
            
            var queue = new NativeQueue<int2>(Allocator.TempJob);

            var resetJob = new ResetGridJob()
            {
                Grid = gridComponent.Grid
            };
            JobHandle resetHandle = resetJob.Schedule(gridComponent.Grid.Length, 64,Dependency);

            var bfsJob = new GenerateIntegrationFieldJob()
            {
                Grid = gridComponent.Grid,
                GridDimensions = gridComponent.GridDimensions,
                TargetCell = FlowFieldUtils.WorldToCell(targetPos,gridComponent.GridOrigin,gridComponent.CellRadius),
                Queue = queue,
            };
            JobHandle bfsHandle = bfsJob.Schedule(resetHandle);

            var vectorJob = new GenerateVectorFieldJob()
            {
                Grid = gridComponent.Grid,
                GridDimensions = gridComponent.GridDimensions,
            };
            JobHandle vectorHandle = vectorJob.Schedule(gridComponent.Grid.Length, 64,bfsHandle);
            
            queue.Dispose(vectorHandle);
            EntityManager.RemoveComponent<RecalculateFlowFieldTag>(gridEntity);
            Dependency= vectorHandle;
            Debug.Log($"预烘培完成！");

        }
        
        protected override void OnDestroy()
        {
            // 查找所有 FlowFieldGrid 并释放内存
            foreach (var grid in SystemAPI.Query<RefRW<FlowFieldGrid>>())
            {
                if (grid.ValueRW.Grid.IsCreated)
                {
                    grid.ValueRW.Grid.Dispose();
                }
            }
        }
    }
    
}