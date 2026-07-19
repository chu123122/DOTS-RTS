using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;

namespace Entities.Unit.System.FlowFieldSystem
{
    
    public struct RecalculateFlowFieldTag : IComponentData {}
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    public partial class FlowFieldBakeSystem : SystemBase
    {
        protected override void OnCreate()
        {
            RequireForUpdate<FlowFieldSettings>();
            RequireForUpdate<RecalculateFlowFieldTag>();
            RequireForUpdate<PhysicsWorldSingleton>();
        }

        protected override void OnUpdate()
        {
            if (!SystemAPI.TryGetSingletonEntity<FlowFieldSettings>(out Entity managerEntity)) return;
            if (!EntityManager.HasComponent<FlowFieldGrid>(managerEntity))
            {
                var settings = EntityManager.GetComponentData<FlowFieldSettings>(managerEntity);
                int totalCells = settings.GridDimensions.x * settings.GridDimensions.y;
                var runtimeGrid = new FlowFieldGrid
                {
                    GridDimensions = settings.GridDimensions,
                    CellRadius = settings.CellRadius,
                    GridOrigin = settings.GridOrigin,
                    Grid = new NativeArray<FlowFieldCell>(totalCells, Allocator.Persistent)
                };

                EntityManager.AddComponentData(managerEntity, runtimeGrid);
        
               // EntityManager.AddComponent<RecalculateFlowFieldTag>(managerEntity);
            }
            
            
            var gridEntity =SystemAPI.GetSingletonEntity<FlowFieldGrid>();
            var gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();
            var costState = SystemAPI.GetSingleton<FlowFieldCostState>();
            
            if(!SystemAPI.HasComponent<FlowFieldGlobalTarget>(gridEntity))return;
            
            float3 targetPos = SystemAPI.GetComponent<FlowFieldGlobalTarget>(gridEntity).TargetPosition;
            var queue = new NativeQueue<int2>(Allocator.TempJob);
            //1.重置流域Job
            var resetJob = new ResetGridJob()
            {
                Grid = gridComponent.Grid
            };
            JobHandle resetHandle = resetJob.Schedule(gridComponent.Grid.Length, 64,Dependency);

            JobHandle costHandle = resetHandle;
            if (costState.IsDirty)
            {
                var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>();
                uint obstacleLayer = 1u << 2;
                CollisionFilter filter = new CollisionFilter
                {
                    BelongsTo = ~0u,
                    CollidesWith = obstacleLayer,
                    GroupIndex = 0
                };

                // Cost 只在障碍物布局失效时重建；普通目标变化直接复用现有结果。
                var costJob = new GenerateCostFieldJob
                {
                    CollisionWorld = physicsWorld.CollisionWorld,
                    Grid = gridComponent.Grid,
                    GridOrigin = gridComponent.GridOrigin,
                    GridDimensions = gridComponent.GridDimensions,
                    CellRadius = gridComponent.CellRadius,
                    ObstacleFilter = filter
                };
                costHandle = costJob.Schedule(gridComponent.Grid.Length, 64, resetHandle);
            }
            
            //3.BFS搜索Job
            var bfsJob = new GenerateIntegrationFieldJob()
            {
                Grid = gridComponent.Grid,
                GridDimensions = gridComponent.GridDimensions,
                TargetCell = FlowFieldUtils.WorldToCell(targetPos,gridComponent.GridOrigin,gridComponent.CellRadius),
                Queue = queue,
            };
            JobHandle bfsHandle = bfsJob.Schedule(costHandle);

            //4.向量场计算Job
            var vectorJob = new GenerateVectorFieldJob()
            {
                Grid = gridComponent.Grid,
                GridDimensions = gridComponent.GridDimensions,
            };
            JobHandle vectorHandle = vectorJob.Schedule(gridComponent.Grid.Length, 64,bfsHandle);
            
            JobHandle queueDisposeHandle = queue.Dispose(vectorHandle);
            EntityManager.RemoveComponent<RecalculateFlowFieldTag>(gridEntity);
            Dependency = queueDisposeHandle;

            var runtimeState = SystemAPI.GetSingletonRW<FlowFieldRuntimeState>();
            runtimeState.ValueRW.ActiveVersion++;

            if (costState.IsDirty)
            {
                var writableCostState = SystemAPI.GetSingletonRW<FlowFieldCostState>();
                writableCostState.ValueRW.IsDirty = false;
                writableCostState.ValueRW.CostVersion++;
            }

        }
        
        protected override void OnDestroy()
        {
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
