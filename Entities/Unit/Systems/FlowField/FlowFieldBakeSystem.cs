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
    /// <summary>
    /// 可启停的流场重算请求。RequestVersion 用于识别计算期间到达的新目标。
    /// </summary>
    public struct RecalculateFlowFieldTag : IComponentData, IEnableableComponent
    {
        public uint RequestVersion;
    }

    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateAfter(typeof(FixedStepSimulationSystemGroup))]
    public partial class FlowFieldBakeSystem : SystemBase
    {
        private JobHandle _bakeHandle;
        private JobHandle _activeGridReaders;
        private JobHandle _pendingReuseHandle;
        private bool _isBaking;
        private bool _scheduledCostDirty;
        private bool _waitingForPhysicsWorldRefresh;
        private uint _scheduledRequestVersion;

        protected override void OnCreate()
        {
            RequireForUpdate<FlowFieldSettings>();
            RequireForUpdate<RecalculateFlowFieldTag>();
            RequireForUpdate<PhysicsWorldSingleton>();
        }

        /// <summary>
        /// 移动系统将所有读取 ActiveGrid 的 Job 注册回来。
        /// 缓冲交换后，旧 ActiveGrid 只有在这些读取完成后才会被重新写入。
        /// </summary>
        public void RegisterActiveGridReader(JobHandle readerHandle)
        {
            if (_activeGridReaders.IsCompleted)
            {
                _activeGridReaders.Complete();
                _activeGridReaders = readerHandle;
                return;
            }

            _activeGridReaders = JobHandle.CombineDependencies(_activeGridReaders, readerHandle);
        }

        protected override void OnUpdate()
        {
            Entity managerEntity = SystemAPI.GetSingletonEntity<FlowFieldSettings>();
            bool requestEnabled =
                EntityManager.IsComponentEnabled<RecalculateFlowFieldTag>(managerEntity);

            if (_isBaking)
            {
                if (!requestEnabled)
                {
                    if (_bakeHandle.IsCompleted)
                    {
                        _bakeHandle.Complete();
                        _isBaking = false;
                    }
                    return;
                }

                TryPublishOrRestart(managerEntity);
                return;
            }

            if (!requestEnabled)
            {
                _waitingForPhysicsWorldRefresh = false;
                return;
            }

            if (!IsCollisionWorldReadyForCostBake(managerEntity))
                return;

            EnsureRuntimeGrid(managerEntity);
            ScheduleBake(managerEntity);
        }

        private bool IsCollisionWorldReadyForCostBake(Entity managerEntity)
        {
            FlowFieldCostState costState =
                EntityManager.GetComponentData<FlowFieldCostState>(managerEntity);
            if (!costState.IsDirty)
            {
                _waitingForPhysicsWorldRefresh = false;
                return true;
            }

            // PhysicsWorldSingleton 可能还没包含 SubScene 异步加载的静态碰撞体。
            // 先观察一次脏请求，等 BuildPhysicsWorld 写入新 singleton 后再采样障碍代价。
            if (!_waitingForPhysicsWorldRefresh)
            {
                _waitingForPhysicsWorldRefresh = true;
                return false;
            }

            Entity physicsWorldEntity =
                SystemAPI.GetSingletonEntity<PhysicsWorldSingleton>();
            ComponentLookup<PhysicsWorldSingleton> physicsWorldLookup =
                GetComponentLookup<PhysicsWorldSingleton>(true);
            if (!physicsWorldLookup.DidChange(physicsWorldEntity, LastSystemVersion))
                return false;

            _waitingForPhysicsWorldRefresh = false;
            return true;
        }

        private void EnsureRuntimeGrid(Entity managerEntity)
        {
            if (EntityManager.HasComponent<FlowFieldGrid>(managerEntity)) return;

            FlowFieldSettings settings = EntityManager.GetComponentData<FlowFieldSettings>(managerEntity);
            int totalCells = settings.GridDimensions.x * settings.GridDimensions.y;
            var runtimeGrid = new FlowFieldGrid
            {
                GridDimensions = settings.GridDimensions,
                CellRadius = settings.CellRadius,
                GridOrigin = settings.GridOrigin,
                Grid = new NativeArray<FlowFieldCell>(totalCells, Allocator.Persistent),
                PendingGrid = new NativeArray<FlowFieldCell>(totalCells, Allocator.Persistent)
            };

            EntityManager.AddComponentData(managerEntity, runtimeGrid);
        }

        private void ScheduleBake(Entity managerEntity)
        {
            FlowFieldGrid grid = EntityManager.GetComponentData<FlowFieldGrid>(managerEntity);
            FlowFieldCostState costState = EntityManager.GetComponentData<FlowFieldCostState>(managerEntity);
            RecalculateFlowFieldTag request =
                EntityManager.GetComponentData<RecalculateFlowFieldTag>(managerEntity);
            float3 targetPosition = EntityManager.GetComponentData<FlowFieldGlobalTarget>(managerEntity).TargetPosition;

            JobHandle prepareDependency = JobHandle.CombineDependencies(Dependency, _pendingReuseHandle);
            var prepareJob = new PreparePendingFlowFieldJob
            {
                ActiveGrid = grid.Grid,
                PendingGrid = grid.PendingGrid
            };
            JobHandle prepareHandle = prepareJob.Schedule(grid.PendingGrid.Length, 64, prepareDependency);

            JobHandle costHandle = prepareHandle;
            if (costState.IsDirty)
            {
                CollisionFilter filter = new CollisionFilter
                {
                    BelongsTo = ~0u,
                    CollidesWith = 1u << 2,
                    GroupIndex = 0
                };

                var costJob = new GenerateCostFieldJob
                {
                    CollisionWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().CollisionWorld,
                    Grid = grid.PendingGrid,
                    GridOrigin = grid.GridOrigin,
                    GridDimensions = grid.GridDimensions,
                    CellRadius = grid.CellRadius,
                    ObstacleFilter = filter
                };
                costHandle = costJob.Schedule(grid.PendingGrid.Length, 64, prepareHandle);
            }

            var queue = new NativeQueue<int2>(Allocator.TempJob);
            var integrationJob = new GenerateIntegrationFieldJob
            {
                Grid = grid.PendingGrid,
                GridDimensions = grid.GridDimensions,
                TargetCell = FlowFieldUtils.WorldToCell(targetPosition, grid.GridOrigin, grid.CellRadius),
                Queue = queue
            };
            JobHandle integrationHandle = integrationJob.Schedule(costHandle);

            var vectorJob = new GenerateVectorFieldJob
            {
                Grid = grid.PendingGrid,
                GridDimensions = grid.GridDimensions
            };
            JobHandle vectorHandle = vectorJob.Schedule(grid.PendingGrid.Length, 64, integrationHandle);

            _bakeHandle = queue.Dispose(vectorHandle);
            _scheduledRequestVersion = request.RequestVersion;
            _scheduledCostDirty = costState.IsDirty;
            _isBaking = true;
            Dependency = _bakeHandle;
        }

        private void TryPublishOrRestart(Entity managerEntity)
        {
            if (!_bakeHandle.IsCompleted) return;

            _bakeHandle.Complete();
            _isBaking = false;

            RecalculateFlowFieldTag currentRequest =
                EntityManager.GetComponentData<RecalculateFlowFieldTag>(managerEntity);
            if (currentRequest.RequestVersion != _scheduledRequestVersion)
            {
                // 计算期间出现了新目标。旧结果不发布，直接重用 PendingGrid 计算最新版。
                ScheduleBake(managerEntity);
                return;
            }

            FlowFieldGrid grid = EntityManager.GetComponentData<FlowFieldGrid>(managerEntity);
            (grid.Grid, grid.PendingGrid) = (grid.PendingGrid, grid.Grid);
            EntityManager.SetComponentData(managerEntity, grid);

            // 旧 ActiveGrid 现在成为 PendingGrid，下一次写入必须等待旧读者完成。
            _pendingReuseHandle = JobHandle.CombineDependencies(_pendingReuseHandle, _activeGridReaders);
            _activeGridReaders = default;

            FlowFieldRuntimeState runtimeState =
                EntityManager.GetComponentData<FlowFieldRuntimeState>(managerEntity);
            runtimeState.ActiveVersion++;
            runtimeState.ActiveRequestVersion = _scheduledRequestVersion;
            EntityManager.SetComponentData(managerEntity, runtimeState);

            if (_scheduledCostDirty)
            {
                FlowFieldCostState costState = EntityManager.GetComponentData<FlowFieldCostState>(managerEntity);
                costState.IsDirty = false;
                costState.CostVersion++;
                EntityManager.SetComponentData(managerEntity, costState);
            }

            EntityManager.SetComponentEnabled<RecalculateFlowFieldTag>(managerEntity, false);
        }

        protected override void OnDestroy()
        {
            _bakeHandle.Complete();
            _activeGridReaders.Complete();
            _pendingReuseHandle.Complete();

            foreach (RefRW<FlowFieldGrid> grid in SystemAPI.Query<RefRW<FlowFieldGrid>>())
            {
                if (grid.ValueRW.Grid.IsCreated) grid.ValueRW.Grid.Dispose();
                if (grid.ValueRW.PendingGrid.IsCreated) grid.ValueRW.PendingGrid.Dispose();
            }
        }
    }
}
