using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;
using RTS.Gameplay.Physics;

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
        private JobHandle _activeEnvironmentReaders;
        private JobHandle _pendingReuseHandle;
        private JobHandle _pendingObstacleReuseHandle;
        private NativeArray<CrowdObstacleCell> _activeObstacleCells;
        private NativeArray<CrowdObstacleCell> _pendingObstacleCells;
        private FlowGridGeometry _obstacleGeometry;
        private uint _obstacleVersion;
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
        /// 移动系统将读取已发布 Navigation/Obstacle field 的 Job 注册回来。
        /// 缓冲交换后，旧 active field 只有在这些读取完成后才会被重新写入。
        /// </summary>
        public void RegisterPublishedEnvironmentReader(JobHandle readerHandle)
        {
            if (_activeEnvironmentReaders.IsCompleted)
            {
                _activeEnvironmentReaders.Complete();
                _activeEnvironmentReaders = readerHandle;
                return;
            }

            _activeEnvironmentReaders = JobHandle.CombineDependencies(
                _activeEnvironmentReaders,
                readerHandle);
        }

        public bool TryGetPublishedObstacleSnapshot(
            out CrowdObstacleSnapshot snapshot)
        {
            if (!_activeObstacleCells.IsCreated || _obstacleVersion == 0)
            {
                snapshot = default;
                return false;
            }

            snapshot = new CrowdObstacleSnapshot(
                _activeObstacleCells,
                _obstacleGeometry,
                _obstacleVersion);
            return true;
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
            FlowFieldSettings settings = EntityManager.GetComponentData<FlowFieldSettings>(managerEntity);
            int totalCells = settings.GridDimensions.x * settings.GridDimensions.y;
            if (!EntityManager.HasComponent<FlowFieldGrid>(managerEntity))
            {
                var runtimeGrid = new FlowFieldGrid
                {
                    GridDimensions = settings.GridDimensions,
                    CellRadius = settings.CellRadius,
                    GridOrigin = settings.GridOrigin,
                    Grid = new NativeArray<FlowFieldCell>(
                        totalCells,
                        Allocator.Persistent),
                    PendingGrid = new NativeArray<FlowFieldCell>(
                        totalCells,
                        Allocator.Persistent)
                };
                EntityManager.AddComponentData(managerEntity, runtimeGrid);
            }

            if (!_activeObstacleCells.IsCreated)
            {
                _activeObstacleCells = new NativeArray<CrowdObstacleCell>(
                    totalCells,
                    Allocator.Persistent,
                    NativeArrayOptions.ClearMemory);
                _pendingObstacleCells = new NativeArray<CrowdObstacleCell>(
                    totalCells,
                    Allocator.Persistent,
                    NativeArrayOptions.ClearMemory);
                _obstacleGeometry = new FlowGridGeometry(
                    settings.GridOrigin,
                    settings.GridDimensions,
                    settings.CellRadius);
            }
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
            JobHandle obstacleHandle = default;
            if (costState.IsDirty)
            {
                CollisionFilter filter =
                    CrowdQueryCollisionFilters.ObstacleOverlap;

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

                var obstacleJob = new GenerateCrowdObstacleFieldJob
                {
                    CollisionWorld = SystemAPI
                        .GetSingleton<PhysicsWorldSingleton>()
                        .CollisionWorld,
                    ObstacleCells = _pendingObstacleCells,
                    GridOrigin = grid.GridOrigin,
                    GridDimensions = grid.GridDimensions,
                    CellRadius = grid.CellRadius,
                    ObstacleFilter = filter
                };
                obstacleHandle = obstacleJob.Schedule(
                    _pendingObstacleCells.Length,
                    64,
                    JobHandle.CombineDependencies(
                        Dependency,
                        _pendingObstacleReuseHandle));
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

            JobHandle navigationBakeHandle = queue.Dispose(vectorHandle);
            _bakeHandle = costState.IsDirty
                ? JobHandle.CombineDependencies(
                    navigationBakeHandle,
                    obstacleHandle)
                : navigationBakeHandle;
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
            JobHandle activeReaders = _activeEnvironmentReaders;
            _pendingReuseHandle = JobHandle.CombineDependencies(
                _pendingReuseHandle,
                activeReaders);

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

                (_activeObstacleCells, _pendingObstacleCells) =
                    (_pendingObstacleCells, _activeObstacleCells);
                _pendingObstacleReuseHandle = JobHandle.CombineDependencies(
                    _pendingObstacleReuseHandle,
                    activeReaders);
                _obstacleVersion = costState.CostVersion;
            }
            _activeEnvironmentReaders = default;

            EntityManager.SetComponentEnabled<RecalculateFlowFieldTag>(managerEntity, false);
        }

        protected override void OnDestroy()
        {
            _bakeHandle.Complete();
            _activeEnvironmentReaders.Complete();
            _pendingReuseHandle.Complete();
            _pendingObstacleReuseHandle.Complete();

            foreach (RefRW<FlowFieldGrid> grid in SystemAPI.Query<RefRW<FlowFieldGrid>>())
            {
                if (grid.ValueRW.Grid.IsCreated) grid.ValueRW.Grid.Dispose();
                if (grid.ValueRW.PendingGrid.IsCreated) grid.ValueRW.PendingGrid.Dispose();
            }
            if (_activeObstacleCells.IsCreated)
                _activeObstacleCells.Dispose();
            if (_pendingObstacleCells.IsCreated)
                _pendingObstacleCells.Dispose();
        }
    }
}
