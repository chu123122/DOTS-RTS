using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using 通用;

/// <summary>
/// 单位流场移动的分阶段调度基类。
/// 每帧依次计算独立流场力、软避让力、半隐式欧拉预测位置、位置约束和最终位姿。
/// </summary>
public abstract partial class BaseFlowMovementSystem : SystemBase
{
    private EntityQuery _movementQuery;

    protected override void OnCreate()
    {
        RequireForUpdate<FlowFieldGrid>();
        RequireForUpdate<FlowFieldRuntimeState>();
        RequireForUpdate<UnitSpatialMap>();

        _movementQuery = GetEntityQuery(
            ComponentType.ReadWrite<LocalTransform>(),
            ComponentType.ReadWrite<Velocity>(),
            ComponentType.ReadWrite<FlowArrivalState>(),
            ComponentType.ReadOnly<UnitMoveSpeed>(),
            ComponentType.ReadOnly<UnitMovementSettings>());
    }

    protected override void OnUpdate()
    {
        var gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();
        var spatialMap = SystemAPI.GetSingleton<UnitSpatialMap>();
        if (!gridComponent.Grid.IsCreated) return;
        if (SystemAPI.GetSingleton<FlowFieldRuntimeState>().ActiveVersion == 0) return;

        int unitCount = _movementQuery.CalculateEntityCount();
        if (unitCount == 0) return;

        // 同一 EntityQuery 的各阶段通过 EntityIndexInQuery 访问相同槽位，
        // 避免把仅在本帧有效的中间状态写回 ECS 组件。
        var states = new NativeArray<FlowMovementFrameState>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.UninitializedMemory);

        // 到达区域容量按每个单位当前 PhysicsCollider 的 XZ 投影计算。
        // 缩放和旋转均来自本帧 LocalTransform，不再假设每个单位固定占一个格子。
        var collisionFootprints = new NativeArray<float2>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.UninitializedMemory);
        var arrivalEnterDistance = new NativeReference<int>(Allocator.TempJob);

        // Pair 生成阶段需要把 Spatial Hash 中的 Entity 映射到本帧稠密状态索引。
        var entityToIndex = new NativeParallelHashMap<Entity, int>(unitCount, Allocator.TempJob);

        var physicsColliderLookup = SystemAPI.GetComponentLookup<PhysicsCollider>(isReadOnly: true);
        physicsColliderLookup.Update(this);

        var footprintJob = new CalculateUnitCollisionFootprintJob
        {
            PhysicsColliderLookup = physicsColliderLookup,
            FallbackCellSize = gridComponent.CellRadius * 2f,
            CollisionFootprints = collisionFootprints
        };
        JobHandle footprintHandle = footprintJob.ScheduleParallel(_movementQuery, Dependency);

        var arrivalAreaJob = new CalculateArrivalAreaJob
        {
            CollisionFootprints = collisionFootprints,
            CellSize = gridComponent.CellRadius * 2f,
            ArrivalEnterDistance = arrivalEnterDistance
        };
        JobHandle arrivalAreaHandle = arrivalAreaJob.Schedule(footprintHandle);

        // 软避让仍使用当前帧真实位置；预测位置会在所有软力计算完成后统一生成。
        var transformLookup = SystemAPI.GetComponentLookup<LocalTransform>(isReadOnly: true);
        transformLookup.Update(this);

        // 阶段 1：只计算流场、到达减速等不依赖其他单位的力。
        var independentForceJob = new CalculateIndependentFlowForceJob
        {
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            ArrivalEnterDistance = arrivalEnterDistance,
            EntityToIndex = entityToIndex.AsParallelWriter(),
            States = states
        };
        JobHandle independentForceHandle = independentForceJob.ScheduleParallel(_movementQuery, arrivalAreaHandle);

        // 阶段 2：基于当前 SpatialMap 和当前位置累计单位/墙壁软避让力。
        var softAvoidanceJob = new CalculateSoftAvoidanceJob
        {
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            SpatialMap = spatialMap.Map,
            TransformLookup = transformLookup,
            SeparationWeight = 4f,
            SeparationRadius = 0.6f,
            States = states
        };
        JobHandle softAvoidanceHandle = softAvoidanceJob.ScheduleParallel(_movementQuery, independentForceHandle);

        // 阶段 3：合力积分得到速度，并为所有单位生成同一时刻的预测位置快照。
        var integrateForcesJob = new IntegrateFlowForcesJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            GridOrigin = gridComponent.GridOrigin,
            CellRadius = gridComponent.CellRadius,
            States = states
        };
        JobHandle integrateForcesHandle = integrateForcesJob.ScheduleParallel(_movementQuery, softAvoidanceHandle);

        // 阶段 4：从 SpatialMap 并行生成唯一单位 Pair 快照。
        var pairStream = new NativeStream(unitCount, Allocator.TempJob);
        var collisionPairs = new NativeList<UnitCollisionPair>(math.max(unitCount * 4, 1), Allocator.TempJob);
        var buildPairJob = new BuildUniqueUnitCollisionPairsJob
        {
            Grid = gridComponent.Grid,
            GridDimensions = gridComponent.GridDimensions,
            SpatialMap = spatialMap.Map,
            EntityToIndex = entityToIndex,
            States = states,
            PairWriter = pairStream.AsWriter()
        };
        JobHandle buildPairHandle = buildPairJob.ScheduleParallel(_movementQuery, integrateForcesHandle);

        var collectPairJob = new CollectUnitCollisionPairsJob
        {
            PairReader = pairStream.AsReader(),
            Pairs = collisionPairs
        };
        JobHandle collectPairHandle = collectPairJob.Schedule(buildPairHandle);

        // 墙壁仍保留原有单体位置投影，与单位 Pair 快照相互独立。
        var wallConstraintJob = new CalculateWallConstraintsJob
        {
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            States = states
        };
        JobHandle wallConstraintHandle =
            wallConstraintJob.ScheduleParallel(_movementQuery, integrateForcesHandle);

        var solvePairJob = new SolveUnitCollisionPairsJob
        {
            Pairs = collisionPairs.AsDeferredJobArray(),
            CollisionFootprints = collisionFootprints,
            States = states
        };
        JobHandle constraintHandle = solvePairJob.Schedule(
            JobHandle.CombineDependencies(collectPairHandle, wallConstraintHandle));

        // FlowField 使用双缓冲。发布后旧 ActiveGrid 会成为下一次 PendingGrid，
        // 因此必须把本帧最后一个网格读取句柄注册给 BakeSystem。
        World.GetExistingSystemManaged<Entities.Unit.System.FlowFieldSystem.FlowFieldBakeSystem>()
            ?.RegisterActiveGridReader(constraintHandle);

        // 阶段 5：应用预测位置和约束修正，写回最终 Transform/Velocity。
        var applyMovementJob = new ApplyFlowMovementJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            States = states
        };
        JobHandle applyMovementHandle = applyMovementJob.ScheduleParallel(_movementQuery, constraintHandle);

        // 所有临时容器都必须等最终应用阶段读完后才能释放。
        JobHandle stateDisposeHandle = states.Dispose(applyMovementHandle);
        JobHandle footprintDisposeHandle = collisionFootprints.Dispose(applyMovementHandle);
        JobHandle arrivalDistanceDisposeHandle = arrivalEnterDistance.Dispose(applyMovementHandle);
        JobHandle entityIndexDisposeHandle = entityToIndex.Dispose(applyMovementHandle);
        JobHandle pairStreamDisposeHandle = pairStream.Dispose(applyMovementHandle);
        JobHandle collisionPairDisposeHandle = collisionPairs.Dispose(applyMovementHandle);
        JobHandle frameStateDisposeHandle = JobHandle.CombineDependencies(
            stateDisposeHandle,
            footprintDisposeHandle);
        JobHandle lookupDisposeHandle = JobHandle.CombineDependencies(
            arrivalDistanceDisposeHandle,
            entityIndexDisposeHandle);
        JobHandle pairDisposeHandle = JobHandle.CombineDependencies(
            pairStreamDisposeHandle,
            collisionPairDisposeHandle);
        Dependency = JobHandle.CombineDependencies(
            frameStateDisposeHandle,
            lookupDisposeHandle,
            pairDisposeHandle);
    }
}
