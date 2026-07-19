using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
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
        RequireForUpdate<UnitSpatialMap>();

        _movementQuery = GetEntityQuery(
            ComponentType.ReadWrite<LocalTransform>(),
            ComponentType.ReadWrite<Velocity>(),
            ComponentType.ReadOnly<UnitMoveSpeed>(),
            ComponentType.ReadOnly<UnitMovementSettings>());
    }

    protected override void OnUpdate()
    {
        var gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();
        var spatialMap = SystemAPI.GetSingleton<UnitSpatialMap>();
        if (!gridComponent.Grid.IsCreated) return;

        int unitCount = _movementQuery.CalculateEntityCount();
        if (unitCount == 0) return;

        // 同一 EntityQuery 的各阶段通过 EntityIndexInQuery 访问相同槽位，
        // 避免把仅在本帧有效的中间状态写回 ECS 组件。
        var states = new NativeArray<FlowMovementFrameState>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.UninitializedMemory);

        // 约束阶段需要按邻居 Entity 查询其预测位置，因此额外建立 Entity -> 预测位置快照。
        var predictedPositions = new NativeParallelHashMap<Entity, float3>(unitCount, Allocator.TempJob);

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
            States = states
        };
        JobHandle independentForceHandle = independentForceJob.ScheduleParallel(_movementQuery, Dependency);

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
            PredictedPositions = predictedPositions.AsParallelWriter(),
            States = states
        };
        JobHandle integrateForcesHandle = integrateForcesJob.ScheduleParallel(_movementQuery, softAvoidanceHandle);

        // 阶段 4：SpatialMap 只负责筛选候选，实际穿透检测使用双方预测位置。
        var constraintJob = new CalculateFlowConstraintsJob
        {
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            SpatialMap = spatialMap.Map,
            PredictedPositions = predictedPositions,
            SeparationRadius = 0.6f,
            States = states
        };
        JobHandle constraintHandle = constraintJob.ScheduleParallel(_movementQuery, integrateForcesHandle);

        // 阶段 5：应用预测位置和约束修正，写回最终 Transform/Velocity。
        var applyMovementJob = new ApplyFlowMovementJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            States = states
        };
        JobHandle applyMovementHandle = applyMovementJob.ScheduleParallel(_movementQuery, constraintHandle);

        // 两个临时容器都必须等最终应用阶段读完后才能释放。
        JobHandle stateDisposeHandle = states.Dispose(applyMovementHandle);
        JobHandle predictedPositionDisposeHandle = predictedPositions.Dispose(applyMovementHandle);
        Dependency = JobHandle.CombineDependencies(stateDisposeHandle, predictedPositionDisposeHandle);
    }
}
