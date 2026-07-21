using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using 通用;

/// <summary>
/// 单位流场移动的分阶段调度基类。
/// 每帧依次计算驱动力、生成唯一动态接触 Pair、执行 XPBD substep/iteration，并写回位姿。
/// </summary>
public abstract partial class BaseFlowMovementSystem : SystemBase
{
    private EntityQuery _movementQuery;
    private NativeList<ShadowFatBodyProxy> _shadowPreviousProxies;
    private NativeList<ShadowEntityPair> _shadowPreviousPairs;
    private NativeReference<FatAabbCacheState> _fatAabbCacheState;

    protected override void OnCreate()
    {
        RequireForUpdate<FlowFieldGrid>();
        RequireForUpdate<FlowFieldRuntimeState>();
        RequireForUpdate<UnitContactSolverSettings>();
        RequireForUpdate<PredictiveDiscContactStatistics>();
        RequireForUpdate<ShadowNeighborCacheStatistics>();
        RequireForUpdate<Stage3ContactDiagnosticSelection>();

        _movementQuery = GetEntityQuery(
            ComponentType.ReadWrite<LocalTransform>(),
            ComponentType.ReadWrite<Velocity>(),
            ComponentType.ReadWrite<FlowArrivalState>(),
            ComponentType.ReadOnly<UnitMoveSpeed>(),
            ComponentType.ReadOnly<UnitMovementSettings>(),
            ComponentType.ReadOnly<UnitContactBody>());

        _shadowPreviousProxies = new NativeList<ShadowFatBodyProxy>(Allocator.Persistent);
        _shadowPreviousPairs = new NativeList<ShadowEntityPair>(Allocator.Persistent);
        _fatAabbCacheState = new NativeReference<FatAabbCacheState>(Allocator.Persistent);
    }

    protected override void OnDestroy()
    {
        Dependency.Complete();
        if (_shadowPreviousProxies.IsCreated)
            _shadowPreviousProxies.Dispose();
        if (_shadowPreviousPairs.IsCreated)
            _shadowPreviousPairs.Dispose();
        if (_fatAabbCacheState.IsCreated)
            _fatAabbCacheState.Dispose();
    }

    protected override void OnUpdate()
    {
        var gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();
        var flowFieldSettings = SystemAPI.GetSingleton<FlowFieldSettings>();
        var contactSolverSettings = SystemAPI.GetSingleton<UnitContactSolverSettings>();
        var diagnosticSelection = SystemAPI.GetSingleton<Stage3ContactDiagnosticSelection>();
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

        // 阶段 1：只计算流场、到达状态等不依赖其他单位的力。
        var independentForceJob = new CalculateIndependentFlowForceJob
        {
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            ArrivalEnterDistance = arrivalEnterDistance,
            CollisionFootprints = collisionFootprints,
            States = states
        };
        JobHandle independentForceHandle = independentForceJob.ScheduleParallel(_movementQuery, arrivalAreaHandle);

        // 阶段 2：每个 substep 先按最新求解位置重算软避让，再生成 swept disc Pair，
        // 随后的全部 XPBD iteration 复用该 Pair 快照。
        var sweptCellEntries = new NativeList<SweptDiscCellEntry>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
        var collisionPairs = new NativeList<UnitCollisionPair>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
        var shadowCellEntries = new NativeList<SweptDiscCellEntry>(
            math.max(unitCount * 8, 1),
            Allocator.TempJob);
        var shadowBodyPairs = new NativeList<UnitCollisionPair>(
            math.max(unitCount * 8, 1),
            Allocator.TempJob);
        var shadowCurrentProxies = new NativeList<ShadowFatBodyProxy>(
            math.max(unitCount, 1),
            Allocator.TempJob);
        var shadowCurrentPairs = new NativeList<ShadowEntityPair>(
            math.max(unitCount * 8, 1),
            Allocator.TempJob);
        var currentBodyIndexByEntity = new NativeParallelHashMap<Entity, int>(
            math.max(unitCount, 1),
            Allocator.TempJob);
        var mappedFatCachePairs = new NativeList<UnitCollisionPair>(
            math.max(unitCount * 8, 1),
            Allocator.TempJob);
        var correctedBodyFlags = new NativeArray<byte>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.ClearMemory);
        var correctedBodyIndices = new NativeList<int>(
            math.max(unitCount, 1),
            Allocator.TempJob);
        var contactStatistics =
            new NativeReference<PredictiveDiscContactStatistics>(Allocator.TempJob);
        var shadowStatistics =
            new NativeReference<ShadowNeighborCacheStatistics>(Allocator.TempJob);
        var iterationDiagnostics = new NativeList<Stage3ContactIterationDiagnostic>(
            math.max(contactSolverSettings.SubstepCount * contactSolverSettings.IterationCount, 1),
            Allocator.TempJob);
        var pairDiagnostics = new NativeList<Stage3ContactPairDiagnostic>(
            math.max(unitCount * 2, 1),
            Allocator.TempJob);
        var selectedBodyDiagnostic =
            new NativeReference<Stage3SelectedBodyDiagnostic>(Allocator.TempJob);
        var solveContactJob = new SolveXpbdUnitContactsJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            SubstepCount = contactSolverSettings.SubstepCount,
            IterationCount = contactSolverSettings.IterationCount,
            Compliance = contactSolverSettings.Compliance,
            PredictiveSkin = contactSolverSettings.PredictiveSkin,
            EnablePredictivePairGeneration = contactSolverSettings.EnablePredictivePairGeneration,
            SoftAvoidanceResponseRate = flowFieldSettings.SoftAvoidanceResponseRate,
            SoftAvoidanceShell = flowFieldSettings.SoftAvoidanceShell,
            SettledSoftAvoidanceMultiplier = flowFieldSettings.SettledSoftAvoidanceMultiplier,
            EnablePredictiveContacts = contactSolverSettings.EnablePredictiveContacts,
            EnableDiagnostics = contactSolverSettings.EnableDiagnostics,
            EnableFatAabbCache = contactSolverSettings.EnableFatAabbCache,
            FatAabbCacheMargin = contactSolverSettings.FatAabbCacheMargin,
            DiagnosticSelectedEntity = diagnosticSelection.SelectedEntity,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            Grid = gridComponent.Grid,
            SweptCellEntries = sweptCellEntries,
            Pairs = collisionPairs,
            ShadowCellEntries = shadowCellEntries,
            ShadowBodyPairs = shadowBodyPairs,
            ShadowCurrentProxies = shadowCurrentProxies,
            ShadowCurrentPairs = shadowCurrentPairs,
            CurrentBodyIndexByEntity = currentBodyIndexByEntity,
            MappedFatCachePairs = mappedFatCachePairs,
            CorrectedBodyFlags = correctedBodyFlags,
            CorrectedBodyIndices = correctedBodyIndices,
            ShadowPreviousProxies = _shadowPreviousProxies,
            ShadowPreviousPairs = _shadowPreviousPairs,
            FatAabbCacheState = _fatAabbCacheState,
            States = states,
            Statistics = contactStatistics,
            ShadowStatistics = shadowStatistics,
            IterationDiagnostics = iterationDiagnostics,
            PairDiagnostics = pairDiagnostics,
            SelectedBodyDiagnostic = selectedBodyDiagnostic
        };
        JobHandle solveContactHandle = solveContactJob.Schedule(independentForceHandle);

        var publishStatisticsJob = new PublishPredictiveDiscContactStatisticsJob
        {
            Source = contactStatistics,
            ShadowSource = shadowStatistics,
            SelectedBodySource = selectedBodyDiagnostic,
            IterationSource = iterationDiagnostics,
            PairSource = pairDiagnostics
        };
        JobHandle publishStatisticsHandle =
            publishStatisticsJob.Schedule(solveContactHandle);

        // FlowField 使用双缓冲。发布后旧 ActiveGrid 会成为下一次 PendingGrid，
        // 因此必须把本帧最后一个网格读取句柄注册给 BakeSystem。
        World.GetExistingSystemManaged<Entities.Unit.System.FlowFieldSystem.FlowFieldBakeSystem>()
            ?.RegisterActiveGridReader(solveContactHandle);

        // 阶段 5：应用预测位置和约束修正，写回最终 Transform/Velocity。
        var applyMovementJob = new ApplyFlowMovementJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            States = states
        };
        JobHandle applyMovementHandle =
            applyMovementJob.ScheduleParallel(_movementQuery, solveContactHandle);

        // 所有临时容器都必须等最终应用阶段读完后才能释放。
        JobHandle stateDisposeHandle = states.Dispose(applyMovementHandle);
        JobHandle footprintDisposeHandle = collisionFootprints.Dispose(applyMovementHandle);
        JobHandle arrivalDistanceDisposeHandle = arrivalEnterDistance.Dispose(applyMovementHandle);
        JobHandle sweptEntryDisposeHandle = sweptCellEntries.Dispose(applyMovementHandle);
        JobHandle collisionPairDisposeHandle = collisionPairs.Dispose(applyMovementHandle);
        JobHandle shadowCellDisposeHandle = shadowCellEntries.Dispose(applyMovementHandle);
        JobHandle shadowBodyPairDisposeHandle = shadowBodyPairs.Dispose(applyMovementHandle);
        JobHandle shadowProxyDisposeHandle = shadowCurrentProxies.Dispose(applyMovementHandle);
        JobHandle shadowPairDisposeHandle = shadowCurrentPairs.Dispose(applyMovementHandle);
        JobHandle currentBodyIndexDisposeHandle =
            currentBodyIndexByEntity.Dispose(applyMovementHandle);
        JobHandle mappedFatCachePairDisposeHandle =
            mappedFatCachePairs.Dispose(applyMovementHandle);
        JobHandle correctedBodyFlagDisposeHandle =
            correctedBodyFlags.Dispose(applyMovementHandle);
        JobHandle correctedBodyIndexDisposeHandle =
            correctedBodyIndices.Dispose(applyMovementHandle);
        JobHandle statisticsDisposeHandle = contactStatistics.Dispose(publishStatisticsHandle);
        JobHandle shadowStatisticsDisposeHandle = shadowStatistics.Dispose(publishStatisticsHandle);
        JobHandle selectedDiagnosticDisposeHandle =
            selectedBodyDiagnostic.Dispose(publishStatisticsHandle);
        JobHandle iterationDiagnosticDisposeHandle =
            iterationDiagnostics.Dispose(publishStatisticsHandle);
        JobHandle pairDiagnosticDisposeHandle = pairDiagnostics.Dispose(publishStatisticsHandle);
        JobHandle frameStateDisposeHandle = JobHandle.CombineDependencies(
            stateDisposeHandle,
            footprintDisposeHandle);
        JobHandle lookupDisposeHandle = JobHandle.CombineDependencies(
            arrivalDistanceDisposeHandle,
            statisticsDisposeHandle,
            shadowStatisticsDisposeHandle);
        JobHandle diagnosticDisposeHandle = JobHandle.CombineDependencies(
            selectedDiagnosticDisposeHandle,
            iterationDiagnosticDisposeHandle,
            pairDiagnosticDisposeHandle);
        JobHandle broadPhaseDisposeHandle = JobHandle.CombineDependencies(
            sweptEntryDisposeHandle,
            collisionPairDisposeHandle,
            shadowCellDisposeHandle);
        JobHandle pairDisposeHandle = JobHandle.CombineDependencies(
            broadPhaseDisposeHandle,
            shadowBodyPairDisposeHandle);
        JobHandle shadowCacheDisposeHandle = JobHandle.CombineDependencies(
            shadowProxyDisposeHandle,
            shadowPairDisposeHandle);
        JobHandle mappingScratchDisposeHandle = JobHandle.CombineDependencies(
            currentBodyIndexDisposeHandle,
            mappedFatCachePairDisposeHandle);
        JobHandle correctionScratchDisposeHandle = JobHandle.CombineDependencies(
            correctedBodyFlagDisposeHandle,
            correctedBodyIndexDisposeHandle);
        JobHandle fatCacheScratchDisposeHandle = JobHandle.CombineDependencies(
            mappingScratchDisposeHandle,
            correctionScratchDisposeHandle);
        JobHandle mainDisposeWithoutShadowCache = JobHandle.CombineDependencies(
            frameStateDisposeHandle,
            lookupDisposeHandle,
            pairDisposeHandle);
        JobHandle mainAndShadowDisposeHandle = JobHandle.CombineDependencies(
            mainDisposeWithoutShadowCache,
            shadowCacheDisposeHandle);
        JobHandle mainDisposeHandle = JobHandle.CombineDependencies(
            mainAndShadowDisposeHandle,
            fatCacheScratchDisposeHandle);
        Dependency = JobHandle.CombineDependencies(
            mainDisposeHandle,
            diagnosticDisposeHandle);
    }
}
