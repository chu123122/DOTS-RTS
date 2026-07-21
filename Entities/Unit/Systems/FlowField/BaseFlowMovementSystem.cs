using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{

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
    private NativeList<AdaptiveFatAabbCellHistory> _adaptiveCellHistory;
    private NativeList<AdaptiveFatAabbRegion> _adaptiveRegions;
    private NativeList<AdaptiveFatAabbDebugCell> _adaptiveDebugCells;
    private NativeList<AdaptiveFatAabbDebugRegion> _adaptiveDebugRegions;
    private NativeList<AdaptiveFatAabbDebugProxy> _adaptiveDebugProxies;
    private NativeList<AdaptiveFatAabbRegionHistory> _adaptiveRegionHistory;
    private NativeReference<int> _adaptiveNextRegionId;
    private NativeReference<AdaptiveFatAabbCacheFeedback> _adaptiveCacheFeedback;
    private NativeList<SimulationDebuggerPairSample> _simulationDebuggerSelectedPairs;
    private NativeReference<SimulationDebuggerUnitSample> _simulationDebuggerSelectedUnit;
    private NativeReference<byte> _simulationDebuggerSelectedUnitValid;
    private int2 _adaptiveCellDimensions;
    private int _adaptiveCellSpan;

    protected override void OnCreate()
    {
        RequireForUpdate<FlowFieldGrid>();
        RequireForUpdate<FlowFieldSettings>();
        RequireForUpdate<FlowFieldRuntimeState>();
        RequireForUpdate<UnitContactSolverSettings>();

        _movementQuery = GetEntityQuery(
            ComponentType.ReadWrite<LocalTransform>(),
            ComponentType.ReadWrite<Velocity>(),
            ComponentType.ReadWrite<FlowArrivalState>(),
            ComponentType.ReadOnly<UnitMoveSpeed>(),
            ComponentType.ReadOnly<UnitMovementSettings>(),
            ComponentType.ReadOnly<UnitContactBody>(),
            ComponentType.ReadOnly<UnitMoveDestination>());

        _shadowPreviousProxies = new NativeList<ShadowFatBodyProxy>(Allocator.Persistent);
        _shadowPreviousPairs = new NativeList<ShadowEntityPair>(Allocator.Persistent);
        _fatAabbCacheState = new NativeReference<FatAabbCacheState>(Allocator.Persistent);
        _adaptiveCellHistory = new NativeList<AdaptiveFatAabbCellHistory>(Allocator.Persistent);
        _adaptiveRegions = new NativeList<AdaptiveFatAabbRegion>(Allocator.Persistent);
        _adaptiveDebugCells = new NativeList<AdaptiveFatAabbDebugCell>(Allocator.Persistent);
        _adaptiveDebugRegions = new NativeList<AdaptiveFatAabbDebugRegion>(Allocator.Persistent);
        _adaptiveDebugProxies = new NativeList<AdaptiveFatAabbDebugProxy>(Allocator.Persistent);
        _adaptiveRegionHistory = new NativeList<AdaptiveFatAabbRegionHistory>(Allocator.Persistent);
        _adaptiveNextRegionId = new NativeReference<int>(Allocator.Persistent);
        _adaptiveNextRegionId.Value = 1;
        _adaptiveCacheFeedback = new NativeReference<AdaptiveFatAabbCacheFeedback>(Allocator.Persistent);
        _simulationDebuggerSelectedPairs =
            new NativeList<SimulationDebuggerPairSample>(64, Allocator.Persistent);
        _simulationDebuggerSelectedUnit =
            new NativeReference<SimulationDebuggerUnitSample>(Allocator.Persistent);
        _simulationDebuggerSelectedUnitValid =
            new NativeReference<byte>(Allocator.Persistent);
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
        if (_adaptiveCellHistory.IsCreated)
            _adaptiveCellHistory.Dispose();
        if (_adaptiveRegions.IsCreated)
            _adaptiveRegions.Dispose();
        if (_adaptiveDebugCells.IsCreated)
            _adaptiveDebugCells.Dispose();
        if (_adaptiveDebugRegions.IsCreated)
            _adaptiveDebugRegions.Dispose();
        if (_adaptiveDebugProxies.IsCreated)
            _adaptiveDebugProxies.Dispose();
        if (_adaptiveRegionHistory.IsCreated)
            _adaptiveRegionHistory.Dispose();
        if (_adaptiveNextRegionId.IsCreated)
            _adaptiveNextRegionId.Dispose();
        if (_adaptiveCacheFeedback.IsCreated)
            _adaptiveCacheFeedback.Dispose();
        if (_simulationDebuggerSelectedPairs.IsCreated)
            _simulationDebuggerSelectedPairs.Dispose();
        if (_simulationDebuggerSelectedUnit.IsCreated)
            _simulationDebuggerSelectedUnit.Dispose();
        if (_simulationDebuggerSelectedUnitValid.IsCreated)
            _simulationDebuggerSelectedUnitValid.Dispose();
    }

    protected override void OnUpdate()
    {
        var gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();
        var flowFieldSettings = SystemAPI.GetSingleton<FlowFieldSettings>();
        var flowFieldRuntimeState = SystemAPI.GetSingleton<FlowFieldRuntimeState>();
        var contactSolverSettings = SystemAPI.GetSingleton<UnitContactSolverSettings>();
        bool hasAdaptiveSettings =
            SystemAPI.TryGetSingleton(out AdaptiveFatAabbSettings configuredAdaptiveSettings);
        AdaptiveFatAabbSettings adaptiveSettings = hasAdaptiveSettings
            ? configuredAdaptiveSettings
            : AdaptiveFatAabbSettings.Default;
        adaptiveSettings = adaptiveSettings.Sanitized();
        ApplySimulationDebuggerRuntimeOverrides(
            ref flowFieldSettings,
            ref contactSolverSettings,
            ref adaptiveSettings,
            hasAdaptiveSettings);
        EnsureAdaptiveFatAabbHistory(gridComponent.GridDimensions, adaptiveSettings);
        PublishSimulationDebuggerSnapshot(gridComponent, contactSolverSettings);
        DrawAdaptiveFatAabbDebug(adaptiveSettings);
        Entity diagnosticSelectedEntity = Entity.Null;
        if (SystemAPI.TryGetSingleton(out Stage3ContactDiagnosticSelection diagnosticSelection))
            diagnosticSelectedEntity = diagnosticSelection.SelectedEntity;
        if (!gridComponent.Grid.IsCreated) return;
        if (flowFieldRuntimeState.ActiveVersion == 0) return;

        int unitCount = _movementQuery.CalculateEntityCount();
        if (unitCount == 0) return;

        // 同一 EntityQuery 的各阶段通过 EntityIndexInQuery 访问相同槽位，
        // 避免把仅在本帧有效的中间状态写回 ECS 组件。
        var states = new NativeArray<FlowMovementFrameState>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.UninitializedMemory);

        // 碰撞体投影用于接触半径。目标槽位已经在移动订单到来时固定分配，
        // 不再逐帧估算一块共享到达区域。
        var collisionFootprints = new NativeArray<float2>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.UninitializedMemory);

        var physicsColliderLookup = SystemAPI.GetComponentLookup<PhysicsCollider>(isReadOnly: true);
        physicsColliderLookup.Update(this);

        var footprintJob = new CalculateUnitCollisionFootprintJob
        {
            PhysicsColliderLookup = physicsColliderLookup,
            FallbackCellSize = gridComponent.CellRadius * 2f,
            CollisionFootprints = collisionFootprints
        };
        JobHandle footprintHandle = footprintJob.ScheduleParallel(_movementQuery, Dependency);

        // 阶段 1：只计算流场、到达状态等不依赖其他单位的力。
        var independentForceJob = new CalculateIndependentFlowForceJob
        {
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            ActiveRequestVersion = flowFieldRuntimeState.ActiveRequestVersion,
            CollisionFootprints = collisionFootprints,
            States = states
        };
        JobHandle independentForceHandle =
            independentForceJob.ScheduleParallel(_movementQuery, footprintHandle);

        // 阶段 2：每个 substep 先按最新求解位置重算软避让，再生成 swept disc Pair，
        // 随后的全部 XPBD iteration 复用该 Pair 快照。
        var sweptCellEntries = new NativeList<SweptDiscCellEntry>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
        var collisionPairs = new NativeList<UnitCollisionPair>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
        var timestepContactPairs = new NativeList<UnitCollisionPair>(
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
        int adaptiveCellCount = math.max(1, _adaptiveCellDimensions.x * _adaptiveCellDimensions.y);
        var adaptiveCellMetrics = new NativeArray<AdaptiveFatAabbCellMetric>(
            adaptiveCellCount,
            Allocator.TempJob,
            NativeArrayOptions.ClearMemory);
        var adaptiveBodyRouting = new NativeArray<AdaptiveFatAabbBodyRouting>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.ClearMemory);
        var adaptiveFloodQueue = new NativeList<int>(adaptiveCellCount, Allocator.TempJob);
        var adaptiveFloodCells = new NativeList<int>(adaptiveCellCount, Allocator.TempJob);
        var adaptiveRegionHistoryScratch =
            new NativeList<AdaptiveFatAabbRegionHistory>(adaptiveCellCount, Allocator.TempJob);
        var heatSamples = new NativeArray<Stage3ContactHeatSample>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.ClearMemory);
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
            SoftAvoidanceVelocitySolver = flowFieldSettings.SoftAvoidanceVelocitySolver,
            RvoTimeHorizon = flowFieldSettings.RvoTimeHorizon,
            EnablePredictiveContacts = contactSolverSettings.EnablePredictiveContacts,
            EnableDiagnostics = contactSolverSettings.EnableDiagnostics,
            EnableFatAabbCache = contactSolverSettings.EnableFatAabbCache,
            FatAabbCacheMargin = contactSolverSettings.FatAabbCacheMargin,
            AdaptiveSettings = adaptiveSettings,
            AdaptiveCellDimensions = _adaptiveCellDimensions,
            TimestepContactMargin = contactSolverSettings.TimestepContactMargin,
            DiagnosticSelectedEntity = diagnosticSelectedEntity,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            Grid = gridComponent.Grid,
            SweptCellEntries = sweptCellEntries,
            Pairs = collisionPairs,
            TimestepContactPairs = timestepContactPairs,
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
            AdaptiveCellHistory = _adaptiveCellHistory.AsArray(),
            AdaptiveCellMetrics = adaptiveCellMetrics,
            AdaptiveBodyRouting = adaptiveBodyRouting,
            AdaptiveFloodQueue = adaptiveFloodQueue,
            AdaptiveFloodCells = adaptiveFloodCells,
            AdaptiveRegions = _adaptiveRegions,
            AdaptiveDebugCells = _adaptiveDebugCells,
            AdaptiveDebugRegions = _adaptiveDebugRegions,
            AdaptiveDebugProxies = _adaptiveDebugProxies,
            AdaptiveRegionHistory = _adaptiveRegionHistory,
            AdaptiveRegionHistoryScratch = adaptiveRegionHistoryScratch,
            AdaptiveNextRegionId = _adaptiveNextRegionId,
            AdaptiveCacheFeedback = _adaptiveCacheFeedback,
            SimulationDebuggerCaptureMask = SimulationDebuggerRuntime.CaptureMask,
            SimulationDebuggerMaximumPairs = SimulationDebuggerRuntime.MaximumVisualizedPairs,
            SimulationDebuggerSelectedPairs = _simulationDebuggerSelectedPairs,
            SimulationDebuggerSelectedUnit = _simulationDebuggerSelectedUnit,
            SimulationDebuggerSelectedUnitValid = _simulationDebuggerSelectedUnitValid,
            States = states,
            Statistics = contactStatistics,
            ShadowStatistics = shadowStatistics,
            IterationDiagnostics = iterationDiagnostics,
            PairDiagnostics = pairDiagnostics,
            SelectedBodyDiagnostic = selectedBodyDiagnostic,
            HeatSamples = heatSamples
        };
        JobHandle solveContactHandle = solveContactJob.Schedule(independentForceHandle);

        var publishStatisticsJob = new PublishPredictiveDiscContactStatisticsJob
        {
            Source = contactStatistics,
            ShadowSource = shadowStatistics,
            SelectedBodySource = selectedBodyDiagnostic,
            IterationSource = iterationDiagnostics,
            PairSource = pairDiagnostics,
            HeatSource = heatSamples
        };
        JobHandle publishStatisticsHandle =
            publishStatisticsJob.Schedule(solveContactHandle);

        // FlowField 使用双缓冲。发布后旧 ActiveGrid 会成为下一次 PendingGrid，
        // 因此必须把本帧最后一个网格读取句柄注册给 BakeSystem。
        World.GetExistingSystemManaged<FlowFieldBakeSystem>()
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
        JobHandle sweptEntryDisposeHandle = sweptCellEntries.Dispose(applyMovementHandle);
        JobHandle collisionPairDisposeHandle = collisionPairs.Dispose(applyMovementHandle);
        JobHandle timestepContactPairDisposeHandle =
            timestepContactPairs.Dispose(applyMovementHandle);
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
        JobHandle adaptiveCellMetricDisposeHandle = adaptiveCellMetrics.Dispose(applyMovementHandle);
        JobHandle adaptiveBodyRoutingDisposeHandle = adaptiveBodyRouting.Dispose(applyMovementHandle);
        JobHandle adaptiveFloodQueueDisposeHandle = adaptiveFloodQueue.Dispose(applyMovementHandle);
        JobHandle adaptiveFloodCellsDisposeHandle = adaptiveFloodCells.Dispose(applyMovementHandle);
        JobHandle adaptiveRegionHistoryScratchDisposeHandle =
            adaptiveRegionHistoryScratch.Dispose(applyMovementHandle);
        JobHandle heatSampleDisposeHandle = heatSamples.Dispose(publishStatisticsHandle);
        JobHandle frameStateDisposeHandle = JobHandle.CombineDependencies(
            stateDisposeHandle,
            footprintDisposeHandle);
        JobHandle lookupDisposeHandle = JobHandle.CombineDependencies(
            statisticsDisposeHandle,
            shadowStatisticsDisposeHandle);
        JobHandle diagnosticDisposeHandle = JobHandle.CombineDependencies(
            selectedDiagnosticDisposeHandle,
            iterationDiagnosticDisposeHandle,
            pairDiagnosticDisposeHandle);
        diagnosticDisposeHandle = JobHandle.CombineDependencies(
            diagnosticDisposeHandle,
            heatSampleDisposeHandle);
        JobHandle contactPairDisposeHandle = JobHandle.CombineDependencies(
            collisionPairDisposeHandle,
            timestepContactPairDisposeHandle);
        JobHandle broadPhaseDisposeHandle = JobHandle.CombineDependencies(
            sweptEntryDisposeHandle,
            contactPairDisposeHandle,
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
        JobHandle adaptiveMetricDisposeHandle = JobHandle.CombineDependencies(
            adaptiveCellMetricDisposeHandle,
            adaptiveBodyRoutingDisposeHandle);
        JobHandle adaptiveFloodDisposeHandle = JobHandle.CombineDependencies(
            adaptiveFloodQueueDisposeHandle,
            adaptiveFloodCellsDisposeHandle,
            adaptiveRegionHistoryScratchDisposeHandle);
        JobHandle adaptiveScratchDisposeHandle = JobHandle.CombineDependencies(
            adaptiveMetricDisposeHandle,
            adaptiveFloodDisposeHandle);
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
        mainDisposeHandle = JobHandle.CombineDependencies(
            mainDisposeHandle,
            adaptiveScratchDisposeHandle);
        Dependency = JobHandle.CombineDependencies(
            mainDisposeHandle,
            diagnosticDisposeHandle);
    }
}
}
