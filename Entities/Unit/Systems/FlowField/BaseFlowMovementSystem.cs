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
    private NativeList<PersistentSweptProxy> _persistentSweptProxies;
    private NativeList<PersistentNeighborPair> _persistentNeighborPairs;
    private NativeList<PersistentPredictiveContact> _persistentPredictiveContacts;
    private NativeList<StableEntityPairKey> _persistentActiveContactKeys;
    private NativeList<StableEntityPairKey> _persistentSoftAvoidancePairKeys;
    private NativeList<PredictiveContactScheduleEntry> _persistentDormantContactSchedule;
    private NativeReference<IncrementalContactCacheState> _incrementalContactCacheState;
    private NativeList<SimulationDebuggerPairSample> _simulationDebuggerSelectedPairs;
    private NativeReference<SimulationDebuggerUnitSample> _simulationDebuggerSelectedUnit;
    private NativeReference<byte> _simulationDebuggerSelectedUnitValid;
    private Entity _incrementalDiagnosticsEntity;

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
        _persistentSweptProxies = new NativeList<PersistentSweptProxy>(Allocator.Persistent);
        _persistentNeighborPairs = new NativeList<PersistentNeighborPair>(Allocator.Persistent);
        _persistentPredictiveContacts =
            new NativeList<PersistentPredictiveContact>(Allocator.Persistent);
        _persistentActiveContactKeys =
            new NativeList<StableEntityPairKey>(Allocator.Persistent);
        _persistentSoftAvoidancePairKeys =
            new NativeList<StableEntityPairKey>(Allocator.Persistent);
        _persistentDormantContactSchedule =
            new NativeList<PredictiveContactScheduleEntry>(Allocator.Persistent);
        _incrementalContactCacheState =
            new NativeReference<IncrementalContactCacheState>(Allocator.Persistent);
        _simulationDebuggerSelectedPairs =
            new NativeList<SimulationDebuggerPairSample>(64, Allocator.Persistent);
        _simulationDebuggerSelectedUnit =
            new NativeReference<SimulationDebuggerUnitSample>(Allocator.Persistent);
        _simulationDebuggerSelectedUnitValid =
            new NativeReference<byte>(Allocator.Persistent);
        _incrementalDiagnosticsEntity = EntityManager.CreateEntity(
            typeof(IncrementalContactPipelineSnapshot));
    }

    protected override void OnDestroy()
    {
        Dependency.Complete();
        if (EntityManager.Exists(_incrementalDiagnosticsEntity))
            EntityManager.DestroyEntity(_incrementalDiagnosticsEntity);
        if (_persistentSweptProxies.IsCreated)
            _persistentSweptProxies.Dispose();
        if (_persistentNeighborPairs.IsCreated)
            _persistentNeighborPairs.Dispose();
        if (_persistentPredictiveContacts.IsCreated)
            _persistentPredictiveContacts.Dispose();
        if (_persistentActiveContactKeys.IsCreated)
            _persistentActiveContactKeys.Dispose();
        if (_persistentSoftAvoidancePairKeys.IsCreated)
            _persistentSoftAvoidancePairKeys.Dispose();
        if (_persistentDormantContactSchedule.IsCreated)
            _persistentDormantContactSchedule.Dispose();
        if (_incrementalContactCacheState.IsCreated)
            _incrementalContactCacheState.Dispose();
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

        // 先发布上一时间步已经完成的统计，再应用下一时间步的实验配置。
        // 这样面板中的 ExperimentId、有效配置和求解结果始终属于同一帧。
        PublishSimulationDebuggerSnapshot(gridComponent, contactSolverSettings);
        ApplySimulationDebuggerRuntimeOverrides(
            ref flowFieldSettings,
            ref contactSolverSettings);
        IncrementalContactPipelineExperimentRuntime.Apply(ref contactSolverSettings);
        bool effectiveTimestepContactSetCache =
            IncrementalContactPipelineExperimentRuntime.OverrideEnabled
                ? IncrementalContactPipelineExperimentRuntime.TimestepCacheEnabled
                : SimulationDebuggerRuntime.TimestepContactSetCacheEnabled;
        bool requestedPersistentContactCache =
            IncrementalContactPipelineExperimentRuntime.OverrideEnabled
                ? IncrementalContactPipelineExperimentRuntime.CrossFrameContactCacheEnabled
                : contactSolverSettings.EnableFatAabbCache;
        // 持久邻居拓扑只能为跨子步接触集提供候选；不允许“跨帧开、跨子步关”。
        bool effectivePersistentContactCache =
            requestedPersistentContactCache && effectiveTimestepContactSetCache;
        if (SimulationDebuggerRuntime.TryConsumeContactCacheReset())
            ResetPersistentContactCaches();
        Entity diagnosticSelectedEntity = SimulationDebuggerRuntime.SelectedEntity;
        if (SystemAPI.TryGetSingleton(out Stage3ContactDiagnosticSelection diagnosticSelection) &&
            diagnosticSelection.SelectedEntity != Entity.Null)
        {
            diagnosticSelectedEntity = diagnosticSelection.SelectedEntity;
            SimulationDebuggerRuntime.SelectedEntity = diagnosticSelectedEntity;
        }
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
        var initializeSolverStateJob = new InitializeFlowMovementSolverStateJob
        {
            States = states
        };
        JobHandle initializeSolverStateHandle = initializeSolverStateJob.Schedule(
            unitCount,
            64,
            independentForceHandle);

        // 阶段 2：A0 全量 Sweep 或 A1 跨帧拓扑先生产统一 InteractionSet；
        // B1 在 timestep 内复用，B0 则每个 substep 重建。Soft 与 XPBD 只消费其派生视图。
        var sweptCellEntries = new NativeList<SweptDiscCellEntry>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
        var collisionPairs = new NativeList<UnitCollisionPair>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
        var timestepContactPairs = new NativeList<UnitCollisionPair>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
        var currentBodyIndexByEntity = new NativeParallelHashMap<Entity, int>(
            math.max(unitCount, 1),
            Allocator.TempJob);
        var timestepInteractionPairs = new NativeList<UnitCollisionPair>(
            math.max(unitCount * 8, 1),
            Allocator.TempJob);
        var softAvoidancePairs = new NativeList<UnitCollisionPair>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
        var previousTimestepContactPairs = new NativeList<UnitCollisionPair>(
            math.max(unitCount * 8, 1),
            Allocator.TempJob);
        var currentIncrementalProxies = new NativeList<PersistentSweptProxy>(
            math.max(unitCount, 1),
            Allocator.TempJob);
        var incrementalDirtyBodies = new NativeList<IncrementalDirtyBody>(
            math.max(unitCount, 1),
            Allocator.TempJob);
        var incrementalDirtyFlagsByBody = new NativeArray<byte>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.ClearMemory);
        var predictiveContactScratch = new NativeList<PersistentPredictiveContact>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
        var incrementalNeighborPairScratch = new NativeList<PersistentNeighborPair>(
            math.max(unitCount * 8, 1),
            Allocator.TempJob);
        var incrementalOracleContactPairs = new NativeList<UnitCollisionPair>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
        var predictiveContactSchedule = new NativeList<PredictiveContactScheduleEntry>(
            math.max(unitCount * 2, 1),
            Allocator.TempJob);
        var predictiveContactScheduleScratch =
            new NativeList<PredictiveContactScheduleEntry>(
                math.max(unitCount, 1),
                Allocator.TempJob);
        var predictiveContactScheduleCursor =
            new NativeReference<int>(Allocator.TempJob);
        var correctedBodyFlags = new NativeArray<byte>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.ClearMemory);
        var correctedBodyIndices = new NativeList<int>(
            math.max(unitCount, 1),
            Allocator.TempJob);
        var activeIncidentOffsets = new NativeArray<int>(
            unitCount + 1,
            Allocator.TempJob,
            NativeArrayOptions.ClearMemory);
        var activeIncidentWriteCursors = new NativeArray<int>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.ClearMemory);
        var activeIncidentPairIndices = new NativeList<int>(
            math.max(unitCount * 8, 1),
            Allocator.TempJob);
        var jacobiPairCorrections = new NativeList<JacobiPairCorrection>(
            math.max(unitCount * 4, 1),
            Allocator.TempJob);
        var contactStatistics =
            new NativeReference<PredictiveDiscContactStatistics>(Allocator.TempJob);
        var incrementalStatistics =
            new NativeReference<IncrementalContactPipelineStatistics>(Allocator.TempJob);
        var iterationDiagnostics = new NativeList<Stage3ContactIterationDiagnostic>(
            math.max(contactSolverSettings.SubstepCount * contactSolverSettings.IterationCount, 1),
            Allocator.TempJob);
        var pairDiagnostics = new NativeList<Stage3ContactPairDiagnostic>(
            math.max(unitCount * 2, 1),
            Allocator.TempJob);
        var selectedBodyDiagnostic =
            new NativeReference<Stage3SelectedBodyDiagnostic>(Allocator.TempJob);
        var heatSamples = new NativeArray<Stage3ContactHeatSample>(
            unitCount,
            Allocator.TempJob,
            NativeArrayOptions.ClearMemory);
        var solveContactJob = new SolveXpbdUnitContactsJob
        {
  Configuration = ContactPipelineConfiguration.Create(
        SystemAPI.Time.DeltaTime,
        flowFieldSettings,
        contactSolverSettings,
        effectivePersistentContactCache,
        effectiveTimestepContactSetCache),
            DiagnosticSelectedEntity = diagnosticSelectedEntity,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            Grid = gridComponent.Grid,
            SweptCellEntries = sweptCellEntries,
            Pairs = collisionPairs,
            TimestepContactPairs = timestepContactPairs,
            CurrentBodyIndexByEntity = currentBodyIndexByEntity,
            PreviousTimestepContactPairs = previousTimestepContactPairs,
            TimestepInteractionPairs = timestepInteractionPairs,
            SoftAvoidancePairs = softAvoidancePairs,
            CorrectedBodyFlags = correctedBodyFlags,
            CorrectedBodyIndices = correctedBodyIndices,
            ActiveIncidentOffsets = activeIncidentOffsets,
            ActiveIncidentWriteCursors = activeIncidentWriteCursors,
            ActiveIncidentPairIndices = activeIncidentPairIndices,
            JacobiPairCorrections = jacobiPairCorrections,
            CurrentIncrementalProxies = currentIncrementalProxies,
            PersistentSweptProxies = _persistentSweptProxies,
            PersistentNeighborPairs = _persistentNeighborPairs,
            PersistentPredictiveContacts = _persistentPredictiveContacts,
            PersistentActiveContactKeys = _persistentActiveContactKeys,
            PersistentSoftAvoidancePairKeys = _persistentSoftAvoidancePairKeys,
            PersistentDormantContactSchedule = _persistentDormantContactSchedule,
            PredictiveContactScratch = predictiveContactScratch,
            IncrementalDirtyBodies = incrementalDirtyBodies,
            IncrementalDirtyFlagsByBody = incrementalDirtyFlagsByBody,
            IncrementalNeighborPairScratch = incrementalNeighborPairScratch,
            IncrementalOracleContactPairs = incrementalOracleContactPairs,
            PredictiveContactSchedule = predictiveContactSchedule,
            PredictiveContactScheduleScratch = predictiveContactScheduleScratch,
            PredictiveContactScheduleCursor = predictiveContactScheduleCursor,
            IncrementalCacheState = _incrementalContactCacheState,
            IncrementalStatistics = incrementalStatistics,
            SimulationDebuggerCaptureMask = SimulationDebuggerRuntime.CaptureMask,
            SimulationDebuggerMaximumPairs = SimulationDebuggerRuntime.MaximumVisualizedPairs,
            SimulationDebuggerSelectedPairs = _simulationDebuggerSelectedPairs,
            SimulationDebuggerSelectedUnit = _simulationDebuggerSelectedUnit,
            SimulationDebuggerSelectedUnitValid = _simulationDebuggerSelectedUnitValid,
            States = states,
            Statistics = contactStatistics,
            IterationDiagnostics = iterationDiagnostics,
            PairDiagnostics = pairDiagnostics,
            SelectedBodyDiagnostic = selectedBodyDiagnostic,
            HeatSamples = heatSamples
        };
        JobHandle solveContactHandle = solveContactJob.Schedule(initializeSolverStateHandle);

        var publishStatisticsJob = new PublishPredictiveDiscContactStatisticsJob
        {
            Source = contactStatistics,
            SelectedBodySource = selectedBodyDiagnostic,
            IterationSource = iterationDiagnostics,
            PairSource = pairDiagnostics,
            HeatSource = heatSamples
        };
        JobHandle publishStatisticsHandle =
            publishStatisticsJob.Schedule(solveContactHandle);
        var publishIncrementalStatisticsJob =
            new PublishIncrementalContactPipelineStatisticsJob
            {
                Configuration = IncrementalContactPipelineExperimentRuntime.CaptureConfiguration(
                    unitCount,
                    SystemAPI.Time.DeltaTime,
                    flowFieldSettings.SoftAvoidanceShell,
                    contactSolverSettings,
                    effectiveTimestepContactSetCache,
                    effectivePersistentContactCache),
                SolverSource = contactStatistics,
                Source = incrementalStatistics,
                Target = _incrementalDiagnosticsEntity,
                SnapshotLookup =
                    GetComponentLookup<IncrementalContactPipelineSnapshot>(false)
            };
        JobHandle publishIncrementalStatisticsHandle =
            publishIncrementalStatisticsJob.Schedule(solveContactHandle);

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
        JobHandle currentBodyIndexDisposeHandle =
            currentBodyIndexByEntity.Dispose(applyMovementHandle);
        JobHandle previousTimestepContactPairDisposeHandle =
            previousTimestepContactPairs.Dispose(applyMovementHandle);
        JobHandle timestepInteractionPairDisposeHandle =
            timestepInteractionPairs.Dispose(applyMovementHandle);
        JobHandle softAvoidancePairDisposeHandle =
            softAvoidancePairs.Dispose(applyMovementHandle);
        JobHandle currentIncrementalProxyDisposeHandle =
            currentIncrementalProxies.Dispose(applyMovementHandle);
        JobHandle incrementalDirtyBodyDisposeHandle =
            incrementalDirtyBodies.Dispose(applyMovementHandle);
        JobHandle incrementalDirtyFlagDisposeHandle =
            incrementalDirtyFlagsByBody.Dispose(applyMovementHandle);
        JobHandle predictiveContactScratchDisposeHandle =
            predictiveContactScratch.Dispose(applyMovementHandle);
        JobHandle incrementalNeighborPairScratchDisposeHandle =
            incrementalNeighborPairScratch.Dispose(applyMovementHandle);
        JobHandle incrementalOracleContactPairDisposeHandle =
            incrementalOracleContactPairs.Dispose(applyMovementHandle);
        JobHandle predictiveContactScheduleDisposeHandle =
            predictiveContactSchedule.Dispose(applyMovementHandle);
        JobHandle predictiveContactScheduleScratchDisposeHandle =
            predictiveContactScheduleScratch.Dispose(applyMovementHandle);
        JobHandle predictiveContactScheduleCursorDisposeHandle =
            predictiveContactScheduleCursor.Dispose(applyMovementHandle);
        JobHandle correctedBodyFlagDisposeHandle =
            correctedBodyFlags.Dispose(applyMovementHandle);
        JobHandle correctedBodyIndexDisposeHandle =
            correctedBodyIndices.Dispose(applyMovementHandle);
        JobHandle activeIncidentOffsetDisposeHandle =
            activeIncidentOffsets.Dispose(applyMovementHandle);
        JobHandle activeIncidentWriteCursorDisposeHandle =
            activeIncidentWriteCursors.Dispose(applyMovementHandle);
        JobHandle activeIncidentPairIndexDisposeHandle =
            activeIncidentPairIndices.Dispose(applyMovementHandle);
        JobHandle jacobiPairCorrectionDisposeHandle =
            jacobiPairCorrections.Dispose(applyMovementHandle);
        JobHandle allStatisticsPublishedHandle = JobHandle.CombineDependencies(
            publishStatisticsHandle,
            publishIncrementalStatisticsHandle);
        JobHandle statisticsDisposeHandle =
            contactStatistics.Dispose(allStatisticsPublishedHandle);
        JobHandle incrementalStatisticsDisposeHandle =
            incrementalStatistics.Dispose(publishIncrementalStatisticsHandle);
        JobHandle selectedDiagnosticDisposeHandle =
            selectedBodyDiagnostic.Dispose(publishStatisticsHandle);
        JobHandle iterationDiagnosticDisposeHandle =
            iterationDiagnostics.Dispose(publishStatisticsHandle);
        JobHandle pairDiagnosticDisposeHandle = pairDiagnostics.Dispose(publishStatisticsHandle);
        JobHandle heatSampleDisposeHandle = heatSamples.Dispose(publishStatisticsHandle);

        JobHandle solverScratchDisposeHandle = JobHandle.CombineDependencies(
  stateDisposeHandle,
  footprintDisposeHandle,
  sweptEntryDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  collisionPairDisposeHandle,
  timestepContactPairDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  currentBodyIndexDisposeHandle,
  previousTimestepContactPairDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  timestepInteractionPairDisposeHandle,
  softAvoidancePairDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  currentIncrementalProxyDisposeHandle,
  incrementalDirtyBodyDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  incrementalDirtyFlagDisposeHandle,
  predictiveContactScratchDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  incrementalNeighborPairScratchDisposeHandle,
  incrementalOracleContactPairDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  predictiveContactScheduleDisposeHandle,
  predictiveContactScheduleScratchDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  predictiveContactScheduleCursorDisposeHandle,
  correctedBodyFlagDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  correctedBodyIndexDisposeHandle,
  activeIncidentOffsetDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  activeIncidentWriteCursorDisposeHandle,
  activeIncidentPairIndexDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  jacobiPairCorrectionDisposeHandle,
  incrementalStatisticsDisposeHandle);

        JobHandle diagnosticDisposeHandle = JobHandle.CombineDependencies(
            statisticsDisposeHandle,
            selectedDiagnosticDisposeHandle);
        diagnosticDisposeHandle = JobHandle.CombineDependencies(
  diagnosticDisposeHandle,
  iterationDiagnosticDisposeHandle,
  pairDiagnosticDisposeHandle);
        diagnosticDisposeHandle = JobHandle.CombineDependencies(
  diagnosticDisposeHandle,
  heatSampleDisposeHandle);

        Dependency = JobHandle.CombineDependencies(
  solverScratchDisposeHandle,
  diagnosticDisposeHandle);
    }

    private void ResetPersistentContactCaches()
    {
        // 复位只发生在 benchmark 采样前。先完成上一帧依赖，避免清空仍被 Job 访问的容器。
        Dependency.Complete();
        _persistentSweptProxies.Clear();
        _persistentNeighborPairs.Clear();
        _persistentPredictiveContacts.Clear();
        _persistentActiveContactKeys.Clear();
        _persistentSoftAvoidancePairKeys.Clear();
        _persistentDormantContactSchedule.Clear();
        _incrementalContactCacheState.Value = default;
    }
}
}
