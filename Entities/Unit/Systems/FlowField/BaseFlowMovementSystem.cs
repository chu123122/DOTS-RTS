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
    private NativeList<int> _persistentProxyIndexByBody;
    private NativeList<PersistentNeighborPair> _persistentNeighborPairs;
    private NativeList<PersistentPredictiveContact> _persistentPredictiveContacts;
    private NativeList<StableEntityPairKey> _persistentActiveContactKeys;
    private NativeList<StableEntityPairKey> _persistentSoftAvoidancePairKeys;
    private NativeList<PredictiveContactScheduleEntry> _persistentDormantContactSchedule;
    private NativeReference<IncrementalContactCacheState> _incrementalContactCacheState;

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
        _persistentProxyIndexByBody = new NativeList<int>(Allocator.Persistent);
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
        CreatePersistentIncidentLookup();
        CreatePersistentDiagnostics();
        ulong diagnosticsWorldId = unchecked((ulong)World.Unmanaged.SequenceNumber);
        SimulationDebuggerRuntime.RegisterWorld(diagnosticsWorldId);
        IncrementalContactPipelineExperimentRuntime.RegisterWorld(diagnosticsWorldId);
    }

    protected override void OnDestroy()
    {
        Dependency.Complete();
        if (_persistentSweptProxies.IsCreated)
            _persistentSweptProxies.Dispose();
        if (_persistentProxyIndexByBody.IsCreated)
            _persistentProxyIndexByBody.Dispose();
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
        DisposePersistentIncidentLookup();
        DisposePersistentDiagnostics();
        ulong diagnosticsWorldId = unchecked((ulong)World.Unmanaged.SequenceNumber);
        IncrementalContactPipelineExperimentRuntime.UnregisterWorld(diagnosticsWorldId);
        SimulationDebuggerRuntime.UnregisterWorld(diagnosticsWorldId);
    }

    protected override void OnUpdate()
    {
        ulong diagnosticsWorldId = unchecked((ulong)World.Unmanaged.SequenceNumber);
        var gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();
        var flowFieldSettings = SystemAPI.GetSingleton<FlowFieldSettings>();
        var flowFieldRuntimeState = SystemAPI.GetSingleton<FlowFieldRuntimeState>();
        var contactSolverSettings = SystemAPI.GetSingleton<UnitContactSolverSettings>();

        // 先发布上一时间步已经完成的统计，再应用下一时间步的实验配置。
        // 这样面板中的 ExperimentId、有效配置和求解结果始终属于同一帧。
        PublishSimulationDebuggerSnapshot(diagnosticsWorldId);
        ApplySimulationDebuggerRuntimeOverrides(
            ref flowFieldSettings,
            ref contactSolverSettings);
        IncrementalContactPipelineExperimentRuntime.Apply(diagnosticsWorldId, ref contactSolverSettings);
        bool effectiveTimestepContactSetCache =
            contactSolverSettings.EnableTimestepContactSetCache;
        bool requestedPersistentContactCache =
            IncrementalContactPipelineExperimentRuntime.OverrideEnabledFor(diagnosticsWorldId)
                ? IncrementalContactPipelineExperimentRuntime.CrossFrameContactCacheEnabledFor(diagnosticsWorldId)
                : contactSolverSettings.EnableFatAabbCache;
        // 持久邻居拓扑只能为跨子步接触集提供候选；不允许“跨帧开、跨子步关”。
        bool effectivePersistentContactCache =
            requestedPersistentContactCache && effectiveTimestepContactSetCache;
        if (SimulationDebuggerRuntime.TryConsumeContactCacheReset(diagnosticsWorldId))
            ResetPersistentContactCaches();
        SimulationDebuggerCaptureMask diagnosticsCaptureMask =
            SimulationDebuggerRuntime.CaptureMaskFor(diagnosticsWorldId);
        int diagnosticsMaximumPairs =
            SimulationDebuggerRuntime.MaximumVisualizedPairsFor(diagnosticsWorldId);
        Entity diagnosticSelectedEntity = ResolveDiagnosticSelectedEntity(diagnosticsWorldId);
        if (!gridComponent.Grid.IsCreated) return;
        if (flowFieldRuntimeState.ActiveVersion == 0) return;

        int unitCount = _movementQuery.CalculateEntityCount();
        if (unitCount == 0) return;
        EnsurePersistentIncidentLookupCapacity(unitCount);
        if (_persistentProxyIndexByBody.Capacity < unitCount)
            _persistentProxyIndexByBody.Capacity = unitCount;
        bool usesJacobiScratch =
            contactSolverSettings.ContactPositionSolver ==
            ContactPositionSolverMode.Jacobi;
        bool useParallelJacobi = usesJacobiScratch;
        bool captureParallelSelectedPairs = ShouldCaptureParallelSelectedPairs(
            useParallelJacobi,
            diagnosticSelectedEntity,
            diagnosticsCaptureMask);
        SimulationDebuggerEffectiveSettings completedEffectiveSettings =
            BuildEffectiveSettings(
                flowFieldSettings,
                contactSolverSettings,
                AdaptiveFatAabbSettings.Default);
        CompletedSimulationStepMetadata completedStep = new CompletedSimulationStepMetadata
        {
            WorldId = diagnosticsWorldId,
            ElapsedTime = SystemAPI.Time.ElapsedTime,
            DeltaTime = SystemAPI.Time.DeltaTime,
            UnitCount = unitCount,
            MaximumVisualizedPairs = diagnosticsMaximumPairs,
            SelectedEntity = diagnosticSelectedEntity,
            CaptureMask = diagnosticsCaptureMask,
            EffectiveSettings = completedEffectiveSettings,
            Experiment = SimulationDebuggerRuntime.UpdateExperimentIdentity(
                diagnosticsWorldId,
                completedEffectiveSettings)
        };
        ContactDiagnosticsFrameResources diagnosticsScratch =
            CreateContactDiagnosticsFrameResources(unitCount, contactSolverSettings, captureParallelSelectedPairs);

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
        NativeList<UnitCollisionPair> incrementalOracleContactPairs =
            diagnosticsScratch.IncrementalOracleContactPairs;
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
        var activeIncidentOffsets = usesJacobiScratch
            ? new NativeArray<int>(
                unitCount + 1,
                Allocator.TempJob,
                NativeArrayOptions.ClearMemory)
            : default;
        var activeIncidentWriteCursors = usesJacobiScratch
            ? new NativeArray<int>(
                unitCount,
                Allocator.TempJob,
                NativeArrayOptions.ClearMemory)
            : default;
        var activeIncidentPairIndices = usesJacobiScratch
            ? new NativeList<int>(math.max(unitCount * 8, 1), Allocator.TempJob)
            : default;
        var jacobiPairCorrections = usesJacobiScratch
            ? new NativeList<JacobiPairCorrection>(
                math.max(unitCount * 4, 1),
                Allocator.TempJob)
            : default;
        var envelopeEscapeFlags = useParallelJacobi
            ? new NativeArray<byte>(
                unitCount,
                Allocator.TempJob,
                NativeArrayOptions.ClearMemory)
            : default;
        var parallelBodyStatistics = useParallelJacobi
            ? new NativeArray<ParallelBodyStageResult>(
                unitCount,
                Allocator.TempJob,
                NativeArrayOptions.ClearMemory)
            : default;
        var softIncidentOffsets = useParallelJacobi
            ? new NativeArray<int>(
                unitCount + 1,
                Allocator.TempJob,
                NativeArrayOptions.ClearMemory)
            : default;
        var softIncidentWriteCursors = useParallelJacobi
            ? new NativeArray<int>(
                unitCount,
                Allocator.TempJob,
                NativeArrayOptions.ClearMemory)
            : default;
        var softIncidentPairIndices = useParallelJacobi
            ? new NativeList<int>(math.max(unitCount * 8, 1), Allocator.TempJob)
            : default;
        var softPairContributions = useParallelJacobi
            ? new NativeList<SoftAvoidancePairContribution>(
                math.max(unitCount * 4, 1),
                Allocator.TempJob)
            : default;
        var activeIncidentIndexState = usesJacobiScratch
            ? new NativeReference<ActiveIncidentIndexState>(Allocator.TempJob)
            : default;
        var persistentClassificationResults = useParallelJacobi
            ? new NativeList<PersistentPairClassificationResult>(
                math.max(unitCount * 8, 1),
                Allocator.TempJob)
            : default;
        var persistentClassificationState = useParallelJacobi
            ? new NativeReference<ParallelPersistentClassificationState>(
                Allocator.TempJob)
            : default;
        NativeList<ParallelSimulationDebuggerPairCapture> parallelSimulationDebuggerPairCandidates =
            diagnosticsScratch.ParallelPairCandidates;
        NativeList<SimulationDebuggerPairSample> parallelSimulationDebuggerPairScratch =
            diagnosticsScratch.ParallelPairScratch;
        var persistentSpatialVisitStampByProxy = new NativeArray<uint>(
            unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory);
        var persistentSpatialVisitStamp =
            new NativeReference<uint>(Allocator.TempJob);
        var parallelJacobiRuntimeState = useParallelJacobi
            ? new NativeReference<ParallelJacobiExecutionState>(Allocator.TempJob)
            : default;
#if RTS_CONTACT_DIAGNOSTICS
        var parallelJacobiIterationState = useParallelJacobi
            ? new NativeReference<ParallelJacobiIterationTelemetry>(Allocator.TempJob)
            : default;
        var parallelJacobiBlockTelemetry = useParallelJacobi
            ? new NativeList<JacobiBlockTelemetry>(
                math.max((unitCount * 4 + 63) / 64, 1),
                Allocator.TempJob)
            : default;
#endif
        NativeReference<PredictiveDiscContactStatistics> contactStatistics =
            diagnosticsScratch.ContactStatistics;
        NativeReference<IncrementalContactPipelineStatistics> incrementalStatistics =
            diagnosticsScratch.IncrementalStatistics;
        NativeList<Stage3ContactIterationDiagnostic> iterationDiagnostics = diagnosticsScratch.Iterations;
        NativeList<Stage3ContactPairDiagnostic> pairDiagnostics = diagnosticsScratch.Pairs;
        NativeReference<Stage3SelectedBodyDiagnostic> selectedBodyDiagnostic = diagnosticsScratch.SelectedBody;
        NativeArray<Stage3ContactHeatSample> heatSamples = diagnosticsScratch.HeatSamples;
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
            EnvelopeEscapeFlags = envelopeEscapeFlags,
            ParallelBodyStatistics = parallelBodyStatistics,
            SoftIncidentOffsets = softIncidentOffsets,
            SoftIncidentWriteCursors = softIncidentWriteCursors,
            SoftIncidentPairIndices = softIncidentPairIndices,
            SoftPairContributions = softPairContributions,
            ActiveIncidentIndexState = activeIncidentIndexState,
            CurrentIncrementalProxies = currentIncrementalProxies,
            PersistentSweptProxies = _persistentSweptProxies,
            PersistentProxyIndexByBody = _persistentProxyIndexByBody,
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
            PersistentIncidentPairLookup = _persistentIncidentPairLookup,
            PersistentIncidentLookupEpoch = _persistentIncidentLookupEpoch,
            PersistentSpatialMembership = _persistentSpatialMembership,
            PersistentSpatialMembershipEpoch = _persistentSpatialMembershipEpoch,
            PersistentSpatialVisitStampByProxy = persistentSpatialVisitStampByProxy,
            PersistentSpatialVisitStamp = persistentSpatialVisitStamp,
            PersistentClassificationResults = persistentClassificationResults,
            PersistentClassificationState = persistentClassificationState,
            SimulationDebuggerCaptureMask = diagnosticsCaptureMask,
            SimulationDebuggerMaximumPairs = diagnosticsMaximumPairs,
            SimulationDebuggerSelectedPairs = _simulationDebuggerSelectedPairs,
            ParallelSimulationDebuggerPairCandidates =
                parallelSimulationDebuggerPairCandidates,
            ParallelSimulationDebuggerPairScratch =
                parallelSimulationDebuggerPairScratch,
            SimulationDebuggerSelectedUnit = _simulationDebuggerSelectedUnit,
            SimulationDebuggerSelectedUnitValid = _simulationDebuggerSelectedUnitValid,
            States = states,
            Statistics = contactStatistics,
            IterationDiagnostics = iterationDiagnostics,
            PairDiagnostics = pairDiagnostics,
            SelectedBodyDiagnostic = selectedBodyDiagnostic,
            HeatSamples = heatSamples
        };
        JobHandle solveContactHandle;
        if (useParallelJacobi)
        {
#if RTS_CONTACT_DIAGNOSTICS
            solveContactHandle = solveContactJob.ScheduleParallelJacobiP1P6(
                parallelJacobiRuntimeState,
                parallelJacobiIterationState,
                parallelJacobiBlockTelemetry,
                initializeSolverStateHandle);
#else
            solveContactHandle = solveContactJob.ScheduleParallelJacobiP1P6(
                parallelJacobiRuntimeState,
                initializeSolverStateHandle);
#endif
        }
        else
        {
            solveContactHandle = solveContactJob.Schedule(initializeSolverStateHandle);
        }

        ContactDiagnosticsPublishHandles diagnosticsPublish = ScheduleContactDiagnosticsPublication(
            diagnosticsScratch,
            contactStatistics,
            incrementalStatistics,
            completedStep,
            unitCount,
            SystemAPI.Time.DeltaTime,
            flowFieldSettings.SoftAvoidanceShell,
            contactSolverSettings,
            effectiveTimestepContactSetCache,
            effectivePersistentContactCache,
            solveContactHandle);
        JobHandle publishStatisticsHandle = diagnosticsPublish.Statistics;
        JobHandle publishIncrementalStatisticsHandle = diagnosticsPublish.Incremental;

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
        JobHandle incrementalOracleContactPairDisposeHandle = default;
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
        JobHandle activeIncidentOffsetDisposeHandle = activeIncidentOffsets.IsCreated
            ? activeIncidentOffsets.Dispose(applyMovementHandle)
            : default;
        JobHandle activeIncidentWriteCursorDisposeHandle =
            activeIncidentWriteCursors.IsCreated
                ? activeIncidentWriteCursors.Dispose(applyMovementHandle)
                : default;
        JobHandle activeIncidentPairIndexDisposeHandle =
            activeIncidentPairIndices.IsCreated
                ? activeIncidentPairIndices.Dispose(applyMovementHandle)
                : default;
        JobHandle jacobiPairCorrectionDisposeHandle = jacobiPairCorrections.IsCreated
            ? jacobiPairCorrections.Dispose(applyMovementHandle)
            : default;
        JobHandle envelopeEscapeFlagDisposeHandle = envelopeEscapeFlags.IsCreated
            ? envelopeEscapeFlags.Dispose(applyMovementHandle)
            : default;
        JobHandle parallelBodyStatisticsDisposeHandle =
            parallelBodyStatistics.IsCreated
                ? parallelBodyStatistics.Dispose(applyMovementHandle)
                : default;
        JobHandle softIncidentOffsetDisposeHandle = softIncidentOffsets.IsCreated
            ? softIncidentOffsets.Dispose(applyMovementHandle)
            : default;
        JobHandle softIncidentWriteCursorDisposeHandle =
            softIncidentWriteCursors.IsCreated
                ? softIncidentWriteCursors.Dispose(applyMovementHandle)
                : default;
        JobHandle softIncidentPairIndexDisposeHandle =
            softIncidentPairIndices.IsCreated
                ? softIncidentPairIndices.Dispose(applyMovementHandle)
                : default;
        JobHandle softPairContributionDisposeHandle = softPairContributions.IsCreated
            ? softPairContributions.Dispose(applyMovementHandle)
            : default;
        JobHandle activeIncidentIndexStateDisposeHandle =
            activeIncidentIndexState.IsCreated
                ? activeIncidentIndexState.Dispose(applyMovementHandle)
                : default;
        JobHandle persistentClassificationResultDisposeHandle =
            persistentClassificationResults.IsCreated
                ? persistentClassificationResults.Dispose(applyMovementHandle)
                : default;
        JobHandle persistentClassificationStateDisposeHandle =
            persistentClassificationState.IsCreated
                ? persistentClassificationState.Dispose(applyMovementHandle)
                : default;
        JobHandle persistentSpatialVisitStampArrayDisposeHandle =
            persistentSpatialVisitStampByProxy.Dispose(applyMovementHandle);
        JobHandle persistentSpatialVisitStampDisposeHandle =
            persistentSpatialVisitStamp.Dispose(applyMovementHandle);
        JobHandle parallelJacobiRuntimeStateDisposeHandle =
            parallelJacobiRuntimeState.IsCreated
                ? parallelJacobiRuntimeState.Dispose(applyMovementHandle)
                : default;
#if RTS_CONTACT_DIAGNOSTICS
        JobHandle parallelJacobiIterationStateDisposeHandle =
            parallelJacobiIterationState.IsCreated
                ? parallelJacobiIterationState.Dispose(applyMovementHandle)
                : default;
        JobHandle parallelJacobiBlockTelemetryDisposeHandle =
            parallelJacobiBlockTelemetry.IsCreated
                ? parallelJacobiBlockTelemetry.Dispose(applyMovementHandle)
                : default;
#endif
        JobHandle diagnosticsScratchDisposeHandle = DisposeContactDiagnosticsFrameResources(
            diagnosticsScratch, solveContactHandle, publishStatisticsHandle,
            publishIncrementalStatisticsHandle);

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
            jacobiPairCorrectionDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            envelopeEscapeFlagDisposeHandle,
            parallelBodyStatisticsDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            softIncidentOffsetDisposeHandle,
            softIncidentWriteCursorDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            softIncidentPairIndexDisposeHandle,
            softPairContributionDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            activeIncidentIndexStateDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            persistentClassificationResultDisposeHandle,
            persistentClassificationStateDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            persistentSpatialVisitStampArrayDisposeHandle,
            persistentSpatialVisitStampDisposeHandle);
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            parallelJacobiRuntimeStateDisposeHandle);
#if RTS_CONTACT_DIAGNOSTICS
        solverScratchDisposeHandle = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            parallelJacobiIterationStateDisposeHandle,
            parallelJacobiBlockTelemetryDisposeHandle);
#endif

        Dependency = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            diagnosticsScratchDisposeHandle);
    }

    private void ResetPersistentContactCaches()
    {
        // 复位只发生在 benchmark 采样前。先完成上一帧依赖，避免清空仍被 Job 访问的容器。
        Dependency.Complete();
        _persistentSweptProxies.Clear();
        _persistentProxyIndexByBody.Clear();
        _persistentNeighborPairs.Clear();
        _persistentPredictiveContacts.Clear();
        _persistentActiveContactKeys.Clear();
        _persistentSoftAvoidancePairKeys.Clear();
        _persistentDormantContactSchedule.Clear();
        _incrementalContactCacheState.Value = default;
        if (_persistentIncidentPairLookup.IsCreated)
            _persistentIncidentPairLookup.Clear();
        if (_persistentIncidentLookupEpoch.IsCreated)
            _persistentIncidentLookupEpoch.Value = 0;
        if (_persistentSpatialMembership.IsCreated)
            _persistentSpatialMembership.Clear();
        if (_persistentSpatialMembershipEpoch.IsCreated)
            _persistentSpatialMembershipEpoch.Value = 0;
    }
}
}
