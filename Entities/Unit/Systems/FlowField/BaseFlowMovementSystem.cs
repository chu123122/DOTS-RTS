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
    private ContactPersistentState _persistentState;

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
        _persistentState = ContactPersistentState.Create();
        CreatePersistentDiagnostics();
        ulong diagnosticsWorldId = unchecked((ulong)World.Unmanaged.SequenceNumber);
        SimulationDebuggerRuntime.RegisterWorld(diagnosticsWorldId);
        IncrementalContactPipelineExperimentRuntime.RegisterWorld(diagnosticsWorldId);
    }

    protected override void OnDestroy()
    {
        Dependency.Complete();
        _persistentState.Dispose();
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
        if (_persistentState.RequiresCapacity(unitCount))
        {
            Dependency.Complete();
            _persistentState.EnsureCapacity(unitCount);
        }
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

        ContactFrameResources frameResources = ContactFrameResources.Create(
            unitCount,
            usesJacobiScratch,
            useParallelJacobi);
        NativeArray<FlowMovementFrameState> states = frameResources.States;
        NativeArray<float2> collisionFootprints = frameResources.CollisionFootprints;

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

        // 阶段 2：Runtime scratch 由 ContactFrameResources 统一拥有。
        NativeList<SweptDiscCellEntry> sweptCellEntries = frameResources.SweptCellEntries;
        NativeList<UnitCollisionPair> collisionPairs = frameResources.CollisionPairs;
        NativeList<UnitCollisionPair> timestepContactPairs = frameResources.TimestepContactPairs;
        NativeParallelHashMap<Entity, int> currentBodyIndexByEntity = frameResources.CurrentBodyIndexByEntity;
        NativeList<UnitCollisionPair> timestepInteractionPairs = frameResources.TimestepInteractionPairs;
        NativeList<UnitCollisionPair> softAvoidancePairs = frameResources.SoftAvoidancePairs;
        NativeList<UnitCollisionPair> previousTimestepContactPairs = frameResources.PreviousTimestepContactPairs;
        NativeList<PersistentSweptProxy> currentIncrementalProxies = frameResources.CurrentIncrementalProxies;
        NativeList<IncrementalDirtyBody> incrementalDirtyBodies = frameResources.IncrementalDirtyBodies;
        NativeArray<byte> incrementalDirtyFlagsByBody = frameResources.IncrementalDirtyFlagsByBody;
        NativeList<PersistentPredictiveContact> predictiveContactScratch = frameResources.PredictiveContactScratch;
        NativeList<PersistentNeighborPair> incrementalNeighborPairScratch = frameResources.IncrementalNeighborPairScratch;
        NativeList<UnitCollisionPair> incrementalOracleContactPairs = diagnosticsScratch.IncrementalOracleContactPairs;
        NativeList<PredictiveContactScheduleEntry> predictiveContactSchedule = frameResources.PredictiveContactSchedule;
        NativeList<PredictiveContactScheduleEntry> predictiveContactScheduleScratch = frameResources.PredictiveContactScheduleScratch;
        NativeReference<int> predictiveContactScheduleCursor = frameResources.PredictiveContactScheduleCursor;
        NativeArray<byte> correctedBodyFlags = frameResources.CorrectedBodyFlags;
        NativeList<int> correctedBodyIndices = frameResources.CorrectedBodyIndices;
        NativeArray<int> activeIncidentOffsets = frameResources.ActiveIncidentOffsets;
        NativeArray<int> activeIncidentWriteCursors = frameResources.ActiveIncidentWriteCursors;
        NativeList<int> activeIncidentPairIndices = frameResources.ActiveIncidentPairIndices;
        NativeList<JacobiPairCorrection> jacobiPairCorrections = frameResources.JacobiPairCorrections;
        NativeArray<byte> envelopeEscapeFlags = frameResources.EnvelopeEscapeFlags;
        NativeArray<ParallelBodyStageResult> parallelBodyStatistics = frameResources.ParallelBodyResults;
        NativeArray<int> dirtyBodyBlockOffsets = frameResources.DirtyBodyBlockOffsets;
        NativeArray<int> softIncidentOffsets = frameResources.SoftIncidentOffsets;
        NativeArray<int> softIncidentWriteCursors = frameResources.SoftIncidentWriteCursors;
        NativeList<int> softIncidentPairIndices = frameResources.SoftIncidentPairIndices;
        NativeList<SoftAvoidancePairContribution> softPairContributions = frameResources.SoftPairContributions;
        NativeReference<ActiveIncidentIndexState> activeIncidentIndexState = frameResources.ActiveIncidentIndexState;
        NativeList<PersistentPairClassificationResult> persistentClassificationResults = frameResources.PersistentClassificationResults;
        NativeReference<PersistentClassificationPhaseState> persistentClassificationState = frameResources.PersistentClassificationState;
        NativeList<ParallelSimulationDebuggerPairCapture> parallelSimulationDebuggerPairCandidates = diagnosticsScratch.ParallelPairCandidates;
        NativeList<SimulationDebuggerPairSample> parallelSimulationDebuggerPairScratch = diagnosticsScratch.ParallelPairScratch;
        NativeArray<uint> persistentSpatialVisitStampByProxy = frameResources.PersistentSpatialVisitStampByProxy;
        NativeReference<uint> persistentSpatialVisitStamp = frameResources.PersistentSpatialVisitStamp;
        NativeReference<ParallelJacobiExecutionState> parallelJacobiRuntimeState = frameResources.ParallelJacobiRuntimeState;
#if RTS_CONTACT_DIAGNOSTICS
        NativeReference<PersistentClassificationTelemetryState> persistentClassificationTelemetry = frameResources.PersistentClassificationTelemetry;
        NativeReference<ParallelJacobiIterationTelemetry> parallelJacobiIterationState = frameResources.ParallelJacobiIterationState;
        NativeList<JacobiBlockTelemetry> parallelJacobiBlockTelemetry = frameResources.ParallelJacobiBlockTelemetry;
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
            DirtyBodyBlockOffsets = dirtyBodyBlockOffsets,
            SoftIncidentOffsets = softIncidentOffsets,
            SoftIncidentWriteCursors = softIncidentWriteCursors,
            SoftIncidentPairIndices = softIncidentPairIndices,
            SoftPairContributions = softPairContributions,
            ActiveIncidentIndexState = activeIncidentIndexState,
            CurrentIncrementalProxies = currentIncrementalProxies,
            PersistentSweptProxies = _persistentState.SweptProxies,
            PersistentProxyIndexByBody = _persistentState.ProxyIndexByBody,
            PersistentNeighborPairs = _persistentState.NeighborPairs,
            PersistentPredictiveContacts = _persistentState.PredictiveContacts,
            PersistentActiveContactKeys = _persistentState.ActiveContactKeys,
            PersistentSoftAvoidancePairKeys = _persistentState.SoftAvoidancePairKeys,
            PersistentDormantContactSchedule = _persistentState.DormantContactSchedule,
            PredictiveContactScratch = predictiveContactScratch,
            IncrementalDirtyBodies = incrementalDirtyBodies,
            IncrementalDirtyFlagsByBody = incrementalDirtyFlagsByBody,
            IncrementalNeighborPairScratch = incrementalNeighborPairScratch,
            IncrementalOracleContactPairs = incrementalOracleContactPairs,
            PredictiveContactSchedule = predictiveContactSchedule,
            PredictiveContactScheduleScratch = predictiveContactScheduleScratch,
            PredictiveContactScheduleCursor = predictiveContactScheduleCursor,
            IncrementalCacheState = _persistentState.CacheState,
            IncrementalStatistics = incrementalStatistics,
            PersistentIncidentPairLookup = _persistentState.IncidentPairLookup,
            PersistentIncidentLookupEpoch = _persistentState.IncidentLookupEpoch,
            PersistentSpatialMembership = _persistentState.SpatialMembership,
            PersistentSpatialMembershipEpoch = _persistentState.SpatialMembershipEpoch,
            PersistentSpatialVisitStampByProxy = persistentSpatialVisitStampByProxy,
            PersistentSpatialVisitStamp = persistentSpatialVisitStamp,
            PersistentClassificationResults = persistentClassificationResults,
            PersistentClassificationState = persistentClassificationState,
#if RTS_CONTACT_DIAGNOSTICS
            PersistentClassificationTelemetry = persistentClassificationTelemetry,
#endif
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

        // Runtime 和 Diagnostics 两类资源分别等待各自最终 reader。
        JobHandle solverScratchDisposeHandle = frameResources.Dispose(applyMovementHandle);
        JobHandle diagnosticsScratchDisposeHandle = DisposeContactDiagnosticsFrameResources(
            diagnosticsScratch,
            solveContactHandle,
            publishStatisticsHandle,
            publishIncrementalStatisticsHandle);

        Dependency = JobHandle.CombineDependencies(
            solverScratchDisposeHandle,
            diagnosticsScratchDisposeHandle);
    }

    private void ResetPersistentContactCaches()
    {
        // 复位只发生在 benchmark 采样前。先完成上一帧依赖，避免清空仍被 Job 访问的容器。
        Dependency.Complete();
        _persistentState.Reset();
    }
}
}
