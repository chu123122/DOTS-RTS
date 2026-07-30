using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling;
using Unity.Transforms;
using UnityEngine;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>
/// Crowd-simulation composition root for one ECS World. It owns stage ordering,
/// resource lifetime and JobHandle dependencies. Each resource owner constructs only
/// its capability-limited stage job; there is no aggregate frame bag or ABI adapter.
/// </summary>
public abstract partial class BaseFlowMovementSystem : SystemBase
{
    // RTS.Simulation.Update：主线程 OnUpdate 调度停留（不含 Worker job 执行）。
    // RTS.Simulation.Total：诊断构建含 Worker job 完整 wall time；
    // Release 只覆盖调度，避免为观测强制同步正常物理管线。
    private static readonly ProfilerMarker SimulationUpdateMarker =
        new ProfilerMarker("RTS.Simulation.Update");
    private static readonly ProfilerMarker SimulationTotalMarker =
        new ProfilerMarker("RTS.Simulation.Total");

    private EntityQuery _movementQuery;
    private CrowdPhysicsRuntime _physicsRuntime;
    private int _crossFrameCapacity;
    private uint _simulationStepId;

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
            ComponentType.ReadWrite<CrowdQueryProxy>(),
            ComponentType.ReadOnly<UnitMoveSpeed>(),
            ComponentType.ReadOnly<UnitMovementSettings>(),
            ComponentType.ReadOnly<UnitContactBody>(),
            ComponentType.ReadOnly<CrowdDiscShape>(),
            ComponentType.ReadOnly<UnitMoveDestination>());
        _physicsRuntime = CrowdPhysicsRuntime.Create();
        CreatePersistentDiagnostics();
        ulong worldId = SimulationDebuggerWorldIdentity.FromSequenceNumber(
            World.Unmanaged.SequenceNumber);
        SimulationDebuggerRuntime.RegisterWorld(worldId);
        IncrementalContactPipelineExperimentRuntime.RegisterWorld(worldId);
    }

    protected override void OnDestroy()
    {
        Dependency.Complete();
        _physicsRuntime.Dispose();
        DisposePersistentDiagnostics();
        ulong worldId = SimulationDebuggerWorldIdentity.FromSequenceNumber(
            World.Unmanaged.SequenceNumber);
        IncrementalContactPipelineExperimentRuntime.UnregisterWorld(worldId);
        SimulationDebuggerRuntime.UnregisterWorld(worldId);
    }

    protected override void OnUpdate()
    {
        SimulationTotalMarker.Begin();
        SimulationUpdateMarker.Begin();
        ulong worldId = SimulationDebuggerWorldIdentity.FromSequenceNumber(
            World.Unmanaged.SequenceNumber);
        FlowFieldGrid gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();
        FlowFieldSettings flowFieldSettings = SystemAPI.GetSingleton<FlowFieldSettings>();
        FlowFieldRuntimeState flowFieldRuntimeState =
            SystemAPI.GetSingleton<FlowFieldRuntimeState>();
        UnitContactSolverSettings contactSolverSettings =
            SystemAPI.GetSingleton<UnitContactSolverSettings>();

        PublishSimulationDebuggerSnapshot(worldId, gridComponent);
        ApplySimulationDebuggerRuntimeOverrides(
            ref contactSolverSettings);
        IncrementalContactPipelineExperimentRuntime.Apply(
            worldId,
            ref contactSolverSettings);

        bool effectiveTimestepContactSetCache =
            contactSolverSettings.EnableTimestepContactSetCache;
        bool requestedPersistentContactCache =
            IncrementalContactPipelineExperimentRuntime.OverrideEnabledFor(worldId)
                ? IncrementalContactPipelineExperimentRuntime
                    .CrossFrameContactCacheEnabledFor(worldId)
                : contactSolverSettings.EnablePersistentContactCache;
        bool effectivePersistentContactCache =
            requestedPersistentContactCache && effectiveTimestepContactSetCache;

        if (SimulationDebuggerRuntime.TryConsumeContactCacheReset(worldId))
            ResetPersistentContactCaches();

        SimulationDebuggerCaptureMask captureMask =
            SimulationDebuggerRuntime.CaptureMaskFor(worldId);
        int maximumVisualizedPairs =
            SimulationDebuggerRuntime.MaximumVisualizedPairsFor(worldId);
        Entity selectedEntity = ResolveDiagnosticSelectedEntity(worldId);

        if (!gridComponent.Grid.IsCreated ||
            flowFieldRuntimeState.ActiveVersion == 0)
        {
            SimulationUpdateMarker.End();
            SimulationTotalMarker.End();
            return;
        }

        FlowFieldBakeSystem environmentPublisher =
            World.GetExistingSystemManaged<FlowFieldBakeSystem>();
        if (environmentPublisher == null ||
            !environmentPublisher.TryGetPublishedObstacleSnapshot(
                out CrowdObstacleSnapshot obstacleSnapshot))
        {
            SimulationUpdateMarker.End();
            SimulationTotalMarker.End();
            return;
        }

        int unitCount = _movementQuery.CalculateEntityCount();
        if (unitCount == 0)
        {
            SimulationUpdateMarker.End();
            SimulationTotalMarker.End();
            return;
        }

        uint simulationStepId = NextSimulationStepId();
        if (unitCount > _crossFrameCapacity)
        {
            // Native container resize 是显式结构变更边界；稳定容量帧不阻塞。
            Dependency.Complete();
            _physicsRuntime.EnsureCapacity(unitCount);
            _crossFrameCapacity = unitCount;
        }

#if RTS_CONTACT_DIAGNOSTICS
        bool usesJacobiSolver =
            contactSolverSettings.ContactPositionSolver ==
            ContactPositionSolverMode.Jacobi;
#endif
        SimulationDebuggerEffectiveSettings effectiveSettings =
            BuildEffectiveSettings(
                contactSolverSettings,
                AdaptiveFatAabbSettings.Default);
        CompletedSimulationStepMetadata completedStep = new CompletedSimulationStepMetadata
        {
            WorldId = worldId,
            SimulationStepId = simulationStepId,
            ElapsedTime = SystemAPI.Time.ElapsedTime,
            DeltaTime = SystemAPI.Time.DeltaTime,
            UnitCount = unitCount,
            MaximumVisualizedPairs = maximumVisualizedPairs,
            SelectedEntity = selectedEntity,
            CaptureMask = captureMask,
            EffectiveSettings = effectiveSettings,
            Experiment = SimulationDebuggerRuntime.UpdateExperimentIdentity(
                worldId,
                effectiveSettings)
        };

        CrowdPhysicsDiagnosticsStep diagnostics = null;
        CrowdPhysicsStep physicsStep = null;
        JobHandle intentHandle = default;
        JobHandle solveHandle = default;
        JobHandle outputReadyHandle = default;
        JobHandle statisticsHandle = default;
        JobHandle incrementalHandle = default;
        JobHandle applyHandle = default;
        bool stepReleased = false;
        bool diagnosticsReleased = false;
        try
        {
            diagnostics = _physicsRuntime.CreateDiagnosticsStep(
                unitCount,
                contactSolverSettings.SubstepCount,
                contactSolverSettings.IterationCount);
            physicsStep = _physicsRuntime.CreateStep(unitCount);

            FlowGridGeometry gridGeometry = new FlowGridGeometry(
                gridComponent.GridOrigin,
                gridComponent.GridDimensions,
                gridComponent.CellRadius);
            intentHandle = new BuildCrowdMotionIntentJob
            {
                NavigationCells = gridComponent.Grid,
                NavigationGrid = gridGeometry,
                ActiveRequestVersion =
                    flowFieldRuntimeState.ActiveRequestVersion,
                StepInputs = physicsStep.InputBodies
            }.ScheduleParallel(_movementQuery, Dependency);

            ContactPipelineConfiguration configuration =
                ContactPipelineConfiguration.Create(
                    worldId,
                    simulationStepId,
                    SystemAPI.Time.DeltaTime,
                    new CrowdPhysicsSettings
                    {
                        ObstacleVersion = obstacleSnapshot.Version,
                        SubstepCount = contactSolverSettings.SubstepCount,
                        IterationCount =
                            contactSolverSettings.IterationCount,
                        ContactPositionSolver =
                            contactSolverSettings.ContactPositionSolver,
                        Compliance = contactSolverSettings.Compliance,
                        PredictiveSkin =
                            contactSolverSettings.PredictiveSkin,
                        SoftAvoidanceResponseRate =
                            contactSolverSettings
                                .SoftAvoidanceResponseRate,
                        SoftAvoidanceShell =
                            contactSolverSettings.SoftAvoidanceShell,
                        SettledSoftAvoidanceMultiplier =
                            contactSolverSettings
                                .SettledSoftAvoidanceMultiplier,
                        SoftAvoidanceVelocitySolver =
                            contactSolverSettings
                                .SoftAvoidanceVelocitySolver,
                        RvoTimeHorizon =
                            contactSolverSettings.RvoTimeHorizon,
                        EnablePredictivePairGeneration =
                            contactSolverSettings
                                .EnablePredictivePairGeneration,
                        EnablePredictiveContacts =
                            contactSolverSettings.EnablePredictiveContacts,
                        EnableDiagnostics =
                            contactSolverSettings.EnableDiagnostics,
                        EnablePersistentContactCache =
                            effectivePersistentContactCache,
                        EnableTimestepContactSetCache =
                            effectiveTimestepContactSetCache,
                        // 兼容性翻译：旧 FatAabb margin 表示受守护 proxy 余量。
                        GuardEnvelopeMargin =
                            contactSolverSettings
                                .PersistentGuardEnvelopeMargin,
                        TimestepContactMargin =
                            contactSolverSettings.TimestepContactMargin
                    });
            CrowdPhysicsScheduleHandles physicsHandles =
                _physicsRuntime.ScheduleStep(
                physicsStep,
                configuration,
                obstacleSnapshot,
                diagnostics,
                selectedEntity,
                captureMask,
                maximumVisualizedPairs,
                SystemAPI.Time.DeltaTime,
                intentHandle);
            solveHandle = physicsHandles.Solve;
            outputReadyHandle = physicsHandles.OutputReady;

            // Register the environment reader before scheduling optional
            // publication/writeback. If any later operation fails, the catch
            // completes solveHandle before the snapshot can be reused.
            environmentPublisher.RegisterPublishedEnvironmentReader(
                solveHandle);

            statisticsHandle =
                diagnostics.ScheduleStatisticsPublication(solveHandle);
            IncrementalContactPipelineConfiguration
                diagnosticsConfiguration =
                    IncrementalContactPipelineExperimentRuntime
                        .CaptureConfiguration(
                            completedStep.WorldId,
                            unitCount,
                            SystemAPI.Time.DeltaTime,
                            contactSolverSettings.SoftAvoidanceShell,
                            contactSolverSettings,
                            effectiveTimestepContactSetCache,
                            effectivePersistentContactCache);
            incrementalHandle =
                diagnostics.ScheduleIncrementalPublication(
                    completedStep,
                    diagnosticsConfiguration,
                    _incrementalDiagnosticsEntity,
                    GetComponentLookup<
                        IncrementalContactPipelineSnapshot>(false),
                    solveHandle);

            applyHandle = new ApplyFlowMovementJob
            {
                Results = physicsStep.OutputBodies,
                CrowdStepVersion = simulationStepId
            }.ScheduleParallel(_movementQuery, outputReadyHandle);

            // DIAG (incident index desync probe):
            // GatherAndApplyParallelJacobiBodiesJob stamps CorrectedBodyFlags
            // when offsets/indices/pairs run out of sync.
#if RTS_CONTACT_DIAGNOSTICS
            if (usesJacobiSolver)
            {
                solveHandle.Complete();
                if (physicsStep.TryGetIncidentIndexDesync(out var desync))
                {
                    Debug.LogError(
                        "[IncidentIndexDesync] " +
                        desync.TotalOutOfRange + "/" + desync.BodyCount +
                        " bodies out-of-range: offsetsEnd>list=" +
                        desync.OffsetsOutOfRange +
                        " pairIndex>=Pairs=" +
                        desync.PairIndexOutOfRange +
                        " pairIndex>=Corrections=" +
                        desync.CorrectionIndexOutOfRange +
                        " | Ensure.PairCount=" +
                        desync.ExpectedPairCount +
                        " Ensure.BodyCount=" +
                        desync.ExpectedBodyCount +
                        " Ensure.IsValid=" + desync.IndexIsValid +
                        " | TimestepContactPairs.Length=" +
                        desync.ContactPairCount +
                        " JacobiPairCorrections.Length=" +
                        desync.CorrectionCount +
                        " ActiveIncidentPairIndices.Length=" +
                        desync.IncidentPairIndexCount +
                        " Bodies.Length=" + unitCount);
                }
            }
#endif

            JobHandle runtimeDispose =
                physicsStep.Dispose(applyHandle, solveHandle);
            stepReleased = true;
            JobHandle diagnosticsDispose = diagnostics.Dispose(
                solveHandle,
                statisticsHandle,
                incrementalHandle);
            diagnosticsReleased = true;
            Dependency = JobHandle.CombineDependencies(
                runtimeDispose,
                diagnosticsDispose);
        }
        catch
        {
            JobHandle scheduledReaders = JobHandle.CombineDependencies(
                JobHandle.CombineDependencies(
                    intentHandle,
                    solveHandle,
                    outputReadyHandle),
                JobHandle.CombineDependencies(
                    statisticsHandle,
                    incrementalHandle,
                    applyHandle));
            scheduledReaders.Complete();
            if (physicsStep != null && !stepReleased)
                physicsStep.Dispose(default, default).Complete();
            if (diagnostics != null && !diagnosticsReleased)
                diagnostics.Dispose(default, default, default).Complete();
            _physicsRuntime.Reset();
            SimulationUpdateMarker.End();
            SimulationTotalMarker.End();
            throw;
        }

        // 主线程调度部分到此结束——RTS.Simulation.Update 不含 Worker job 执行。
        SimulationUpdateMarker.End();

#if RTS_CONTACT_DIAGNOSTICS
        // 诊断构建测量完整管线 wall time；Release 让 Dependency 正常跨系统重叠。
        Dependency.Complete();
#endif
        SimulationTotalMarker.End();
    }

    private uint NextSimulationStepId()
    {
        _simulationStepId = _simulationStepId == uint.MaxValue
            ? 1u
            : _simulationStepId + 1u;
        return _simulationStepId;
    }

    private void ResetPersistentContactCaches()
    {
        Dependency.Complete();
        _physicsRuntime.Reset();
    }
}
}
