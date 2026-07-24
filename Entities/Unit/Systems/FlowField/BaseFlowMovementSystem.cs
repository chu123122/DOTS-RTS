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
/// Crowd-simulation composition root for one ECS World. It owns stage ordering,
/// resource lifetime and JobHandle dependencies; detailed solver ABI expansion is
/// isolated in BaseFlowMovementComposition.
/// </summary>
public abstract partial class BaseFlowMovementSystem : SystemBase
{
    private EntityQuery _movementQuery;
    private ContactPersistentState _persistentState;
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
            ComponentType.ReadOnly<UnitMoveSpeed>(),
            ComponentType.ReadOnly<UnitMovementSettings>(),
            ComponentType.ReadOnly<UnitContactBody>(),
            ComponentType.ReadOnly<UnitMoveDestination>());
        _persistentState = ContactPersistentState.Create();
        CreatePersistentDiagnostics();
        ulong worldId = unchecked((ulong)World.Unmanaged.SequenceNumber);
        SimulationDebuggerRuntime.RegisterWorld(worldId);
        IncrementalContactPipelineExperimentRuntime.RegisterWorld(worldId);
    }

    protected override void OnDestroy()
    {
        Dependency.Complete();
        _persistentState.Dispose();
        DisposePersistentDiagnostics();
        ulong worldId = unchecked((ulong)World.Unmanaged.SequenceNumber);
        IncrementalContactPipelineExperimentRuntime.UnregisterWorld(worldId);
        SimulationDebuggerRuntime.UnregisterWorld(worldId);
    }

    protected override void OnUpdate()
    {
        ulong worldId = unchecked((ulong)World.Unmanaged.SequenceNumber);
        FlowFieldGrid gridComponent = SystemAPI.GetSingleton<FlowFieldGrid>();
        FlowFieldSettings flowFieldSettings = SystemAPI.GetSingleton<FlowFieldSettings>();
        FlowFieldRuntimeState flowFieldRuntimeState =
            SystemAPI.GetSingleton<FlowFieldRuntimeState>();
        UnitContactSolverSettings contactSolverSettings =
            SystemAPI.GetSingleton<UnitContactSolverSettings>();

        // Publish the completed previous step before freezing controls for this one.
        PublishSimulationDebuggerSnapshot(worldId, gridComponent);
        ApplySimulationDebuggerRuntimeOverrides(
            ref flowFieldSettings,
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

        if (!gridComponent.Grid.IsCreated || flowFieldRuntimeState.ActiveVersion == 0)
            return;

        int unitCount = _movementQuery.CalculateEntityCount();
        if (unitCount == 0)
            return;

        uint simulationStepId = NextSimulationStepId();
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
            selectedEntity,
            captureMask);

        SimulationDebuggerEffectiveSettings effectiveSettings =
            BuildEffectiveSettings(
                flowFieldSettings,
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

        ContactDiagnosticsFrameResources diagnostics =
            CreateContactDiagnosticsFrameResources(
                unitCount,
                contactSolverSettings,
                captureParallelSelectedPairs);
        ContactFrameResources frame = ContactFrameResources.Create(
            unitCount,
            usesJacobiScratch,
            useParallelJacobi);

        ComponentLookup<PhysicsCollider> colliderLookup =
            SystemAPI.GetComponentLookup<PhysicsCollider>(isReadOnly: true);
        colliderLookup.Update(this);
        JobHandle footprintHandle = new CalculateUnitCollisionFootprintJob
        {
            PhysicsColliderLookup = colliderLookup,
            FallbackCellSize = gridComponent.CellRadius * 2f,
            CollisionFootprints = frame.CollisionFootprints
        }.ScheduleParallel(_movementQuery, Dependency);

        // Navigation/arrival stage. Its output still uses the compatibility state
        // array; the next migration splits these fields physically without changing
        // the solver schedule.
        JobHandle intentHandle = new CalculateIndependentFlowForceJob
        {
            Grid = gridComponent.Grid,
            GridOrigin = gridComponent.GridOrigin,
            GridDimensions = gridComponent.GridDimensions,
            CellRadius = gridComponent.CellRadius,
            ActiveRequestVersion = flowFieldRuntimeState.ActiveRequestVersion,
            CollisionFootprints = frame.CollisionFootprints,
            States = frame.States
        }.ScheduleParallel(_movementQuery, footprintHandle);

        JobHandle initializeHandle = new InitializeFlowMovementSolverStateJob
        {
            States = frame.States
        }.Schedule(unitCount, 64, intentHandle);

        ContactPipelineConfiguration configuration =
            ContactPipelineConfiguration.Create(
                worldId,
                simulationStepId,
                SystemAPI.Time.DeltaTime,
                flowFieldSettings,
                contactSolverSettings,
                effectivePersistentContactCache,
                effectiveTimestepContactSetCache);
        SolveXpbdUnitContactsJob solver = ComposeContactSolverJob(
            configuration,
            gridComponent,
            frame,
            diagnostics,
            selectedEntity,
            captureMask,
            maximumVisualizedPairs);

        JobHandle solveHandle;
        if (useParallelJacobi)
        {
#if RTS_CONTACT_DIAGNOSTICS
            solveHandle = solver.ScheduleParallelJacobiP1P6(
                frame.ParallelJacobiRuntimeState,
                frame.ParallelJacobiIterationState,
                frame.ParallelJacobiBlockTelemetry,
                initializeHandle);
#else
            solveHandle = solver.ScheduleParallelJacobiP1P6(
                frame.ParallelJacobiRuntimeState,
                initializeHandle);
#endif
        }
        else
        {
            solveHandle = solver.Schedule(initializeHandle);
        }

        ContactDiagnosticsPublishHandles diagnosticsPublish =
            ScheduleContactDiagnosticsPublication(
                diagnostics,
                diagnostics.ContactStatistics,
                diagnostics.IncrementalStatistics,
                completedStep,
                unitCount,
                SystemAPI.Time.DeltaTime,
                flowFieldSettings.SoftAvoidanceShell,
                contactSolverSettings,
                effectiveTimestepContactSetCache,
                effectivePersistentContactCache,
                solveHandle);

        // FlowField uses double buffering. The active grid can only be recycled
        // after the final solver reader is complete.
        World.GetExistingSystemManaged<FlowFieldBakeSystem>()
            ?.RegisterActiveGridReader(solveHandle);

        JobHandle applyHandle = new ApplyFlowMovementJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            States = frame.States
        }.ScheduleParallel(_movementQuery, solveHandle);

        JobHandle runtimeDispose = frame.Dispose(applyHandle);
        JobHandle diagnosticsDispose = DisposeContactDiagnosticsFrameResources(
            diagnostics,
            solveHandle,
            diagnosticsPublish.Statistics,
            diagnosticsPublish.Incremental);
        Dependency = JobHandle.CombineDependencies(
            runtimeDispose,
            diagnosticsDispose);
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
        _persistentState.Reset();
    }
}
}
