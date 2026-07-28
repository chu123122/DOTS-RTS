using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
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
    // RTS.Simulation.Total：含末尾 Dependency.Complete() 的 wall time——模拟管线
    //   从调度到所有 job 跑完的真实耗时，用于基准/简历数据。强制 Complete 会减少
    //   job 与其他系统（渲染）的重叠，所以这只适合 A/B 基准，不代表正常帧表现。
    // TODO: 后续把 Total 的 Complete 用 #if RTS_CONTACT_DIAGNOSTICS 包起来，
    //   诊断关时回到无 Complete 的正常重叠；当前先不包，直接测试。
    private static readonly ProfilerMarker SimulationUpdateMarker =
        new ProfilerMarker("RTS.Simulation.Update");
    private static readonly ProfilerMarker SimulationTotalMarker =
        new ProfilerMarker("RTS.Simulation.Total");

    private EntityQuery _movementQuery;
    private InteractionCandidateStore _candidateStore;
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
        _candidateStore = InteractionCandidateStore.Create();
        CreatePersistentDiagnostics();
        ulong worldId = SimulationDebuggerWorldIdentity.FromSequenceNumber(
            World.Unmanaged.SequenceNumber);
        SimulationDebuggerRuntime.RegisterWorld(worldId);
        IncrementalContactPipelineExperimentRuntime.RegisterWorld(worldId);
    }

    protected override void OnDestroy()
    {
        Dependency.Complete();
        _candidateStore.Dispose();
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
        Dependency.Complete();
        if (_candidateStore.RequiresCapacity(unitCount))
            _candidateStore.EnsureCapacity(unitCount);

        bool usesJacobiScratch =
            contactSolverSettings.ContactPositionSolver ==
            ContactPositionSolverMode.Jacobi;
        bool useParallelJacobi = usesJacobiScratch;
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
                contactSolverSettings);
        CrowdStepBodyResources body = CrowdStepBodyResources.Create(unitCount);
        InteractionCertificationFrameResources certificationResources =
            InteractionCertificationFrameResources.Create(unitCount);
        SoftAvoidanceFrameResources softResources =
            SoftAvoidanceFrameResources.Create(unitCount);
        ConstraintSolverFrameResources solverResources =
            ConstraintSolverFrameResources.Create(unitCount);
        ContactPipelineExecutionResources executionResources =
            ContactPipelineExecutionResources.Create(unitCount);

        ComponentLookup<PhysicsCollider> colliderLookup =
            SystemAPI.GetComponentLookup<PhysicsCollider>(isReadOnly: true);
        colliderLookup.Update(this);
        JobHandle footprintHandle = new CalculateUnitCollisionFootprintJob
        {
            PhysicsColliderLookup = colliderLookup,
            FallbackCellSize = gridComponent.CellRadius * 2f,
            CollisionFootprints = body.CollisionFootprints
        }.ScheduleParallel(_movementQuery, Dependency);

        FlowGridGeometry gridGeometry = new FlowGridGeometry(
            gridComponent.GridOrigin,
            gridComponent.GridDimensions,
            gridComponent.CellRadius);
        JobHandle intentHandle = new BuildCrowdMotionIntentJob
        {
            NavigationCells = gridComponent.Grid,
            NavigationGrid = gridGeometry,
            ActiveRequestVersion = flowFieldRuntimeState.ActiveRequestVersion,
            CollisionFootprints = body.CollisionFootprints,
            Bodies = body.Bodies,
            NavigationStates = body.NavigationStates,
            MotionIntents = body.MotionIntents
        }.ScheduleParallel(_movementQuery, footprintHandle);

        JobHandle initializeHandle = new InitializeCrowdStepStateJob
        {
            Bodies = body.Bodies,
            MotionEvidence = body.MotionEvidence,
            StepStates = body.StepStates
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
        SerialContactPipelineLifecycleJob serialLifecycle =
            executionResources.CreateSerialLifecycleJob(
                configuration,
                solverResources,
                diagnostics,
                _simulationDebuggerSelectedPairs);
        ParallelContactPipelineLifecycleJob parallelLifecycle =
            _candidateStore.CreateParallelLifecycleJob(
                configuration,
                executionResources,
                solverResources,
                diagnostics,
                _simulationDebuggerSelectedPairs);
        InteractionCertificationJob certification = certificationResources.CreateJob(
            configuration,
            gridComponent,
            body,
            _candidateStore,
            solverResources,
            executionResources,
            diagnostics,
            selectedEntity);
        MotionIntegrationJob motion = body.CreateMotionJob(
            configuration,
            gridComponent,
            diagnostics);
        SoftAvoidanceJob softAvoidance = softResources.CreateJob(
            configuration,
            gridComponent,
            body,
            certificationResources,
            solverResources,
            executionResources,
            diagnostics);
        ConstraintSolverJob constraintSolver = solverResources.CreateJob(
            configuration,
            gridComponent,
            body,
            certificationResources,
            executionResources,
            diagnostics,
            selectedEntity,
            captureMask,
            maximumVisualizedPairs,
            _simulationDebuggerSelectedPairs,
            _simulationDebuggerSelectedUnit,
            _simulationDebuggerSelectedUnitValid);
        CrowdContactPipelineScheduler solver = new CrowdContactPipelineScheduler
        {
            Configuration = configuration,
            SerialLifecycle = serialLifecycle,
            ParallelLifecycle = parallelLifecycle,
            Certification = certification,
            Motion = motion,
            SoftAvoidance = softAvoidance,
            ConstraintSolver = constraintSolver
        };

        JobHandle solveHandle;
        if (useParallelJacobi)
        {
#if RTS_CONTACT_DIAGNOSTICS
            solveHandle = solver.ScheduleParallelJacobiP1P6(
                executionResources.ParallelJacobiRuntimeState,
                executionResources.ParallelJacobiIterationState,
                executionResources.ParallelJacobiBlockTelemetry,
                initializeHandle);
#else
            solveHandle = solver.ScheduleParallelJacobiP1P6(
                executionResources.ParallelJacobiRuntimeState,
                initializeHandle);
#endif
        }
        else
        {
            solveHandle = solver.ScheduleSerial(initializeHandle);
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

        World.GetExistingSystemManaged<FlowFieldBakeSystem>()
            ?.RegisterActiveGridReader(solveHandle);

        JobHandle resultHandle = new BuildCrowdBodyResultsJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            Bodies = body.Bodies,
            NavigationStates = body.NavigationStates,
            StepStates = body.StepStates,
            Results = body.Results
        }.Schedule(unitCount, 64, solveHandle);

        JobHandle applyHandle = new ApplyFlowMovementJob
        {
            Results = body.Results
        }.ScheduleParallel(_movementQuery, resultHandle);

        // DIAG (incident index desync probe): GatherAndApplyParallelJacobiBodiesJob
        // stamps CorrectedBodyFlags when offsets/indices/pairs run out of sync.
        // 2 = offsets end > IncidentPairIndices.Length (offsets built for more pairs)
        // 3 = stored pairIndex >= Pairs.Length (index built against larger pair view)
        // 4 = stored pairIndex >= Corrections.Length (corrections view too short)
        // Complete the solver jobs and tally per-mode here so the desync surfaces
        // as a Debug.LogError instead of a silent Burst abort. Remove once the root
        // cause (deferred incident-index timing) is fixed.
        if (useParallelJacobi)
        {
            solveHandle.Complete();
            int flag2 = 0, flag3 = 0, flag4 = 0;
            NativeArray<byte> flags = solverResources.CorrectedBodyFlags;
            for (int i = 0; i < flags.Length; i++)
            {
                byte f = flags[i];
                if (f == 2) flag2++;
                else if (f == 3) flag3++;
                else if (f == 4) flag4++;
            }
            int desyncCount = flag2 + flag3 + flag4;
            if (desyncCount > 0)
            {
                ActiveIncidentIndexState incidentState = solverResources.ActiveIncidentIndexState.Value;
                Debug.LogError(
                    "[IncidentIndexDesync] " + desyncCount + "/" + flags.Length +
                    " bodies out-of-range: offsetsEnd>list=" + flag2 +
                    " pairIndex>=Pairs=" + flag3 +
                    " pairIndex>=Corrections=" + flag4 +
                    " | Ensure.PairCount=" + incidentState.PairCount +
                    " Ensure.BodyCount=" + incidentState.BodyCount +
                    " Ensure.IsValid=" + incidentState.IsValid +
                    " | TimestepContactPairs.Length=" + certificationResources.TimestepContactPairs.Length +
                    " JacobiPairCorrections.Length=" + solverResources.JacobiPairCorrections.Length +
                    " ActiveIncidentPairIndices.Length=" + solverResources.ActiveIncidentPairIndices.Length +
                    " Bodies.Length=" + unitCount);
            }
        }

        JobHandle runtimeDispose = body.Dispose(applyHandle);
        runtimeDispose = JobHandle.CombineDependencies(
            runtimeDispose,
            certificationResources.Dispose(solveHandle));
        runtimeDispose = JobHandle.CombineDependencies(
            runtimeDispose,
            softResources.Dispose(solveHandle));
        runtimeDispose = JobHandle.CombineDependencies(
            runtimeDispose,
            solverResources.Dispose(solveHandle));
        runtimeDispose = JobHandle.CombineDependencies(
            runtimeDispose,
            executionResources.Dispose(solveHandle));
        JobHandle diagnosticsDispose = DisposeContactDiagnosticsFrameResources(
            diagnostics,
            solveHandle,
            diagnosticsPublish.Statistics,
            diagnosticsPublish.Incremental);
        Dependency = JobHandle.CombineDependencies(
            runtimeDispose,
            diagnosticsDispose);

        // 主线程调度部分到此结束——RTS.Simulation.Update 不含 Worker job 执行。
        SimulationUpdateMarker.End();

        // TODO: 后续用 #if RTS_CONTACT_DIAGNOSTICS 包裹这个 Complete，诊断关时移除，
        //   让正常帧恢复 job 与其他系统（渲染）的重叠。当前先不包，直接量 wall time。
        Dependency.Complete();
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
        _candidateStore.Reset();
    }
}
}
