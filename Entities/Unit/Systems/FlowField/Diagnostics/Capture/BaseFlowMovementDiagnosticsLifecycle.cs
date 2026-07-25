using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
internal struct ContactDiagnosticsFrameResources
{
#if RTS_CONTACT_DIAGNOSTICS
    public NativeList<BodyPair> IncrementalOracleContactPairs;
    public NativeList<ParallelSimulationDebuggerPairCapture> ParallelPairCandidates;
    public NativeList<SimulationDebuggerPairSample> ParallelPairScratch;
    public NativeList<Stage3ContactIterationDiagnostic> Iterations;
    public NativeList<Stage3ContactPairDiagnostic> Pairs;
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBody;
    public NativeArray<Stage3ContactHeatSample> HeatSamples;
    public NativeReference<PredictiveDiscContactStatistics> ContactStatistics;
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
#else
    public NativeList<BodyPair> IncrementalOracleContactPairs { get => default; set { } }
    public NativeList<ParallelSimulationDebuggerPairCapture> ParallelPairCandidates { get => default; set { } }
    public NativeList<SimulationDebuggerPairSample> ParallelPairScratch { get => default; set { } }
    public NativeList<Stage3ContactIterationDiagnostic> Iterations { get => default; set { } }
    public NativeList<Stage3ContactPairDiagnostic> Pairs { get => default; set { } }
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBody { get => default; set { } }
    public NativeArray<Stage3ContactHeatSample> HeatSamples { get => default; set { } }
    public NativeReference<PredictiveDiscContactStatistics> ContactStatistics { get => default; set { } }
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics { get => default; set { } }
#endif
}

internal struct ContactDiagnosticsPublishHandles
{
    public JobHandle Statistics;
    public JobHandle Incremental;
}

public abstract partial class BaseFlowMovementSystem
{
#if RTS_CONTACT_DIAGNOSTICS
    private NativeList<SimulationDebuggerPairSample> _simulationDebuggerSelectedPairs;
    private NativeReference<SimulationDebuggerUnitSample> _simulationDebuggerSelectedUnit;
    private NativeReference<byte> _simulationDebuggerSelectedUnitValid;
    private Entity _incrementalDiagnosticsEntity;
#else
    private NativeList<SimulationDebuggerPairSample> _simulationDebuggerSelectedPairs { get => default; set { } }
    private NativeReference<SimulationDebuggerUnitSample> _simulationDebuggerSelectedUnit { get => default; set { } }
    private NativeReference<byte> _simulationDebuggerSelectedUnitValid { get => default; set { } }
    private Entity _incrementalDiagnosticsEntity { get => Entity.Null; set { } }
#endif

    private void CreatePersistentDiagnostics()
    {
#if RTS_CONTACT_DIAGNOSTICS
        _simulationDebuggerSelectedPairs = new NativeList<SimulationDebuggerPairSample>(64, Allocator.Persistent);
        _simulationDebuggerSelectedUnit = new NativeReference<SimulationDebuggerUnitSample>(Allocator.Persistent);
        _simulationDebuggerSelectedUnitValid = new NativeReference<byte>(Allocator.Persistent);
        _incrementalDiagnosticsEntity = EntityManager.CreateEntity(
            typeof(IncrementalContactPipelineSnapshot),
            typeof(PredictiveDiscContactStatistics),
            typeof(ShadowNeighborCacheStatistics),
            typeof(Stage3ContactDiagnosticSelection),
            typeof(Stage3SelectedBodyDiagnostic));
        EntityManager.SetComponentData(
            _incrementalDiagnosticsEntity,
            new Stage3ContactDiagnosticSelection { SelectedEntity = Entity.Null });
        EntityManager.AddBuffer<Stage3ContactIterationDiagnostic>(
            _incrementalDiagnosticsEntity);
        EntityManager.AddBuffer<Stage3ContactPairDiagnostic>(
            _incrementalDiagnosticsEntity);
        EntityManager.AddBuffer<Stage3ContactHeatSample>(
            _incrementalDiagnosticsEntity);
#endif
    }

    private void DisposePersistentDiagnostics()
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (EntityManager.Exists(_incrementalDiagnosticsEntity)) EntityManager.DestroyEntity(_incrementalDiagnosticsEntity);
        if (_simulationDebuggerSelectedPairs.IsCreated) _simulationDebuggerSelectedPairs.Dispose();
        if (_simulationDebuggerSelectedUnit.IsCreated) _simulationDebuggerSelectedUnit.Dispose();
        if (_simulationDebuggerSelectedUnitValid.IsCreated) _simulationDebuggerSelectedUnitValid.Dispose();
#endif
    }

    private Entity ResolveDiagnosticSelectedEntity(ulong worldId)
    {
#if RTS_CONTACT_DIAGNOSTICS
        Entity selected = SimulationDebuggerRuntime.SelectedEntityFor(worldId);
        if (SystemAPI.TryGetSingleton(out Stage3ContactDiagnosticSelection selection) && selection.SelectedEntity != Entity.Null)
        {
            selected = selection.SelectedEntity;
            SimulationDebuggerRuntime.SetSelectedEntityFor(worldId, selected);
        }
        return selected;
#else
        return Entity.Null;
#endif
    }

    private static ContactDiagnosticsFrameResources CreateContactDiagnosticsFrameResources(int unitCount, UnitContactSolverSettings settings)
    {
        ContactDiagnosticsFrameResources scratch = default;
#if RTS_CONTACT_DIAGNOSTICS
        scratch.IncrementalOracleContactPairs = new NativeList<BodyPair>(math.max(unitCount * 4, 1), Allocator.TempJob);
        scratch.ParallelPairCandidates = new NativeList<ParallelSimulationDebuggerPairCapture>(math.max(unitCount * 4, 1), Allocator.TempJob);
        scratch.ParallelPairScratch = new NativeList<SimulationDebuggerPairSample>(math.max(unitCount, 1), Allocator.TempJob);
        scratch.Iterations = new NativeList<Stage3ContactIterationDiagnostic>(math.max(settings.SubstepCount * settings.IterationCount, 1), Allocator.TempJob);
        scratch.Pairs = new NativeList<Stage3ContactPairDiagnostic>(math.max(unitCount * 2, 1), Allocator.TempJob);
        scratch.SelectedBody = new NativeReference<Stage3SelectedBodyDiagnostic>(Allocator.TempJob);
        scratch.HeatSamples = new NativeArray<Stage3ContactHeatSample>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory);
        scratch.ContactStatistics = new NativeReference<PredictiveDiscContactStatistics>(Allocator.TempJob);
        scratch.IncrementalStatistics = new NativeReference<IncrementalContactPipelineStatistics>(Allocator.TempJob);
#endif
        return scratch;
    }

    private ContactDiagnosticsPublishHandles ScheduleContactDiagnosticsPublication(
        ContactDiagnosticsFrameResources scratch,
        NativeReference<PredictiveDiscContactStatistics> contactStatistics,
        NativeReference<IncrementalContactPipelineStatistics> incrementalStatistics,
        CompletedSimulationStepMetadata completedStep,
        int unitCount,
        float deltaTime,
        float softAvoidanceShell,
        UnitContactSolverSettings solverSettings,
        bool effectiveTimestepContactSetCache,
        bool effectivePersistentContactCache,
        JobHandle solveContactHandle)
    {
#if RTS_CONTACT_DIAGNOSTICS
        JobHandle statistics = new PublishPredictiveDiscContactStatisticsJob
        {
            Source = contactStatistics, SelectedBodySource = scratch.SelectedBody,
            IterationSource = scratch.Iterations, PairSource = scratch.Pairs, HeatSource = scratch.HeatSamples
        }.Schedule(solveContactHandle);
        JobHandle incremental = new PublishIncrementalContactPipelineStatisticsJob
        {
            CompletedStep = completedStep,
            Configuration = IncrementalContactPipelineExperimentRuntime.CaptureConfiguration(completedStep.WorldId, unitCount, deltaTime, softAvoidanceShell, solverSettings, effectiveTimestepContactSetCache, effectivePersistentContactCache),
            SolverSource = contactStatistics, Source = incrementalStatistics,
            Target = _incrementalDiagnosticsEntity,
            SnapshotLookup = GetComponentLookup<IncrementalContactPipelineSnapshot>(false)
        }.Schedule(solveContactHandle);
        return new ContactDiagnosticsPublishHandles { Statistics = statistics, Incremental = incremental };
#else
        return new ContactDiagnosticsPublishHandles { Statistics = solveContactHandle, Incremental = solveContactHandle };
#endif
    }


    private bool TryGetCompletedIncrementalContactSnapshot(
        out IncrementalContactPipelineSnapshot snapshot)
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (_incrementalDiagnosticsEntity != Entity.Null &&
            EntityManager.Exists(_incrementalDiagnosticsEntity) &&
            EntityManager.HasComponent<IncrementalContactPipelineSnapshot>(
                _incrementalDiagnosticsEntity))
        {
            snapshot = EntityManager.GetComponentData<IncrementalContactPipelineSnapshot>(
                _incrementalDiagnosticsEntity);
            return snapshot.Statistics.Timestep != 0;
        }
#endif
        snapshot = default;
        return false;
    }

    private static JobHandle DisposeContactDiagnosticsFrameResources(ContactDiagnosticsFrameResources scratch, JobHandle solveContactHandle, JobHandle publishStatisticsHandle, JobHandle publishIncrementalStatisticsHandle)
    {
#if RTS_CONTACT_DIAGNOSTICS
        JobHandle a = scratch.IncrementalOracleContactPairs.IsCreated ? scratch.IncrementalOracleContactPairs.Dispose(solveContactHandle) : default;
        JobHandle b = scratch.ParallelPairCandidates.IsCreated ? scratch.ParallelPairCandidates.Dispose(solveContactHandle) : default;
        JobHandle c = scratch.ParallelPairScratch.IsCreated ? scratch.ParallelPairScratch.Dispose(solveContactHandle) : default;
        JobHandle d = scratch.SelectedBody.IsCreated ? scratch.SelectedBody.Dispose(publishStatisticsHandle) : default;
        JobHandle e = scratch.Iterations.IsCreated ? scratch.Iterations.Dispose(publishStatisticsHandle) : default;
        JobHandle f = scratch.Pairs.IsCreated ? scratch.Pairs.Dispose(publishStatisticsHandle) : default;
        JobHandle g = scratch.HeatSamples.IsCreated ? scratch.HeatSamples.Dispose(publishStatisticsHandle) : default;
        JobHandle published = JobHandle.CombineDependencies(publishStatisticsHandle, publishIncrementalStatisticsHandle);
        JobHandle h = scratch.ContactStatistics.IsCreated ? scratch.ContactStatistics.Dispose(published) : default;
        JobHandle i = scratch.IncrementalStatistics.IsCreated ? scratch.IncrementalStatistics.Dispose(publishIncrementalStatisticsHandle) : default;
        return JobHandle.CombineDependencies(
            JobHandle.CombineDependencies(a,b,c),
            JobHandle.CombineDependencies(d,e,f),
            JobHandle.CombineDependencies(g,h,i));
#else
        return default;
#endif
    }
}
}
