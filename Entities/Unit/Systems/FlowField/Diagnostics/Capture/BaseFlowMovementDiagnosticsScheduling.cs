using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
internal struct ContactDiagnosticsFrameScratch
{
#if RTS_CONTACT_DIAGNOSTICS
    public NativeList<UnitCollisionPair> IncrementalOracleContactPairs;
    public NativeList<ParallelSimulationDebuggerPairCapture> ParallelPairCandidates;
    public NativeList<SimulationDebuggerPairSample> ParallelPairScratch;
    public NativeList<Stage3ContactIterationDiagnostic> Iterations;
    public NativeList<Stage3ContactPairDiagnostic> Pairs;
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBody;
    public NativeArray<Stage3ContactHeatSample> HeatSamples;
#else
    public NativeList<UnitCollisionPair> IncrementalOracleContactPairs { get => default; set { } }
    public NativeList<ParallelSimulationDebuggerPairCapture> ParallelPairCandidates { get => default; set { } }
    public NativeList<SimulationDebuggerPairSample> ParallelPairScratch { get => default; set { } }
    public NativeList<Stage3ContactIterationDiagnostic> Iterations { get => default; set { } }
    public NativeList<Stage3ContactPairDiagnostic> Pairs { get => default; set { } }
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBody { get => default; set { } }
    public NativeArray<Stage3ContactHeatSample> HeatSamples { get => default; set { } }
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
        _incrementalDiagnosticsEntity = EntityManager.CreateEntity(typeof(IncrementalContactPipelineSnapshot));
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

    private Entity ResolveDiagnosticSelectedEntity()
    {
#if RTS_CONTACT_DIAGNOSTICS
        Entity selected = SimulationDebuggerRuntime.SelectedEntity;
        if (SystemAPI.TryGetSingleton(out Stage3ContactDiagnosticSelection selection) && selection.SelectedEntity != Entity.Null)
        {
            selected = selection.SelectedEntity;
            SimulationDebuggerRuntime.SelectedEntity = selected;
        }
        return selected;
#else
        return Entity.Null;
#endif
    }

    private static bool ShouldCaptureParallelSelectedPairs(bool useParallelJacobi, Entity selectedEntity)
    {
#if RTS_CONTACT_DIAGNOSTICS
        return useParallelJacobi && selectedEntity != Entity.Null &&
               (SimulationDebuggerRuntime.CaptureMask & SimulationDebuggerCaptureMask.SelectedUnit) != 0 &&
               (SimulationDebuggerRuntime.CaptureMask & SimulationDebuggerCaptureMask.SelectedPairs) != 0;
#else
        return false;
#endif
    }

    private static ContactDiagnosticsFrameScratch CreateContactDiagnosticsFrameScratch(int unitCount, UnitContactSolverSettings settings, bool captureParallelSelectedPairs)
    {
        ContactDiagnosticsFrameScratch scratch = default;
#if RTS_CONTACT_DIAGNOSTICS
        scratch.IncrementalOracleContactPairs = new NativeList<UnitCollisionPair>(math.max(unitCount * 4, 1), Allocator.TempJob);
        scratch.ParallelPairCandidates = captureParallelSelectedPairs ? new NativeList<ParallelSimulationDebuggerPairCapture>(math.max(unitCount * 4, 1), Allocator.TempJob) : default;
        scratch.ParallelPairScratch = captureParallelSelectedPairs ? new NativeList<SimulationDebuggerPairSample>(math.max(unitCount, 1), Allocator.TempJob) : default;
        scratch.Iterations = new NativeList<Stage3ContactIterationDiagnostic>(math.max(settings.SubstepCount * settings.IterationCount, 1), Allocator.TempJob);
        scratch.Pairs = new NativeList<Stage3ContactPairDiagnostic>(math.max(unitCount * 2, 1), Allocator.TempJob);
        scratch.SelectedBody = new NativeReference<Stage3SelectedBodyDiagnostic>(Allocator.TempJob);
        scratch.HeatSamples = new NativeArray<Stage3ContactHeatSample>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory);
#endif
        return scratch;
    }

    private ContactDiagnosticsPublishHandles ScheduleContactDiagnosticsPublication(ContactDiagnosticsFrameScratch scratch, NativeReference<PredictiveDiscContactStatistics> contactStatistics, NativeReference<IncrementalContactPipelineStatistics> incrementalStatistics, int unitCount, float deltaTime, float softAvoidanceShell, UnitContactSolverSettings solverSettings, bool effectiveTimestepContactSetCache, bool effectivePersistentContactCache, JobHandle solveContactHandle)
    {
#if RTS_CONTACT_DIAGNOSTICS
        JobHandle statistics = new PublishPredictiveDiscContactStatisticsJob
        {
            Source = contactStatistics, SelectedBodySource = scratch.SelectedBody,
            IterationSource = scratch.Iterations, PairSource = scratch.Pairs, HeatSource = scratch.HeatSamples
        }.Schedule(solveContactHandle);
        JobHandle incremental = new PublishIncrementalContactPipelineStatisticsJob
        {
            Configuration = IncrementalContactPipelineExperimentRuntime.CaptureConfiguration(unitCount, deltaTime, softAvoidanceShell, solverSettings, effectiveTimestepContactSetCache, effectivePersistentContactCache),
            SolverSource = contactStatistics, Source = incrementalStatistics,
            Target = _incrementalDiagnosticsEntity,
            SnapshotLookup = GetComponentLookup<IncrementalContactPipelineSnapshot>(false)
        }.Schedule(solveContactHandle);
        return new ContactDiagnosticsPublishHandles { Statistics = statistics, Incremental = incremental };
#else
        return new ContactDiagnosticsPublishHandles { Statistics = solveContactHandle, Incremental = solveContactHandle };
#endif
    }

    private static JobHandle DisposeContactDiagnosticsFrameScratch(ContactDiagnosticsFrameScratch scratch, JobHandle solveContactHandle, JobHandle publishStatisticsHandle)
    {
#if RTS_CONTACT_DIAGNOSTICS
        JobHandle a = scratch.IncrementalOracleContactPairs.IsCreated ? scratch.IncrementalOracleContactPairs.Dispose(solveContactHandle) : default;
        JobHandle b = scratch.ParallelPairCandidates.IsCreated ? scratch.ParallelPairCandidates.Dispose(solveContactHandle) : default;
        JobHandle c = scratch.ParallelPairScratch.IsCreated ? scratch.ParallelPairScratch.Dispose(solveContactHandle) : default;
        JobHandle d = scratch.SelectedBody.IsCreated ? scratch.SelectedBody.Dispose(publishStatisticsHandle) : default;
        JobHandle e = scratch.Iterations.IsCreated ? scratch.Iterations.Dispose(publishStatisticsHandle) : default;
        JobHandle f = scratch.Pairs.IsCreated ? scratch.Pairs.Dispose(publishStatisticsHandle) : default;
        JobHandle g = scratch.HeatSamples.IsCreated ? scratch.HeatSamples.Dispose(publishStatisticsHandle) : default;
        return JobHandle.CombineDependencies(JobHandle.CombineDependencies(a,b,c), JobHandle.CombineDependencies(d,e,f), g);
#else
        return default;
#endif
    }
}
}
