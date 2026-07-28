using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>帧级协调状态，非游戏或候选数据。</summary>
internal struct ContactPipelineExecutionResources
{
    public NativeReference<ParallelJacobiExecutionState> ParallelJacobiRuntimeState;
    public NativeReference<SerialContactPipelineControlState> SerialControlState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<ParallelJacobiIterationTelemetry> ParallelJacobiIterationState;
    public NativeList<JacobiBlockTelemetry> ParallelJacobiBlockTelemetry;
#endif

    public static ContactPipelineExecutionResources Create(int unitCount)
    {
        return new ContactPipelineExecutionResources
        {
            ParallelJacobiRuntimeState = new NativeReference<ParallelJacobiExecutionState>(Allocator.TempJob),
            SerialControlState = new NativeReference<SerialContactPipelineControlState>(Allocator.TempJob),
#if RTS_CONTACT_DIAGNOSTICS
            ParallelJacobiIterationState = new NativeReference<ParallelJacobiIterationTelemetry>(Allocator.TempJob),
            ParallelJacobiBlockTelemetry = new NativeList<JacobiBlockTelemetry>(math.max((unitCount * 4 + 63) / 64, 1), Allocator.TempJob),
#endif
        };
    }

    public SerialContactPipelineLifecycleJob CreateSerialLifecycleJob(
        ContactPipelineConfiguration configuration,
        ConstraintSolverFrameResources solver,
        ContactDiagnosticsFrameResources diagnostics,
        NativeList<SimulationDebuggerPairSample> debuggerSelectedPairs)
    {
        return new SerialContactPipelineLifecycleJob
        {
            Configuration = configuration,
            SerialControl = SerialControlState,
            ActiveIncidentIndexState = solver.ActiveIncidentIndexState,
#if RTS_CONTACT_DIAGNOSTICS
            IncrementalStatistics = diagnostics.IncrementalStatistics,
            Statistics = diagnostics.ContactStatistics,
            IterationDiagnostics = diagnostics.Iterations,
            PairDiagnostics = diagnostics.Pairs,
            SelectedBodyDiagnostic = diagnostics.SelectedBody,
            SimulationDebuggerSelectedPairs = debuggerSelectedPairs,
#endif
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = finalReader;
        if (ParallelJacobiRuntimeState.IsCreated)
            combined = JobHandle.CombineDependencies(combined, ParallelJacobiRuntimeState.Dispose(finalReader));
        if (SerialControlState.IsCreated)
            combined = JobHandle.CombineDependencies(combined, SerialControlState.Dispose(finalReader));
#if RTS_CONTACT_DIAGNOSTICS
        if (ParallelJacobiIterationState.IsCreated)
            combined = JobHandle.CombineDependencies(combined, ParallelJacobiIterationState.Dispose(finalReader));
        if (ParallelJacobiBlockTelemetry.IsCreated)
            combined = JobHandle.CombineDependencies(combined, ParallelJacobiBlockTelemetry.Dispose(finalReader));
#endif
        return combined;
    }
}
}
