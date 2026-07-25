using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>Initializes only serial execution state and optional observation outputs.</summary>
[BurstCompile]
public struct SerialContactPipelineLifecycleJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    public NativeReference<SerialContactPipelineControlState> SerialControl;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<Stage3ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<Stage3ContactPairDiagnostic> PairDiagnostics;
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBodyDiagnostic;
    public NativeList<SimulationDebuggerPairSample> SimulationDebuggerSelectedPairs;
#endif

    public void Execute()
    {
        SerialControl.Value = new SerialContactPipelineControlState
        {
            IsValid = (byte)(Configuration.DeltaTime /
                math.max(1, Configuration.SubstepCount) > 0f ? 1 : 0),
#if RTS_CONTACT_DIAGNOSTICS
            SolverStartTimestamp =
                Unity.Profiling.LowLevel.Unsafe.ProfilerUnsafeUtility.Timestamp
#endif
        };
#if RTS_CONTACT_DIAGNOSTICS
        if (IterationDiagnostics.IsCreated) IterationDiagnostics.Clear();
        if (PairDiagnostics.IsCreated) PairDiagnostics.Clear();
        if (SelectedBodyDiagnostic.IsCreated) SelectedBodyDiagnostic.Value = default;
        if (SimulationDebuggerSelectedPairs.IsCreated) SimulationDebuggerSelectedPairs.Clear();
        IncrementalStatistics.Value = default;
        Statistics.Value = new PredictiveDiscContactStatistics
        {
            TimestepContactSetFirstEscapeSubstep = -1
        };
#endif
        ActiveIncidentIndexState.Value = default;
    }
}
}
