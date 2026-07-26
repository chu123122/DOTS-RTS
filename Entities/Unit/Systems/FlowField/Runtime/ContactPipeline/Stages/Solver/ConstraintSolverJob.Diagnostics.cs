using Unity.Collections;
using Unity.Entities;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Diagnostics-only field declarations and telemetry accessors for
/// <see cref="ConstraintSolverJob"/>. Kept in a separate partial so the
/// gameplay-critical solver struct body stays free of diagnostics noise;
/// every member here is compile-gated by RTS_CONTACT_DIAGNOSTICS.
/// </summary>
public partial struct ConstraintSolverJob
{
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<ParallelJacobiIterationTelemetry> IterationState;
    public NativeList<JacobiBlockTelemetry> BlockStatistics;

    public Entity DiagnosticSelectedEntity;
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<ContactPairDiagnostic> PairDiagnostics;
    public NativeReference<SelectedBodyContactDiagnostic> SelectedBodyDiagnostic;
    public NativeArray<ContactHeatSample> HeatSamples;
    public SimulationDebuggerCaptureMask SimulationDebuggerCaptureMask;
    public int SimulationDebuggerMaximumPairs;
    public NativeList<SimulationDebuggerPairSample> SimulationDebuggerSelectedPairs;
    public NativeList<ParallelSimulationDebuggerPairCapture> ParallelSimulationDebuggerPairCandidates;
    public NativeList<SimulationDebuggerPairSample> ParallelSimulationDebuggerPairScratch;
    public NativeReference<SimulationDebuggerUnitSample> SimulationDebuggerSelectedUnit;
    public NativeReference<byte> SimulationDebuggerSelectedUnitValid;
#endif

    private IncrementalContactPipelineStatistics LoadIncrementalStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        EnableDiagnostics ? IncrementalStatistics.Value : default;
#else
        default;
#endif
    private void StoreIncrementalStatistics(IncrementalContactPipelineStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (EnableDiagnostics) IncrementalStatistics.Value = value;
#endif
    }
    private PredictiveDiscContactStatistics LoadContactStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        EnableDiagnostics ? Statistics.Value : default;
#else
        default;
#endif
    private void StoreContactStatistics(PredictiveDiscContactStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (EnableDiagnostics) Statistics.Value = value;
#endif
    }
}
}
