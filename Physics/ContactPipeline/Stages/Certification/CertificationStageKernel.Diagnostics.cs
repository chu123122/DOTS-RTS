using Unity.Collections;
using Unity.Entities;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public struct CertificationDiagnosticsResources
{
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<ContactSolverIterationTelemetry> IterationState;
    public NativeList<JacobiBlockTelemetry> BlockStatistics;
    public NativeList<ParallelSimulationDebuggerPairCapture> ParallelSimulationDebuggerPairCandidates;
    public NativeReference<PersistentClassificationTelemetryState> PersistentClassificationTelemetry;
    public Entity DiagnosticSelectedEntity;
    public NativeList<BodyPair> IncrementalOracleContactPairs;
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<ContactPairDiagnostic> PairDiagnostics;
    public NativeArray<ContactHeatSample> HeatSamples;
#endif
}

internal partial struct CertificationStageKernel
{
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<ContactSolverIterationTelemetry> IterationState;
    public NativeList<JacobiBlockTelemetry> BlockStatistics;
    public NativeList<ParallelSimulationDebuggerPairCapture> ParallelSimulationDebuggerPairCandidates;
    public NativeReference<PersistentClassificationTelemetryState> PersistentClassificationTelemetry;
    public Entity DiagnosticSelectedEntity;
    public NativeList<BodyPair> IncrementalOracleContactPairs;
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<ContactPairDiagnostic> PairDiagnostics;
    public NativeArray<ContactHeatSample> HeatSamples;
#endif
    private IncrementalContactPipelineStatistics LoadIncrementalStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        IncrementalStatistics.Value;
#else
        default;
#endif
    private void StoreIncrementalStatistics(IncrementalContactPipelineStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        IncrementalStatistics.Value = value;
#endif
    }
    private PredictiveDiscContactStatistics LoadContactStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value;
#else
        default;
#endif
    private void StoreContactStatistics(PredictiveDiscContactStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value = value;
#endif
    }
}
}
