using Unity.Collections;
using RTS.Unit.FlowField;
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
    public NativeList<ContactIterationDiagnostic> Iterations;
    public NativeList<ContactPairDiagnostic> Pairs;
    public NativeReference<SelectedBodyContactDiagnostic> SelectedBody;
    public NativeArray<ContactHeatSample> HeatSamples;
    public NativeReference<PredictiveDiscContactStatistics> ContactStatistics;
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
#else
    private byte _disabledStorage;
    public NativeReference<PredictiveDiscContactStatistics> ContactStatistics
    {
        get => default;
        set { }
    }
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics
    {
        get => default;
        set { }
    }
#endif
}

}
