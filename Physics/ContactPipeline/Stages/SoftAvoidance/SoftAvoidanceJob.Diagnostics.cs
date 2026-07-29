using Unity.Collections;
using Unity.Entities;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Diagnostics-only field declarations and telemetry accessors for
/// <see cref="SoftAvoidanceJob"/>. Physically isolated from the solver body;
/// every member here is compile-gated by RTS_CONTACT_DIAGNOSTICS.
/// </summary>
public partial struct SoftAvoidanceJob
{
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<JacobiBlockTelemetry> BlockStatistics;
    public NativeArray<int> EscapeCountsByBlock;
    public int EscapeBlockCount;
#endif

    // No runtime EnableDiagnostics guard: timing/counting stats must accumulate
    // even with diagnostics off (benchmark needs perf numbers with oracle off).
    // Release builds stay zero-cost via the outer #if RTS_CONTACT_DIAGNOSTICS.
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
