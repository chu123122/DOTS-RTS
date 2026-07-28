using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
#if RTS_CONTACT_DIAGNOSTICS
[BurstCompile]
public partial struct PublishPredictiveDiscContactStatisticsJob : IJobEntity
{
    [ReadOnly] public NativeReference<PredictiveDiscContactStatistics> Source;
    [ReadOnly] public NativeReference<SelectedBodyContactDiagnostic> SelectedBodySource;
    [ReadOnly] public NativeList<ContactIterationDiagnostic> IterationSource;
    [ReadOnly] public NativeList<ContactPairDiagnostic> PairSource;
    [ReadOnly] public NativeArray<ContactHeatSample> HeatSource;

    public void Execute(
        ref PredictiveDiscContactStatistics destination,
        ref ShadowNeighborCacheStatistics shadowDestination,
        ref SelectedBodyContactDiagnostic selectedBodyDestination,
        DynamicBuffer<ContactIterationDiagnostic> iterationDestination,
        DynamicBuffer<ContactPairDiagnostic> pairDestination,
        DynamicBuffer<ContactHeatSample> heatDestination)
    {
        destination = Source.Value;
        // The legacy shadow broad-phase is retired. Keep its compatibility
        // component cleared so old debugger assets cannot display stale data.
        shadowDestination = default;
        selectedBodyDestination = SelectedBodySource.Value;
        iterationDestination.Clear();
        pairDestination.Clear();
        heatDestination.Clear();
        iterationDestination.AddRange(IterationSource.AsArray());
        pairDestination.AddRange(PairSource.AsArray());
        heatDestination.AddRange(HeatSource);
    }
}
#else
/// <summary>
/// Gameplay-only publication facade. Changing the backend from IJobEntity to an
/// empty IJob removes the diagnostic entity query while preserving the existing
/// scheduler call site and source member names.
/// </summary>
[BurstCompile]
public struct PublishPredictiveDiscContactStatisticsJob : IJob
{
    private byte _disabledStorage;
    public NativeReference<PredictiveDiscContactStatistics> Source { get => default; set { } }
    public NativeReference<SelectedBodyContactDiagnostic> SelectedBodySource
    {
        get => default;
        set { }
    }
    public NativeList<ContactIterationDiagnostic> IterationSource
    {
        get => default;
        set { }
    }
    public NativeList<ContactPairDiagnostic> PairSource
    {
        get => default;
        set { }
    }
    public NativeArray<ContactHeatSample> HeatSource
    {
        get => default;
        set { }
    }

    public void Execute() { }
}
#endif
}
