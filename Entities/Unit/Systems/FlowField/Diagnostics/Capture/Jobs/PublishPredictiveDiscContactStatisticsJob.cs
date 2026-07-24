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
    [ReadOnly] public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBodySource;
    [ReadOnly] public NativeList<Stage3ContactIterationDiagnostic> IterationSource;
    [ReadOnly] public NativeList<Stage3ContactPairDiagnostic> PairSource;
    [ReadOnly] public NativeArray<Stage3ContactHeatSample> HeatSource;

    public void Execute(
        ref PredictiveDiscContactStatistics destination,
        ref ShadowNeighborCacheStatistics shadowDestination,
        ref Stage3SelectedBodyDiagnostic selectedBodyDestination,
        DynamicBuffer<Stage3ContactIterationDiagnostic> iterationDestination,
        DynamicBuffer<Stage3ContactPairDiagnostic> pairDestination,
        DynamicBuffer<Stage3ContactHeatSample> heatDestination)
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
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBodySource
    {
        get => default;
        set { }
    }
    public NativeList<Stage3ContactIterationDiagnostic> IterationSource
    {
        get => default;
        set { }
    }
    public NativeList<Stage3ContactPairDiagnostic> PairSource
    {
        get => default;
        set { }
    }
    public NativeArray<Stage3ContactHeatSample> HeatSource
    {
        get => default;
        set { }
    }

    public void Execute() { }
}
#endif
}
