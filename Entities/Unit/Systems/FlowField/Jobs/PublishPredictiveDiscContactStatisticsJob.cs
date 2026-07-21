using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
[BurstCompile]
public partial struct PublishPredictiveDiscContactStatisticsJob : IJobEntity
{
    [ReadOnly] public NativeReference<PredictiveDiscContactStatistics> Source;
    [ReadOnly] public NativeReference<ShadowNeighborCacheStatistics> ShadowSource;
    [ReadOnly] public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBodySource;
    [ReadOnly] public NativeList<Stage3ContactIterationDiagnostic> IterationSource;
    [ReadOnly] public NativeList<Stage3ContactPairDiagnostic> PairSource;

    public void Execute(
        ref PredictiveDiscContactStatistics destination,
        ref ShadowNeighborCacheStatistics shadowDestination,
        ref Stage3SelectedBodyDiagnostic selectedBodyDestination,
        DynamicBuffer<Stage3ContactIterationDiagnostic> iterationDestination,
        DynamicBuffer<Stage3ContactPairDiagnostic> pairDestination)
    {
        destination = Source.Value;
        shadowDestination = ShadowSource.Value;
        selectedBodyDestination = SelectedBodySource.Value;
        iterationDestination.Clear();
        pairDestination.Clear();
        iterationDestination.AddRange(IterationSource.AsArray());
        pairDestination.AddRange(PairSource.AsArray());
    }
}
}
