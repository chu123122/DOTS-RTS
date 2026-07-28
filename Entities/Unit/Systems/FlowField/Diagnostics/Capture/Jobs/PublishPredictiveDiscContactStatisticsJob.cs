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
        // 旧 shadow broadphase 已退役；清零兼容组件，避免旧调试器读到残留数据。
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
/// 非诊断构建的发布占位：空实现，保留与诊断版相同的字段名和调用点。
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
