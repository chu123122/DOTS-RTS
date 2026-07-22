using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Diagnostics
{
/// <summary>
/// ECS-visible snapshot for the incremental contact pipeline. One instance is
/// created by each movement system, avoiding managed writes from a worker job.
/// </summary>
public struct IncrementalContactPipelineSnapshot : IComponentData
{
    public IncrementalContactPipelineStatistics Statistics;
    public float TopologyDirtyRatio;
    public float NeighborToSweptHitRatio;
    public float SweptHitToActiveRatio;
    public float ActiveToCorrectedRatio;

    public static IncrementalContactPipelineSnapshot From(
        IncrementalContactPipelineStatistics statistics)
    {
        return new IncrementalContactPipelineSnapshot
        {
            Statistics = statistics,
            TopologyDirtyRatio = statistics.ProxyCount > 0
                ? (float)statistics.TopologyDirtyBodyCount / statistics.ProxyCount
                : 0f,
            NeighborToSweptHitRatio = statistics.PersistentNeighborPairCount > 0
                ? (float)statistics.SweptHitCount / statistics.PersistentNeighborPairCount
                : 0f,
            SweptHitToActiveRatio = statistics.SweptHitCount > 0
                ? (float)statistics.ActiveConstraintCount / statistics.SweptHitCount
                : 0f,
            ActiveToCorrectedRatio = statistics.ActiveConstraintCount > 0
                ? (float)statistics.CorrectedPairCount / statistics.ActiveConstraintCount
                : 0f
        };
    }
}

[BurstCompile]
public struct PublishIncrementalContactPipelineStatisticsJob : IJob
{
    [ReadOnly] public NativeReference<IncrementalContactPipelineStatistics> Source;
    public Entity Target;
    public ComponentLookup<IncrementalContactPipelineSnapshot> SnapshotLookup;

    public void Execute()
    {
        if (!SnapshotLookup.HasComponent(Target))
            return;
        SnapshotLookup[Target] = IncrementalContactPipelineSnapshot.From(Source.Value);
    }
}

public static class IncrementalContactPipelineDiagnosticsRuntime
{
    public static IncrementalContactPipelineSnapshot Latest { get; internal set; }
}

[UpdateInGroup(typeof(PresentationSystemGroup))]
public partial class IncrementalContactPipelineDiagnosticsSystem : SystemBase
{
    protected override void OnUpdate()
    {
        IncrementalContactPipelineSnapshot latest = default;
        uint latestTimestep = 0;
        foreach (RefRO<IncrementalContactPipelineSnapshot> snapshot
                 in SystemAPI.Query<RefRO<IncrementalContactPipelineSnapshot>>())
        {
            IncrementalContactPipelineSnapshot value = snapshot.ValueRO;
            if (value.Statistics.Timestep < latestTimestep)
                continue;
            latest = value;
            latestTimestep = value.Statistics.Timestep;
        }
        IncrementalContactPipelineDiagnosticsRuntime.Latest = latest;
    }
}
}
