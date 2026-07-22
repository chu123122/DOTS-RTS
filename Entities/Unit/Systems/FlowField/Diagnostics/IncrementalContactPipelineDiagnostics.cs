using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Diagnostics
{
public enum IncrementalContactPipelineMode : byte
{
    Disabled,
    IncrementalReuse,
    IncrementalRepair,
    FullRebuild
}

/// <summary>
/// Unified ECS-visible diagnostics snapshot. Solver, legacy compatibility and
/// incremental-pipeline statistics are published from the same completed job,
/// so the UI/CSV layer never joins counters from different timesteps.
/// </summary>
public struct IncrementalContactPipelineSnapshot : IComponentData
{
    public int SchemaVersion;
    public PredictiveDiscContactStatistics SolverStatistics;
    public ShadowNeighborCacheStatistics LegacyBroadPhaseStatistics;
    public IncrementalContactPipelineStatistics Statistics;
    public IncrementalContactPipelineMode Mode;

    public float TopologyDirtyRatio;
    public float CleanProxyRatio;
    public float RetainedNeighborPairRatio;
    public float NeighborToSweptRatio;
    public float SweptToCurrentActiveRatio;
    public float ActivatedToCorrectedRatio;

    public byte OracleHealthy;
    public byte Reserved0;
    public ushort Reserved1;

    public static IncrementalContactPipelineSnapshot From(
        PredictiveDiscContactStatistics solverStatistics,
        ShadowNeighborCacheStatistics legacyBroadPhaseStatistics,
        IncrementalContactPipelineStatistics statistics)
    {
        IncrementalContactPipelineMode mode = IncrementalContactPipelineMode.Disabled;
        if (statistics.UsedFullRebuild != 0)
            mode = IncrementalContactPipelineMode.FullRebuild;
        else if (statistics.IncrementalRepairCount > 0)
            mode = IncrementalContactPipelineMode.IncrementalRepair;
        else if (statistics.UsedIncrementalTopology != 0)
            mode = IncrementalContactPipelineMode.IncrementalReuse;

        return new IncrementalContactPipelineSnapshot
        {
            SchemaVersion = IncrementalContactPipelineStatistics.CurrentSchemaVersion,
            SolverStatistics = solverStatistics,
            LegacyBroadPhaseStatistics = legacyBroadPhaseStatistics,
            Statistics = statistics,
            Mode = mode,
            TopologyDirtyRatio = statistics.ProxyCount > 0
                ? (float)statistics.TopologyDirtyBodyCount / statistics.ProxyCount
                : 0f,
            CleanProxyRatio = statistics.CleanProxyRatio,
            RetainedNeighborPairRatio = statistics.RetainedNeighborPairRatio,
            NeighborToSweptRatio = statistics.NeighborToSweptRatio,
            SweptToCurrentActiveRatio = statistics.SweptToCurrentActiveRatio,
            ActivatedToCorrectedRatio = statistics.ActivatedToCorrectedRatio,
            OracleHealthy = (byte)(statistics.OracleMissingPairCount == 0 ? 1 : 0)
        };
    }
}

[BurstCompile]
public struct PublishIncrementalContactPipelineStatisticsJob : IJob
{
    [ReadOnly] public NativeReference<PredictiveDiscContactStatistics> SolverSource;
    [ReadOnly] public NativeReference<ShadowNeighborCacheStatistics> LegacyBroadPhaseSource;
    [ReadOnly] public NativeReference<IncrementalContactPipelineStatistics> Source;
    public Entity Target;
    public ComponentLookup<IncrementalContactPipelineSnapshot> SnapshotLookup;

    public void Execute()
    {
        if (!SnapshotLookup.HasComponent(Target))
            return;
        SnapshotLookup[Target] = IncrementalContactPipelineSnapshot.From(
            SolverSource.Value,
            LegacyBroadPhaseSource.Value,
            Source.Value);
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
