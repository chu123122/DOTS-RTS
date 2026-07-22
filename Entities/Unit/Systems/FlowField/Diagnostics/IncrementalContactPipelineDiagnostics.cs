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
/// Effective configuration attached to the same completed timestep as the
/// diagnostics counters. Strings are fixed-size so the snapshot remains an
/// unmanaged ECS component and can be written by a job.
/// </summary>
public struct IncrementalContactPipelineConfiguration
{
    public FixedString64Bytes ExperimentId;
    public FixedString64Bytes Scenario;
    public FixedString64Bytes ConfigurationLabel;

    public int UnitCount;
    public int SubstepCount;
    public int IterationCount;
    public float DeltaTime;
    public float GuardEnvelopeMargin;
    public float PredictiveSkin;
    public float TimestepContactMargin;
    public float SoftAvoidanceShell;

    public byte TimestepCacheEnabled;
    public byte CrossFrameTopologyEnabled;
    public byte PredictiveContactsEnabled;
    public byte DiagnosticsEnabled;
}

/// <summary>
/// Unified ECS-visible diagnostics snapshot. Solver, compatibility and
/// incremental-pipeline statistics are published from the same completed job,
/// so the UI/CSV layer never joins counters from different timesteps.
/// </summary>
public struct IncrementalContactPipelineSnapshot : IComponentData
{
    public int SchemaVersion;
    public IncrementalContactPipelineConfiguration Configuration;
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
        IncrementalContactPipelineConfiguration configuration,
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
            Configuration = configuration,
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
    public IncrementalContactPipelineConfiguration Configuration;
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
            Configuration,
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
        IncrementalContactPipelineCsvRecorderRuntime.TryRecord(latest);
    }
}
}
