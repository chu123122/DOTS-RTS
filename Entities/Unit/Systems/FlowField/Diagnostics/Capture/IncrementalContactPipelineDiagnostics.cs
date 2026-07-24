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
/// diagnostics counters. Gameplay-only builds retain only a one-byte unmanaged
/// placeholder so the scheduler can keep a single source-compatible call graph.
/// </summary>
public struct IncrementalContactPipelineConfiguration
{
#if RTS_CONTACT_DIAGNOSTICS
    public FixedString64Bytes ExperimentId;
    public FixedString64Bytes Scenario;
    public FixedString64Bytes ConfigurationLabel;
    public int UnitCount;
    public int SubstepCount;
    public int IterationCount;
    public byte ContactPositionSolver;
    public float DeltaTime;
    public float GuardEnvelopeMargin;
    public float PredictiveSkin;
    public float TimestepContactMargin;
    public float SoftAvoidanceShell;
    public byte TimestepCacheEnabled;
    public byte CrossFrameTopologyEnabled;
    public byte PredictiveContactsEnabled;
    public byte DiagnosticsEnabled;
#else
    private byte _disabledStorage;
    public FixedString64Bytes ExperimentId { get => default; set { } }
    public FixedString64Bytes Scenario { get => default; set { } }
    public FixedString64Bytes ConfigurationLabel { get => default; set { } }
    public int UnitCount { get => default; set { } }
    public int SubstepCount { get => default; set { } }
    public int IterationCount { get => default; set { } }
    public byte ContactPositionSolver { get => default; set { } }
    public float DeltaTime { get => default; set { } }
    public float GuardEnvelopeMargin { get => default; set { } }
    public float PredictiveSkin { get => default; set { } }
    public float TimestepContactMargin { get => default; set { } }
    public float SoftAvoidanceShell { get => default; set { } }
    public byte TimestepCacheEnabled { get => default; set { } }
    public byte CrossFrameTopologyEnabled { get => default; set { } }
    public byte PredictiveContactsEnabled { get => default; set { } }
    public byte DiagnosticsEnabled { get => default; set { } }
#endif
}

/// <summary>
/// Unified ECS-visible diagnostics snapshot. Solver, compatibility and
/// incremental-pipeline statistics are published from the same completed job.
/// </summary>
public struct IncrementalContactPipelineSnapshot : IComponentData
{
#if RTS_CONTACT_DIAGNOSTICS
    public int SchemaVersion;
    public IncrementalContactPipelineConfiguration Configuration;
    public PredictiveDiscContactStatistics SolverStatistics;
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
#else
    private byte _disabledStorage;
    public int SchemaVersion { get => default; set { } }
    public IncrementalContactPipelineConfiguration Configuration { get => default; set { } }
    public PredictiveDiscContactStatistics SolverStatistics { get => default; set { } }
    public IncrementalContactPipelineStatistics Statistics { get => default; set { } }
    public IncrementalContactPipelineMode Mode { get => default; set { } }
    public float TopologyDirtyRatio { get => default; set { } }
    public float CleanProxyRatio { get => default; set { } }
    public float RetainedNeighborPairRatio { get => default; set { } }
    public float NeighborToSweptRatio { get => default; set { } }
    public float SweptToCurrentActiveRatio { get => default; set { } }
    public float ActivatedToCorrectedRatio { get => default; set { } }
    public byte OracleHealthy { get => default; set { } }
    public byte Reserved0 { get => default; set { } }
    public ushort Reserved1 { get => default; set { } }
#endif

    public static IncrementalContactPipelineSnapshot From(
        IncrementalContactPipelineConfiguration configuration,
        PredictiveDiscContactStatistics solverStatistics,
        IncrementalContactPipelineStatistics statistics)
    {
#if RTS_CONTACT_DIAGNOSTICS
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
#else
        return default;
#endif
    }
}

[BurstCompile]
public struct PublishIncrementalContactPipelineStatisticsJob : IJob
{
#if RTS_CONTACT_DIAGNOSTICS
    public IncrementalContactPipelineConfiguration Configuration;
    [ReadOnly] public NativeReference<PredictiveDiscContactStatistics> SolverSource;
    [ReadOnly] public NativeReference<IncrementalContactPipelineStatistics> Source;
    public Entity Target;
    public ComponentLookup<IncrementalContactPipelineSnapshot> SnapshotLookup;
#else
    private byte _disabledStorage;
    public IncrementalContactPipelineConfiguration Configuration { get => default; set { } }
    public NativeReference<PredictiveDiscContactStatistics> SolverSource { get => default; set { } }
    public NativeReference<IncrementalContactPipelineStatistics> Source { get => default; set { } }
    public Entity Target { get => Entity.Null; set { } }
    public ComponentLookup<IncrementalContactPipelineSnapshot> SnapshotLookup
    {
        get => default;
        set { }
    }
#endif

    public void Execute()
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (!SnapshotLookup.HasComponent(Target))
            return;
        SnapshotLookup[Target] = IncrementalContactPipelineSnapshot.From(
            Configuration,
            SolverSource.Value,
            Source.Value);
#endif
    }
}

public static class IncrementalContactPipelineDiagnosticsRuntime
{
#if RTS_CONTACT_DIAGNOSTICS
    public static IncrementalContactPipelineSnapshot Latest
    {
        get
        {
            return PublishedPublishedSimulationDiagnosticsRuntime.TryGetLatest(
                       out PublishedSimulationDiagnosticsSnapshot unified)
                ? unified.Pipeline
                : default;
        }
    }
#else
    public static IncrementalContactPipelineSnapshot Latest => default;
#endif
}

#if RTS_CONTACT_DIAGNOSTICS
[UpdateInGroup(typeof(SimulationSystemGroup), OrderFirst = true)]
[UpdateBefore(typeof(RTS.Unit.FlowField.Systems.LocalUnitFlowMovementSystem))]
public partial class IncrementalContactPipelineDiagnosticsSystem : SystemBase
{
    private ulong _lastRecordedGeneration;

    protected override void OnUpdate()
    {
        if (!PublishedPublishedSimulationDiagnosticsRuntime.TryGetLatest(
                out PublishedSimulationDiagnosticsSnapshot published) ||
            published.Generation == _lastRecordedGeneration)
            return;

        _lastRecordedGeneration = published.Generation;
        IncrementalContactPipelineCsvRecorderRuntime.TryRecord(published.Pipeline);
    }
}
#endif
}
