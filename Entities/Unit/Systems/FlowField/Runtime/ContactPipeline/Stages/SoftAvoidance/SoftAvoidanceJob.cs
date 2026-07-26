using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public enum SoftAvoidanceOperation : byte
{
    None,
    SolveSerial,
    PrepareParallelWorkset,
    FinalizeParallel
}

[BurstCompile]
public partial struct SoftAvoidanceJob : IJob
{
    public SoftAvoidanceOperation Operation;
    public ContactPipelineConfiguration Configuration;
    public NativeReference<ParallelJacobiExecutionState> RuntimeState;
    [ReadOnly] public NativeArray<FlowFieldCell> Grid;
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
    public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdBodyStepState> StepStates;
    public NativeList<BodyPair> SoftAvoidancePairs;
    public NativeArray<int> SoftIncidentOffsets;
    public NativeArray<int> SoftIncidentWriteCursors;
    public NativeList<int> SoftIncidentPairIndices;
    public NativeList<SoftAvoidancePairContribution> SoftPairContributions;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<JacobiBlockTelemetry> BlockStatistics;
    public NativeArray<int> EscapeCountsByBlock;
    public int EscapeBlockCount;
#endif
    private float SoftAvoidanceResponseRate => Configuration.SoftAvoidanceResponseRate;
    private float SoftAvoidanceShell => Configuration.SoftAvoidanceShell;
    private float SettledSoftAvoidanceMultiplier => Configuration.SettledSoftAvoidanceMultiplier;
    private SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver => Configuration.SoftAvoidanceVelocitySolver;
    private float RvoTimeHorizon => Configuration.RvoTimeHorizon;
    private bool EnableDiagnostics => Configuration.EnableDiagnostics;
    private bool EnablePersistentContactCache => Configuration.EnablePersistentContactCache;
    private FlowGridGeometry EnvironmentGeometry => new FlowGridGeometry(GridOrigin, GridDimensions, CellRadius);
    private IncrementalContactPipelineStatistics LoadIncrementalStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        EnableDiagnostics ? IncrementalStatistics.Value : default;
#else
        default;
#endif
    private void StoreIncrementalStatistics(IncrementalContactPipelineStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (EnableDiagnostics) IncrementalStatistics.Value = value;
#endif
    }
    private PredictiveDiscContactStatistics LoadContactStatistics() =>
#if RTS_CONTACT_DIAGNOSTICS
        EnableDiagnostics ? Statistics.Value : default;
#else
        default;
#endif
    private void StoreContactStatistics(PredictiveDiscContactStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (EnableDiagnostics) Statistics.Value = value;
#endif
    }
    public void Execute()
    {
        switch (Operation)
        {
            case SoftAvoidanceOperation.SolveSerial:
            {
                float dt = Configuration.DeltaTime / math.max(1, Configuration.SubstepCount);
#if RTS_CONTACT_DIAGNOSTICS
                PredictiveDiscContactStatistics statistics = Statistics.Value;
                IncrementalContactPipelineStatistics incremental = IncrementalStatistics.Value;
#else
                PredictiveDiscContactStatistics statistics = default;
                IncrementalContactPipelineStatistics incremental = default;
#endif
                long start = ProfilerUnsafeUtility.Timestamp;
                CalculateSoftAvoidanceForSubstep(dt, ref statistics, ref incremental);
                statistics.SoftAvoidanceEvaluationCount++;
                statistics.SoftAvoidanceNanoseconds += ContactPipelineMath.TimestampToNanoseconds(
                    ProfilerUnsafeUtility.Timestamp - start);
#if RTS_CONTACT_DIAGNOSTICS
                Statistics.Value = statistics;
                IncrementalStatistics.Value = incremental;
#endif
                break;
            }
            case SoftAvoidanceOperation.PrepareParallelWorkset:
                PrepareP1P6SoftWorkset(RuntimeState
#if RTS_CONTACT_DIAGNOSTICS
                    , BlockStatistics
#endif
                );
                break;
            case SoftAvoidanceOperation.FinalizeParallel:
#if RTS_CONTACT_DIAGNOSTICS
                FinalizeP1P6SoftAvoidance(RuntimeState, BlockStatistics, EscapeCountsByBlock, EscapeBlockCount);
#endif
                break;
        }
    }
}
}
