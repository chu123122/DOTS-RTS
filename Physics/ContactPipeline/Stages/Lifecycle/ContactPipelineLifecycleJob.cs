using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public enum ContactPipelineTimingOperation : byte
{
    Begin,
    EndMotion,
    EndValidationRepair
}

#if RTS_CONTACT_DIAGNOSTICS
[BurstCompile]
public struct ContactPipelineTimingJob : IJob
{
    public ContactPipelineTimingOperation Operation;
    public NativeReference<ContactPipelineExecutionState> RuntimeState;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    [ReadOnly]
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;

    public void Execute()
    {
        ContactPipelineExecutionState runtime = RuntimeState.Value;
        if (runtime.IsValid == 0)
            return;

        long now = Unity.Profiling.LowLevel.Unsafe
            .ProfilerUnsafeUtility.Timestamp;
        if (Operation == ContactPipelineTimingOperation.Begin)
        {
            runtime.StageStartTimestamp = now;
            runtime.StageAccountedStartNanoseconds =
                AccountedCandidateNanoseconds(IncrementalStatistics.Value);
            RuntimeState.Value = runtime;
            return;
        }

        long elapsed = ContactPipelineMath.TimestampToNanoseconds(
            now - runtime.StageStartTimestamp);
        PredictiveDiscContactStatistics statistics = Statistics.Value;
        if (Operation == ContactPipelineTimingOperation.EndMotion)
        {
            statistics.MotionNanoseconds += elapsed;
        }
        else
        {
            long nestedCandidateNanoseconds =
                AccountedCandidateNanoseconds(IncrementalStatistics.Value) -
                runtime.StageAccountedStartNanoseconds;
            statistics.ValidationRepairNanoseconds += math.max(
                0L,
                elapsed - math.max(0L, nestedCandidateNanoseconds));
        }
        Statistics.Value = statistics;
    }

    private static long AccountedCandidateNanoseconds(
        IncrementalContactPipelineStatistics statistics) =>
        statistics.ProxyValidationNanoseconds +
        statistics.FullSweepSourceNanoseconds +
        statistics.PersistentPairMappingNanoseconds +
        statistics.LocalBroadPhaseNanoseconds +
        statistics.PairDiffNanoseconds +
        statistics.FallbackNanoseconds +
        statistics.SweptClassificationNanoseconds +
        statistics.ContactActivationNanoseconds;
}
#endif

/// <summary>
/// Initializes parallel execution and candidate indexes. Every NativeContainer field
/// is required on the parallel path; no optional serial container is carried.
/// </summary>
[BurstCompile]
public struct ContactPipelineLifecycleJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    private bool EnableDiagnostics => Configuration.EnableDiagnostics;
    public NativeReference<ContactPipelineExecutionState> RuntimeState;
    public NativeList<PersistentSweptProxy> PersistentSweptProxies;
    public NativeList<int> PersistentProxyIndexByBody;
    public NativeList<PersistentNeighborPair> PersistentNeighborPairs;
    public NativeList<PersistentPredictiveContact> PersistentPredictiveContacts;
    public NativeParallelMultiHashMap<int, int> PersistentSpatialMembership;
    public NativeReference<uint> PersistentSpatialMembershipEpoch;
    public NativeParallelMultiHashMap<Entity, int> PersistentIncidentPairLookup;
    public NativeReference<uint> PersistentIncidentLookupEpoch;
    public NativeReference<IncrementalContactCacheState> IncrementalCacheState;
    public NativeReference<ActiveIncidentIndexState> ActiveIncidentIndexState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics;
    public NativeReference<PredictiveDiscContactStatistics> Statistics;
    public NativeList<ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<ContactPairDiagnostic> PairDiagnostics;
    public NativeReference<SelectedBodyContactDiagnostic> SelectedBodyDiagnostic;
    public NativeList<SimulationDebuggerPairSample> SimulationDebuggerSelectedPairs;
#endif

    public void Execute()
    {
        ContactPipelineExecutionState runtime = new ContactPipelineExecutionState
        {
            IsValid = 1
        };
#if RTS_CONTACT_DIAGNOSTICS
        IncrementalStatistics.Value = default;
        Statistics.Value = new PredictiveDiscContactStatistics
        {
            TimestepContactSetFirstEscapeSubstep = -1
        };
        runtime.SolverStartTimestamp =
            Unity.Profiling.LowLevel.Unsafe.ProfilerUnsafeUtility.Timestamp;
        if (EnableDiagnostics)
        {
            if (IterationDiagnostics.IsCreated) IterationDiagnostics.Clear();
            if (PairDiagnostics.IsCreated) PairDiagnostics.Clear();
            if (SelectedBodyDiagnostic.IsCreated) SelectedBodyDiagnostic.Value = default;
            if (SimulationDebuggerSelectedPairs.IsCreated) SimulationDebuggerSelectedPairs.Clear();
        }
#endif
        ActiveIncidentIndexState.Value = default;

        if (Configuration.DeltaTime / math.max(1, Configuration.SubstepCount) <= 0f)
            runtime.IsValid = 0;
        if (!Configuration.EnablePersistentContactCache)
        {
            PersistentSweptProxies.Clear();
            PersistentProxyIndexByBody.Clear();
            PersistentNeighborPairs.Clear();
            PersistentPredictiveContacts.Clear();
            PersistentSpatialMembership.Clear();
            PersistentSpatialMembershipEpoch.Value = 0;
            PersistentIncidentPairLookup.Clear();
            PersistentIncidentLookupEpoch.Value = 0;
            IncrementalCacheState.Value = default;
        }
        RuntimeState.Value = runtime;
    }
}
}
