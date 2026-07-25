using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Initializes parallel execution and candidate indexes. Every NativeContainer field
/// is required on the parallel path; no optional serial container is carried.
/// </summary>
[BurstCompile]
public struct ParallelContactPipelineLifecycleJob : IJob
{
    public ContactPipelineConfiguration Configuration;
    public NativeReference<ParallelJacobiExecutionState> RuntimeState;
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
        ParallelJacobiExecutionState runtime = new ParallelJacobiExecutionState
        {
            IsValid = 1
        };
#if RTS_CONTACT_DIAGNOSTICS
        runtime.SolverStartTimestamp =
            Unity.Profiling.LowLevel.Unsafe.ProfilerUnsafeUtility.Timestamp;
        if (IterationDiagnostics.IsCreated) IterationDiagnostics.Clear();
        if (PairDiagnostics.IsCreated) PairDiagnostics.Clear();
        if (SelectedBodyDiagnostic.IsCreated) SelectedBodyDiagnostic.Value = default;
        if (SimulationDebuggerSelectedPairs.IsCreated) SimulationDebuggerSelectedPairs.Clear();
        IncrementalStatistics.Value = default;
        Statistics.Value = new PredictiveDiscContactStatistics
        {
            TimestepContactSetFirstEscapeSubstep = -1
        };
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
