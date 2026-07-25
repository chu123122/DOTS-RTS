using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
public enum ContactPipelineLifecycleOperation : byte
{
    InitializeSerial,
    InitializeParallel
}

[BurstCompile]
public partial struct ContactPipelineLifecycleJob : IJob
{
    public ContactPipelineLifecycleOperation Operation;
    public ContactPipelineConfiguration Configuration;
    public NativeReference<SerialContactPipelineControlState> SerialControl;
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
    public NativeList<Stage3ContactIterationDiagnostic> IterationDiagnostics;
    public NativeList<Stage3ContactPairDiagnostic> PairDiagnostics;
    public NativeReference<Stage3SelectedBodyDiagnostic> SelectedBodyDiagnostic;
    public NativeList<SimulationDebuggerPairSample> SimulationDebuggerSelectedPairs;
#else
    public NativeReference<IncrementalContactPipelineStatistics> IncrementalStatistics { get => default; set { } }
    public NativeReference<PredictiveDiscContactStatistics> Statistics { get => default; set { } }
#endif
    private float DeltaTime => Configuration.DeltaTime;
    private int SubstepCount => Configuration.SubstepCount;
    private bool EnablePersistentContactCache => Configuration.EnablePersistentContactCache;
    private void StoreIncrementalStatistics(IncrementalContactPipelineStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        IncrementalStatistics.Value = value;
#endif
    }
    private void StoreContactStatistics(PredictiveDiscContactStatistics value)
    {
#if RTS_CONTACT_DIAGNOSTICS
        Statistics.Value = value;
#endif
    }
    private void ResetContactDiagnosticsCapture()
    {
#if RTS_CONTACT_DIAGNOSTICS
        if (IterationDiagnostics.IsCreated) IterationDiagnostics.Clear();
        if (PairDiagnostics.IsCreated) PairDiagnostics.Clear();
        if (SelectedBodyDiagnostic.IsCreated) SelectedBodyDiagnostic.Value = default;
        if (SimulationDebuggerSelectedPairs.IsCreated) SimulationDebuggerSelectedPairs.Clear();
#endif
    }
    public void Execute()
    {
        if (Operation == ContactPipelineLifecycleOperation.InitializeParallel)
        {
            InitializeP1P6Pipeline(RuntimeState);
            return;
        }
        SerialContactPipelineControlState control = new SerialContactPipelineControlState
        {
            IsValid = (byte)(DeltaTime / math.max(1, SubstepCount) > 0f ? 1 : 0),
#if RTS_CONTACT_DIAGNOSTICS
            SolverStartTimestamp = Unity.Profiling.LowLevel.Unsafe.ProfilerUnsafeUtility.Timestamp
#endif
        };
        SerialControl.Value = control;
        PredictiveDiscContactStatistics statistics = new PredictiveDiscContactStatistics
        {
            TimestepContactSetFirstEscapeSubstep = -1
        };
        ResetContactDiagnosticsCapture();
        StoreIncrementalStatistics(default);
        StoreContactStatistics(statistics);
        ActiveIncidentIndexState.Value = default;
    }
}
}
