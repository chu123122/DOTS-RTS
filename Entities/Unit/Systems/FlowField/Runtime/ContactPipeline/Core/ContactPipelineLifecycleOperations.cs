using Unity.Collections;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField.Diagnostics;
namespace RTS.Unit.FlowField.Jobs
{
public partial struct ContactPipelineLifecycleJob
{
    private void InitializeP1P6Pipeline(
        NativeReference<ParallelJacobiExecutionState> runtimeState)
    {
        var runtime = new ParallelJacobiExecutionState
        {
            IsValid = 1
        };
#if RTS_CONTACT_DIAGNOSTICS
        runtime.SolverStartTimestamp = ProfilerUnsafeUtility.Timestamp;
#endif
        var statistics = new PredictiveDiscContactStatistics
        {
            TimestepContactSetFirstEscapeSubstep = -1
        };
        ResetContactDiagnosticsCapture();
        StoreIncrementalStatistics(default);
        StoreContactStatistics(statistics);
        ActiveIncidentIndexState.Value = default;

        if (DeltaTime / math.max(1, SubstepCount) <= 0f)
            runtime.IsValid = 0;
        if (!EnablePersistentContactCache)
        {
            PersistentSweptProxies.Clear();
            PersistentProxyIndexByBody.Clear();
            PersistentNeighborPairs.Clear();
            PersistentPredictiveContacts.Clear();
            if (PersistentSpatialMembership.IsCreated)
                PersistentSpatialMembership.Clear();
            if (PersistentSpatialMembershipEpoch.IsCreated)
                PersistentSpatialMembershipEpoch.Value = 0;
            if (PersistentIncidentPairLookup.IsCreated)
                PersistentIncidentPairLookup.Clear();
            if (PersistentIncidentLookupEpoch.IsCreated)
                PersistentIncidentLookupEpoch.Value = 0;
            IncrementalCacheState.Value = default;
        }
        runtimeState.Value = runtime;
    }
}
}
