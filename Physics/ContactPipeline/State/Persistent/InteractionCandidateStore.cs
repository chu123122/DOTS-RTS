using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>
/// 世界寿命的、未认证候选源。仅生命周期与认证阶段接收这些容器；运动、软避让、求解器 Job 一律不接收。
/// </summary>
internal struct InteractionCandidateStore
{
    public NativeList<PersistentSweptProxy> SweptProxies;
    public NativeList<int> ProxyIndexByBody;
    public NativeList<PersistentNeighborPair> NeighborPairs;
    public NativeList<PersistentPredictiveContact> PredictiveContacts;
    // O(1) 查找索引，替代有序列表+二分查找。全量重建后从 PredictiveContacts 重建；
    // 增量 patch 路径直接写入，无需重排序。
    public NativeHashMap<StableEntityPairKey, PersistentPredictiveContact> PredictiveContactIndex;
    public NativeList<StableEntityPairKey> ActiveContactKeys;
    public NativeList<StableEntityPairKey> SoftAvoidancePairKeys;
    public NativeList<PredictiveContactScheduleEntry> DormantContactSchedule;
    public NativeReference<IncrementalContactCacheState> CacheState;
    public NativeParallelMultiHashMap<Entity, int> IncidentPairLookup;
    public NativeReference<uint> IncidentLookupEpoch;
    public NativeParallelMultiHashMap<int, int> SpatialMembership;
    public NativeReference<uint> SpatialMembershipEpoch;

    public static InteractionCandidateStore Create()
    {
        return new InteractionCandidateStore
        {
            SweptProxies = new NativeList<PersistentSweptProxy>(Allocator.Persistent),
            ProxyIndexByBody = new NativeList<int>(Allocator.Persistent),
            NeighborPairs = new NativeList<PersistentNeighborPair>(Allocator.Persistent),
            PredictiveContacts = new NativeList<PersistentPredictiveContact>(Allocator.Persistent),
            PredictiveContactIndex = new NativeHashMap<StableEntityPairKey, PersistentPredictiveContact>(1, Allocator.Persistent),
            ActiveContactKeys = new NativeList<StableEntityPairKey>(Allocator.Persistent),
            SoftAvoidancePairKeys = new NativeList<StableEntityPairKey>(Allocator.Persistent),
            DormantContactSchedule = new NativeList<PredictiveContactScheduleEntry>(Allocator.Persistent),
            CacheState = new NativeReference<IncrementalContactCacheState>(Allocator.Persistent),
            IncidentPairLookup = new NativeParallelMultiHashMap<Entity, int>(1, Allocator.Persistent),
            IncidentLookupEpoch = new NativeReference<uint>(Allocator.Persistent),
            SpatialMembership = new NativeParallelMultiHashMap<int, int>(1, Allocator.Persistent),
            SpatialMembershipEpoch = new NativeReference<uint>(Allocator.Persistent)
        };
    }

    public bool RequiresCapacity(int unitCount)
    {
        int incidentRequired = math.max(1, unitCount * 64);
        int spatialRequired = math.max(1, unitCount * 128);
        return ProxyIndexByBody.Capacity < unitCount ||
               IncidentPairLookup.Capacity < incidentRequired ||
               SpatialMembership.Capacity < spatialRequired ||
               PredictiveContactIndex.Capacity < incidentRequired;
    }

    public void EnsureCapacity(int unitCount)
    {
        if (ProxyIndexByBody.Capacity < unitCount)
            ProxyIndexByBody.Capacity = unitCount;
        int incidentRequired = math.max(1, unitCount * 64);
        int spatialRequired = math.max(1, unitCount * 128);
        if (IncidentPairLookup.Capacity < incidentRequired)
            IncidentPairLookup.Capacity = incidentRequired;
        if (SpatialMembership.Capacity < spatialRequired)
            SpatialMembership.Capacity = spatialRequired;
        // 预分配哈希表避免首批帧反复 rehash；上界与 IncidentPairLookup 对齐
        if (PredictiveContactIndex.Capacity < incidentRequired)
            PredictiveContactIndex.Capacity = incidentRequired;
    }

    public void Reset()
    {
        SweptProxies.Clear();
        ProxyIndexByBody.Clear();
        NeighborPairs.Clear();
        PredictiveContacts.Clear();
        PredictiveContactIndex.Clear();
        ActiveContactKeys.Clear();
        SoftAvoidancePairKeys.Clear();
        DormantContactSchedule.Clear();
        CacheState.Value = default;
        IncidentPairLookup.Clear();
        IncidentLookupEpoch.Value = 0;
        SpatialMembership.Clear();
        SpatialMembershipEpoch.Value = 0;
    }

    public ContactPipelineLifecycleJob CreateLifecycleJob(
        ContactPipelineConfiguration configuration,
        ContactPipelineExecutionResources execution,
        ConstraintSolverFrameResources solver,
        ContactDiagnosticsFrameResources diagnostics,
        NativeList<SimulationDebuggerPairSample> debuggerSelectedPairs)
    {
        return new ContactPipelineLifecycleJob
        {
            Configuration = configuration,
            RuntimeState = execution.PipelineRuntimeState,
            PersistentSweptProxies = SweptProxies,
            PersistentProxyIndexByBody = ProxyIndexByBody,
            PersistentNeighborPairs = NeighborPairs,
            PersistentPredictiveContacts = PredictiveContacts,
            PersistentSpatialMembership = SpatialMembership,
            PersistentSpatialMembershipEpoch = SpatialMembershipEpoch,
            PersistentIncidentPairLookup = IncidentPairLookup,
            PersistentIncidentLookupEpoch = IncidentLookupEpoch,
            IncrementalCacheState = CacheState,
            ActiveIncidentIndexState = solver.ActiveIncidentIndexState,
#if RTS_CONTACT_DIAGNOSTICS
            IncrementalStatistics = diagnostics.IncrementalStatistics,
            Statistics = diagnostics.ContactStatistics,
            IterationDiagnostics = diagnostics.Iterations,
            PairDiagnostics = diagnostics.Pairs,
            SelectedBodyDiagnostic = diagnostics.SelectedBody,
            SimulationDebuggerSelectedPairs = debuggerSelectedPairs,
#endif
        };
    }

    public void Dispose()
    {
        if (SweptProxies.IsCreated) SweptProxies.Dispose();
        if (ProxyIndexByBody.IsCreated) ProxyIndexByBody.Dispose();
        if (NeighborPairs.IsCreated) NeighborPairs.Dispose();
        if (PredictiveContacts.IsCreated) PredictiveContacts.Dispose();
        if (PredictiveContactIndex.IsCreated) PredictiveContactIndex.Dispose();
        if (ActiveContactKeys.IsCreated) ActiveContactKeys.Dispose();
        if (SoftAvoidancePairKeys.IsCreated) SoftAvoidancePairKeys.Dispose();
        if (DormantContactSchedule.IsCreated) DormantContactSchedule.Dispose();
        if (CacheState.IsCreated) CacheState.Dispose();
        if (IncidentPairLookup.IsCreated) IncidentPairLookup.Dispose();
        if (IncidentLookupEpoch.IsCreated) IncidentLookupEpoch.Dispose();
        if (SpatialMembership.IsCreated) SpatialMembership.Dispose();
        if (SpatialMembershipEpoch.IsCreated) SpatialMembershipEpoch.Dispose();
    }
}
}
