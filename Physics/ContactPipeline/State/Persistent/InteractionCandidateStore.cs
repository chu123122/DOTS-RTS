using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>
/// World 生命周期的跨帧缓存。缓存只是 BroadPhase/NarrowPhase 的优化实现，
/// 只有能力受限的资源切片会进入具体 Job；Solver 永远不能取得整个缓存。
/// </summary>
internal struct CrossFrameCache
{
    private NativeList<PersistentSweptProxy> SweptProxies;
    private NativeList<int> ProxyIndexByBody;
    private NativeList<PersistentNeighborPair> NeighborPairs;
    private NativeList<PersistentPredictiveContact> PredictiveContacts;
    // 派生 key -> list index。PredictiveContacts 是唯一权威值存储；
    // 任何列表替换/压缩都必须在发布前重建此索引。
    private NativeParallelHashMap<StableEntityPairKey, int> PredictiveContactIndex;
    private NativeReference<IncrementalContactCacheState> CacheState;
    private NativeParallelMultiHashMap<Entity, int> IncidentPairLookup;
    private NativeReference<uint> IncidentLookupEpoch;
    private NativeParallelMultiHashMap<int, int> SpatialMembership;
    private NativeReference<uint> SpatialMembershipEpoch;

    public static CrossFrameCache Create()
    {
        return new CrossFrameCache
        {
            SweptProxies = new NativeList<PersistentSweptProxy>(Allocator.Persistent),
            ProxyIndexByBody = new NativeList<int>(Allocator.Persistent),
            NeighborPairs = new NativeList<PersistentNeighborPair>(Allocator.Persistent),
            PredictiveContacts = new NativeList<PersistentPredictiveContact>(Allocator.Persistent),
            PredictiveContactIndex = new NativeParallelHashMap<StableEntityPairKey, int>(1, Allocator.Persistent),
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
            PersistentContactIndex = PredictiveContactIndex,
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

    internal NativeList<PersistentSweptProxy> PersistentSweptProxies =>
        SweptProxies;
    internal NativeList<int> PersistentProxyIndexByBody =>
        ProxyIndexByBody;
    internal NativeList<PersistentNeighborPair> PersistentNeighborPairs =>
        NeighborPairs;
    internal NativeList<PersistentPredictiveContact>
        PersistentPredictiveContacts => PredictiveContacts;
    internal NativeParallelHashMap<StableEntityPairKey, int>
        PersistentContactIndex => PredictiveContactIndex;
    internal NativeReference<IncrementalContactCacheState>
        IncrementalCacheState => CacheState;
    internal NativeParallelMultiHashMap<Entity, int>
        PersistentIncidentPairLookup => IncidentPairLookup;
    internal NativeReference<uint> PersistentIncidentLookupEpoch =>
        IncidentLookupEpoch;
    internal NativeParallelMultiHashMap<int, int>
        PersistentSpatialMembership => SpatialMembership;
    internal NativeReference<uint> PersistentSpatialMembershipEpoch =>
        SpatialMembershipEpoch;

    internal int DebugSweptProxyCount => SweptProxies.Length;

    internal PersistentSweptProxy ReadDebugSweptProxy(int index) =>
        SweptProxies[index];

    public void Dispose()
    {
        if (SweptProxies.IsCreated) SweptProxies.Dispose();
        if (ProxyIndexByBody.IsCreated) ProxyIndexByBody.Dispose();
        if (NeighborPairs.IsCreated) NeighborPairs.Dispose();
        if (PredictiveContacts.IsCreated) PredictiveContacts.Dispose();
        if (PredictiveContactIndex.IsCreated) PredictiveContactIndex.Dispose();
        if (CacheState.IsCreated) CacheState.Dispose();
        if (IncidentPairLookup.IsCreated) IncidentPairLookup.Dispose();
        if (IncidentLookupEpoch.IsCreated) IncidentLookupEpoch.Dispose();
        if (SpatialMembership.IsCreated) SpatialMembership.Dispose();
        if (SpatialMembershipEpoch.IsCreated) SpatialMembershipEpoch.Dispose();
    }
}
}
