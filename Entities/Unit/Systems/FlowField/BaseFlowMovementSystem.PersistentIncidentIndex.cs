using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Systems
{
public abstract partial class BaseFlowMovementSystem
{
    // Cross-frame views are system-owned so their lifetime follows the scheduled
    // solver dependency instead of being recreated inside each contact job.
    private NativeParallelMultiHashMap<Entity, int> _persistentIncidentPairLookup;
    private NativeReference<uint> _persistentIncidentLookupEpoch;
    private NativeParallelMultiHashMap<int, int> _persistentSpatialMembership;
    private NativeReference<uint> _persistentSpatialMembershipEpoch;

    private void CreatePersistentIncidentLookup()
    {
        _persistentIncidentPairLookup =
            new NativeParallelMultiHashMap<Entity, int>(1, Allocator.Persistent);
        _persistentIncidentLookupEpoch =
            new NativeReference<uint>(Allocator.Persistent);
        _persistentSpatialMembership =
            new NativeParallelMultiHashMap<int, int>(1, Allocator.Persistent);
        _persistentSpatialMembershipEpoch =
            new NativeReference<uint>(Allocator.Persistent);
    }

    private void EnsurePersistentIncidentLookupCapacity(int unitCount)
    {
        // Do not inspect persistent-list lengths here: the previous frame may still
        // be writing them. Capacity is derived only from the stable main-thread body
        // count, and growth is the only synchronization point.
        int incidentRequired = math.max(1, unitCount * 64);
        int spatialRequired = math.max(1, unitCount * 128);
        if (_persistentIncidentPairLookup.Capacity >= incidentRequired &&
            _persistentSpatialMembership.Capacity >= spatialRequired)
            return;

        Dependency.Complete();
        if (_persistentIncidentPairLookup.Capacity < incidentRequired)
            _persistentIncidentPairLookup.Capacity = incidentRequired;
        if (_persistentSpatialMembership.Capacity < spatialRequired)
            _persistentSpatialMembership.Capacity = spatialRequired;
    }

    private void DisposePersistentIncidentLookup()
    {
        if (_persistentIncidentPairLookup.IsCreated)
            _persistentIncidentPairLookup.Dispose();
        if (_persistentIncidentLookupEpoch.IsCreated)
            _persistentIncidentLookupEpoch.Dispose();
        if (_persistentSpatialMembership.IsCreated)
            _persistentSpatialMembership.Dispose();
        if (_persistentSpatialMembershipEpoch.IsCreated)
            _persistentSpatialMembershipEpoch.Dispose();
    }
}
}
