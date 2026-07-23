using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Systems
{
public abstract partial class BaseFlowMovementSystem
{
    private NativeParallelMultiHashMap<Entity, int> _persistentIncidentPairLookup;
    private NativeReference<uint> _persistentIncidentLookupEpoch;

    private void CreatePersistentIncidentLookup()
    {
        _persistentIncidentPairLookup =
            new NativeParallelMultiHashMap<Entity, int>(1, Allocator.Persistent);
        _persistentIncidentLookupEpoch =
            new NativeReference<uint>(Allocator.Persistent);
    }

    private void EnsurePersistentIncidentLookupCapacity(int unitCount)
    {
        // Do not inspect PersistentNeighborPairs.Length here: the previous frame may
        // still be writing that persistent list. Capacity is derived only from the
        // stable main-thread body count, and growth is the only synchronization point.
        int required = math.max(1, unitCount * 64);
        if (_persistentIncidentPairLookup.Capacity >= required)
            return;
        Dependency.Complete();
        _persistentIncidentPairLookup.Capacity = required;
    }

    private void DisposePersistentIncidentLookup()
    {
        if (_persistentIncidentPairLookup.IsCreated)
            _persistentIncidentPairLookup.Dispose();
        if (_persistentIncidentLookupEpoch.IsCreated)
            _persistentIncidentLookupEpoch.Dispose();
    }
}
}
