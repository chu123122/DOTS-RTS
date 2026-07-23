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
        int required = math.max(
            1,
            math.max(unitCount * 64, _persistentNeighborPairs.Length * 2 + 1));
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
