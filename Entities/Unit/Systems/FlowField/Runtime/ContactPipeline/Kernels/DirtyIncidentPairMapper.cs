using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Dirty-body-driven mapping from the persistent incident-pair index to the
/// current body-indexed constraint list. For each dirty body, looks up its
/// persistent incident neighbours via the multi-hashmap and emits a
/// deterministically-ordered (lower body index first) constraint per retained
/// pair. The output is sorted and deduplicated in place.
///
/// Pure value function: all stores arrive as parameters. Returns false (with a
/// cleared pair list) when the incident lookup is missing or stale relative to
/// the committed cache epoch, or when a mapped pair references a body no longer
/// present — the caller must fall back to a full sweep.
/// </summary>
internal static class DirtyIncidentPairMapper
{
    internal static bool TryMap(
        NativeList<ContactConstraint> pairs,
        NativeList<IncrementalDirtyBody> dirtyBodies,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeParallelHashMap<Entity, int> bodyIndexByEntity,
        NativeParallelMultiHashMap<Entity, int> incidentPairLookup,
        NativeReference<uint> incidentLookupEpoch,
        NativeList<PersistentNeighborPair> persistentNeighborPairs,
        IncrementalContactCacheState cacheState)
    {
        pairs.Clear();

        if (!incidentPairLookup.IsCreated ||
            !incidentLookupEpoch.IsCreated ||
            incidentLookupEpoch.Value != cacheState.TopologyEpoch)
            return false;

        for (int dirtyIndex = 0; dirtyIndex < dirtyBodies.Length; dirtyIndex++)
        {
            int dirtyBodyIndex = dirtyBodies[dirtyIndex].BodyIndex;
            if ((uint)dirtyBodyIndex >= (uint)bodies.Length)
                return false;
            Entity entity = bodies[dirtyBodyIndex].Entity;
            NativeParallelMultiHashMapIterator<Entity> iterator;
            if (!incidentPairLookup.TryGetFirstValue(
                    entity, out int persistentPairIndex, out iterator))
                continue;
            do
            {
                if ((uint)persistentPairIndex >= (uint)persistentNeighborPairs.Length)
                    return false;
                StableEntityPairKey key =
                    persistentNeighborPairs[persistentPairIndex].Key;
                if (!TryGetBodyIndex(bodyIndexByEntity, bodies.Length, key.EntityA, out int bodyA) ||
                    !TryGetBodyIndex(bodyIndexByEntity, bodies.Length, key.EntityB, out int bodyB))
                    return false;
                pairs.Add(new ContactConstraint
                {
                    BodyA = math.min(bodyA, bodyB),
                    BodyB = math.max(bodyA, bodyB)
                });
            }
            while (incidentPairLookup.TryGetNextValue(
                out persistentPairIndex, ref iterator));
        }

        ContactPipelineShared.SortAndDeduplicateConstraints(pairs);
        return true;
    }

    private static bool TryGetBodyIndex(
        NativeParallelHashMap<Entity, int> bodyIndexByEntity,
        int bodyCount,
        Entity entity,
        out int bodyIndex) =>
        bodyIndexByEntity.TryGetValue(entity, out bodyIndex) &&
        bodyIndex >= 0 && bodyIndex < bodyCount;
}
}
