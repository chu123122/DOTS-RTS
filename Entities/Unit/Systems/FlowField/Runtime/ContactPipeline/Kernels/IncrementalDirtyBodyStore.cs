using Unity.Collections;
using Unity.Entities;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Compact dirty-body tracking for the incremental (P1P6) path. Two parallel
/// stores: a byte flag array indexed by body, and an append-only list of dirty
/// bodies. The flag array is authoritative for "is body X dirty and how"; the
/// list drives compact repair iteration. Pure value functions over those two
/// stores plus the body-index lookup.
/// </summary>
internal static class IncrementalDirtyBodyStore
{
    /// <summary>
    /// Reads the dirty flags for <paramref name="bodyIndex"/>. Out-of-range
    /// bodies are treated as entity-set dirty (the safe default that forces a
    /// full topology refresh for the unknown body).
    /// </summary>
    internal static IncrementalBodyDirtyFlags GetFlags(
        NativeArray<byte> dirtyFlagsByBody,
        int bodyIndex) =>
        (uint)bodyIndex < (uint)dirtyFlagsByBody.Length
            ? (IncrementalBodyDirtyFlags)dirtyFlagsByBody[bodyIndex]
            : IncrementalBodyDirtyFlags.EntitySet;

    /// <summary>
    /// Merges <paramref name="flags"/> into the body's current flags and, when
    /// the body was previously clean, appends it to the dirty-body list. A
    /// no-op when the body index is out of range.
    /// </summary>
    internal static void SetFlags(
        int bodyIndex,
        IncrementalBodyDirtyFlags flags,
        NativeArray<byte> dirtyFlagsByBody,
        NativeList<IncrementalDirtyBody> dirtyBodies)
    {
        if ((uint)bodyIndex >= (uint)dirtyFlagsByBody.Length)
            return;
        IncrementalBodyDirtyFlags previous =
            (IncrementalBodyDirtyFlags)dirtyFlagsByBody[bodyIndex];
        IncrementalBodyDirtyFlags merged = previous | flags;
        dirtyFlagsByBody[bodyIndex] = (byte)merged;
        if (previous == IncrementalBodyDirtyFlags.None)
        {
            dirtyBodies.Add(new IncrementalDirtyBody
            {
                BodyIndex = bodyIndex,
                Flags = merged
            });
        }
    }

    /// <summary>
    /// Clears every body's dirty flag (using the dirty-body list to avoid an
    /// O(N) sweep) and empties the list. Leaves the flag array sparse-cleared:
    /// only previously-dirty bodies are zeroed.
    /// </summary>
    internal static void Clear(
        NativeArray<byte> dirtyFlagsByBody,
        NativeList<IncrementalDirtyBody> dirtyBodies)
    {
        for (int dirtyIndex = 0; dirtyIndex < dirtyBodies.Length; dirtyIndex++)
        {
            int bodyIndex = dirtyBodies[dirtyIndex].BodyIndex;
            if ((uint)bodyIndex < (uint)dirtyFlagsByBody.Length)
                dirtyFlagsByBody[bodyIndex] = 0;
        }
        dirtyBodies.Clear();
    }

    /// <summary>
    /// Whether <paramref name="entity"/>'s body carries the topology-dirty flag.
    /// Bodies absent from the current body-index map are treated as
    /// topology-dirty (forces a refresh).
    /// </summary>
    internal static bool IsTopologyDirtyEntity(
        Entity entity,
        NativeParallelHashMap<Entity, int> bodyIndexByEntity,
        NativeArray<byte> dirtyFlagsByBody)
    {
        if (!bodyIndexByEntity.TryGetValue(entity, out int bodyIndex))
            return true;
        return (GetFlags(dirtyFlagsByBody, bodyIndex) &
                IncrementalBodyDirtyFlags.Topology) != 0;
    }

    /// <summary>
    /// Whether <paramref name="bodyIndex"/> carries any dirty flag. Out-of-range
    /// bodies are treated as dirty (they fall through GetFlags' entity-set
    /// default).
    /// </summary>
    internal static bool IsDirtyBodyIndex(
        NativeArray<byte> dirtyFlagsByBody,
        int bodyIndex) =>
        GetFlags(dirtyFlagsByBody, bodyIndex) != IncrementalBodyDirtyFlags.None;

    /// <summary>
    /// Whether <paramref name="entity"/>'s body is dirty by any flag. Bodies
    /// absent from the current body-index map are treated as dirty.
    /// </summary>
    internal static bool IsDirtyEntity(
        Entity entity,
        NativeParallelHashMap<Entity, int> bodyIndexByEntity,
        NativeArray<byte> dirtyFlagsByBody)
    {
        if (!bodyIndexByEntity.TryGetValue(entity, out int bodyIndex))
            return true;
        return IsDirtyBodyIndex(dirtyFlagsByBody, bodyIndex);
    }
}
}
