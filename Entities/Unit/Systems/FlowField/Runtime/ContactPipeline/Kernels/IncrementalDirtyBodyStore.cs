using Unity.Collections;
using Unity.Entities;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 增量路径的紧凑型 dirty-body 追踪。两路并行存储：按 body 索引的 byte 标记数组，
/// 以及仅追加的 dirty body 列表。标记数组权威决定 body X 是否 dirty 以及何种 dirty；列表驱动紧凑型修复迭代。
/// 纯值函数作用于这两个存储以及 body 索引查找。
/// </summary>
internal static class IncrementalDirtyBodyStore
{
    /// <summary>
    /// 读取 <paramref name="bodyIndex"/> 的 dirty 标记。超出范围的 body 视作 entity-set dirty（未知 body 触发全拓扑刷新的安全默认）。
    /// </summary>
    internal static IncrementalBodyDirtyFlags GetFlags(
        NativeArray<byte> dirtyFlagsByBody,
        int bodyIndex) =>
        (uint)bodyIndex < (uint)dirtyFlagsByBody.Length
            ? (IncrementalBodyDirtyFlags)dirtyFlagsByBody[bodyIndex]
            : IncrementalBodyDirtyFlags.EntitySet;

    /// <summary>
    /// 将 <paramref name="flags"/> 合并进 body 当前标记；当 body 此前为 clean 时，将其追加到 dirty-body 列表。
    /// body 索引越界时为 no-op。
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
    /// 借助 dirty-body 列表避免 O(N) 扫描，将每个 body 的 dirty 标记清零并清空列表。
    /// 仅清理此前 dirty 的 body，标记数组保持稀疏清零状态。
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
    /// <paramref name="entity"/> 对应的 body 是否带 topology-dirty 标记。当前 body 索引映射中缺失的 body 一律视为 topology-dirty（强制刷新）。
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
    /// <paramref name="bodyIndex"/> 是否携带任何 dirty 标记。越界 body 一律视为 dirty（落入 GetFlags 的 entity-set 默认）。
    /// </summary>
    internal static bool IsDirtyBodyIndex(
        NativeArray<byte> dirtyFlagsByBody,
        int bodyIndex) =>
        GetFlags(dirtyFlagsByBody, bodyIndex) != IncrementalBodyDirtyFlags.None;

    /// <summary>
    /// <paramref name="entity"/> 对应的 body 是否带有任一 dirty 标记。当前 body 索引映射中缺失的 body 一律视为 dirty。
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
