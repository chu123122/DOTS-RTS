using Unity.Collections;
using Unity.Entities;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 对已排序持久存储（按实体的 proxy、按稳定 pair key 的预测接触）的二分查找。
/// 存储由各自比较器保持有序，复杂度 O(log N)。纯值函数：无 Job 状态，所有输入作为参数传入。
/// </summary>
internal static class PersistentStoreLookup
{
    /// <summary>
    /// 在已排序的接触列表中，与 <paramref name="key"/> 匹配的预测接触下标；未匹配则返回 -1。
    /// 键的比较使用 <see cref="StableEntityPairKeyComparer"/>。
    /// </summary>
    /// <summary>
    /// O(1) 哈希表查找：在 <paramref name="contactIndex"/> 中取 <paramref name="key"/> 对应的持久接触。
    /// 未找到时返回 false，contact 为默认值。
    /// </summary>
    internal static bool TryGetPredictiveContact(
        NativeHashMap<StableEntityPairKey, PersistentPredictiveContact> contactIndex,
        StableEntityPairKey key,
        out PersistentPredictiveContact contact) =>
        contactIndex.TryGetValue(key, out contact);

    internal static int FindPredictiveContactIndex(
        NativeList<PersistentPredictiveContact> contacts,
        StableEntityPairKey key)
    {
        int low = 0;
        int high = contacts.Length - 1;
        var comparer = new StableEntityPairKeyComparer();
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            int comparison = comparer.Compare(contacts[middle].Key, key);
            if (comparison == 0)
                return middle;
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        return -1;
    }

    /// <summary>
    /// 在已排序的持久 proxy 列表中承载 <paramref name="entity"/> 的 proxy 下标；未匹配则返回 -1。
    /// 实体的比较使用 <see cref="StableEntityPairKey.CompareEntity"/>。
    /// </summary>
    internal static int FindProxyIndex(
        NativeList<PersistentSweptProxy> proxies,
        Entity entity)
    {
        int low = 0;
        int high = proxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            int comparison = StableEntityPairKey.CompareEntity(
                proxies[middle].Entity, entity);
            if (comparison == 0)
                return middle;
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }
        return -1;
    }

    /// <summary>
    /// 尝试在已排序列表中获取 <paramref name="entity"/> 的持久 proxy。缺失时返回 false（proxy 为默认值）。
    /// </summary>
    internal static bool TryFindPersistentProxy(
        NativeList<PersistentSweptProxy> proxies,
        Entity entity,
        out PersistentSweptProxy proxy)
    {
        int low = 0;
        int high = proxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            PersistentSweptProxy candidate = proxies[middle];
            int comparison = StableEntityPairKey.CompareEntity(candidate.Entity, entity);
            if (comparison == 0)
            {
                proxy = candidate;
                return true;
            }
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }

        proxy = default;
        return false;
    }

    /// <summary>
    /// 尝试在已排序的增量 proxy 列表中获取 <paramref name="entity"/> 的当前 proxy。缺失时返回 false（proxy 为默认值）。
    /// </summary>
    internal static bool TryFindIncrementalProxy(
        NativeList<PersistentSweptProxy> incrementalProxies,
        Entity entity,
        out PersistentSweptProxy proxy)
    {
        int low = 0;
        int high = incrementalProxies.Length - 1;
        while (low <= high)
        {
            int middle = (low + high) >> 1;
            PersistentSweptProxy candidate = incrementalProxies[middle];
            int comparison = StableEntityPairKey.CompareEntity(candidate.Entity, entity);
            if (comparison == 0)
            {
                proxy = candidate;
                return true;
            }
            if (comparison < 0)
                low = middle + 1;
            else
                high = middle - 1;
        }

        proxy = default;
        return false;
    }
}
}
