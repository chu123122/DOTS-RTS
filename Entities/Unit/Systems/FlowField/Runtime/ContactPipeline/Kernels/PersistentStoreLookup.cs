using Unity.Collections;
using Unity.Entities;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Binary-search lookups over the sorted persistent stores (proxies by entity,
/// predictive contacts by stable pair key). The stores are kept sorted by their
/// respective comparers, so these are O(log N). Pure value functions: no job
/// state, all inputs arrive as parameters.
/// </summary>
internal static class PersistentStoreLookup
{
    /// <summary>
    /// Index of the predictive contact matching <paramref name="key"/> in the
    /// sorted contact list, or -1. Keys are compared with
    /// <see cref="StableEntityPairKeyComparer"/>.
    /// </summary>
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
    /// Index of the proxy carrying <paramref name="entity"/> in the sorted
    /// persistent-proxy list, or -1. Entities are compared with
    /// <see cref="StableEntityPairKey.CompareEntity"/>.
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
    /// Tries to fetch the persistent proxy for <paramref name="entity"/> from
    /// the sorted list. Returns false (with a default proxy) when absent.
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
    /// Tries to fetch the current incremental proxy for <paramref name="entity"/>
    /// from the sorted incremental-proxy list. Returns false (with a default
    /// proxy) when absent.
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
