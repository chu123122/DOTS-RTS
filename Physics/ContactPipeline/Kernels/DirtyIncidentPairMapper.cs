using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 由 dirty body 驱动的持久事件对索引到当前按 body 索引约束列表的映射。对每个 dirty body，
/// 通过 multi-hashmap 查找其持久事件邻居，并按（较小 body 索引在前）输出确定性顺序约束。
/// 结果就地排序并去重。
///
/// Pair 级 eligibility filter：一个 dirty 事件对只有在本时间步可能产生接触时才输出（从而
/// 才会被重新分类）——它上次分类时非 Expired（Actual/Approaching/Predictive/Dormant/
/// Separating），或两个端点的 tight 扫掠 AABB 本帧重叠（Expired 对重新进入接触范围）。
/// 远处的 Expired 对被跳过，这正是意义所在：持久邻居池是 FatGuard 扩大的集合，其中大部分
/// 对是远处 Dormant/Expired，否则每个 dirty 帧都要被完整重新分类。这从不丢弃真实接触：
/// 仍在重叠的 Actual 非 Expired 故 eligible；重新接近的对因重叠故 eligible；只有既 Expired
/// 又不再重叠的对才被跳过。
///
/// 当 dirty 集很大（事件访问超过持久池一半）时，邻居索引枚举退化为几乎访问全部对还要付
/// 排序/去重。此时切到对持久邻居对的单趟线性扫描，每对只做一次 dirty 端点 + eligibility
/// 测试，无去重。
///
/// 纯值函数：所有存储以参数传入。当事件查找缺失/相对已提交缓存 epoch 过期，
/// 或映射到的对包含已不存在的 body 时返回 false（对列表清空），调用方须回退到全量扫描。
/// </summary>
internal static class DirtyIncidentPairMapper
{
    private const float LinearScanIncidentVisitRatio = 0.5f;

    internal static bool TryMap(
        NativeList<ContactConstraint> pairs,
        NativeList<IncrementalDirtyBody> dirtyBodies,
        NativeArray<byte> dirtyFlagsByBody,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeParallelHashMap<Entity, int> bodyIndexByEntity,
        NativeParallelMultiHashMap<Entity, int> incidentPairLookup,
        NativeReference<uint> incidentLookupEpoch,
        NativeList<PersistentNeighborPair> persistentNeighborPairs,
        NativeList<PersistentPredictiveContact> contacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex,
        NativeList<PersistentSweptProxy> persistentProxies,
        NativeList<int> proxyIndexByBody,
        IncrementalContactCacheState cacheState,
        out int dirtyIncidentPairCount,
        out int eligibilitySkippedCount)
    {
        pairs.Clear();
        dirtyIncidentPairCount = 0;
        eligibilitySkippedCount = 0;

        if (!incidentPairLookup.IsCreated ||
            !incidentLookupEpoch.IsCreated ||
            incidentLookupEpoch.Value != cacheState.TopologyEpoch)
            return false;

        // 探测邻居索引：若 dirty 体的邻接访问会超过持久池的一半，索引不再值得付出
        // 去重成本——转入下方的单趟线性扫描。
        int linearScanThreshold = math.max(
            1,
            (int)math.ceil(persistentNeighborPairs.Length * LinearScanIncidentVisitRatio));
        int incidentVisitCount = 0;
        bool useLinearScan = false;
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
                if (++incidentVisitCount > linearScanThreshold)
                {
                    useLinearScan = true;
                    break;
                }
                if ((uint)persistentPairIndex >= (uint)persistentNeighborPairs.Length)
                    return false;
            }
            while (incidentPairLookup.TryGetNextValue(
                out persistentPairIndex, ref iterator));
            if (useLinearScan)
                break;
        }

        if (useLinearScan)
        {
            return TryMapByLinearScan(
                pairs,
                dirtyFlagsByBody,
                bodies,
                bodyIndexByEntity,
                persistentNeighborPairs,
                contacts,
                contactIndex,
                persistentProxies,
                proxyIndexByBody,
                out dirtyIncidentPairCount,
                out eligibilitySkippedCount);
        }

        // 邻居索引路径：收集全部 dirty 事件对，再就地应用 eligibility filter。
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
                    Definition = new ContactConstraintDefinition
                    {
                        BodyA = math.min(bodyA, bodyB),
                        BodyB = math.max(bodyA, bodyB)
                    }
                });
            }
            while (incidentPairLookup.TryGetNextValue(
                out persistentPairIndex, ref iterator));
        }

        ContactPipelineShared.SortAndDeduplicateConstraints(pairs);
        // 邻居索引路径：这里计数的是去重后的 pair 数（线性扫描路径无去重步骤，
        // 计的是原始脏端点 pair 数；两者都语义为"送入 eligibility 过滤器的输入量"，
        // 仅当两个脏体共享邻居时会有微小差异。当前无调用方读取此 out，仅保留作诊断。）
        dirtyIncidentPairCount = pairs.Length;
        return FilterEligiblePairsInPlace(
            pairs,
            bodies,
            contacts,
            contactIndex,
            persistentProxies,
            proxyIndexByBody,
            out eligibilitySkippedCount);
    }

    private static bool TryMapByLinearScan(
        NativeList<ContactConstraint> pairs,
        NativeArray<byte> dirtyFlagsByBody,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeParallelHashMap<Entity, int> bodyIndexByEntity,
        NativeList<PersistentNeighborPair> persistentNeighborPairs,
        NativeList<PersistentPredictiveContact> contacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex,
        NativeList<PersistentSweptProxy> persistentProxies,
        NativeList<int> proxyIndexByBody,
        out int dirtyIncidentPairCount,
        out int eligibilitySkippedCount)
    {
        // 对持久池的单趟扫描：仅当至少一个端点 dirty 且通过 eligibility filter 时保留。
        // 无去重、无排序——每个持久对恰好访问一次。
        dirtyIncidentPairCount = 0;
        eligibilitySkippedCount = 0;
        for (int pairIndex = 0;
             pairIndex < persistentNeighborPairs.Length;
             pairIndex++)
        {
            StableEntityPairKey key = persistentNeighborPairs[pairIndex].Key;
            if (!TryGetBodyIndex(
                    bodyIndexByEntity, bodies.Length, key.EntityA, out int bodyA) ||
                !TryGetBodyIndex(
                    bodyIndexByEntity, bodies.Length, key.EntityB, out int bodyB))
                return false;
            if (!IncrementalDirtyBodyStore.IsDirtyBodyIndex(dirtyFlagsByBody, bodyA) &&
                !IncrementalDirtyBodyStore.IsDirtyBodyIndex(dirtyFlagsByBody, bodyB))
                continue;

            dirtyIncidentPairCount++;
                if (!TryEvaluateEligibility(
                    key, bodyA, bodyB,
                    contacts, contactIndex, persistentProxies, proxyIndexByBody,
                    out bool eligible))
                return false;
            if (!eligible)
            {
                eligibilitySkippedCount++;
                continue;
            }
            pairs.Add(new ContactConstraint
            {
                Definition = new ContactConstraintDefinition
                {
                    BodyA = math.min(bodyA, bodyB),
                    BodyB = math.max(bodyA, bodyB)
                }
            });
        }
        return true;
    }

    private static bool FilterEligiblePairsInPlace(
        NativeList<ContactConstraint> pairs,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeList<PersistentPredictiveContact> contacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex,
        NativeList<PersistentSweptProxy> persistentProxies,
        NativeList<int> proxyIndexByBody,
        out int eligibilitySkippedCount)
    {
        // 就地压缩：只保留 eligible 对。保持上方建立的有序性（从不越过被保留元素交换）。
        eligibilitySkippedCount = 0;
        int writeIndex = 0;
        for (int pairIndex = 0; pairIndex < pairs.Length; pairIndex++)
        {
            ContactConstraint pair = pairs[pairIndex];
            StableEntityPairKey key = StableEntityPairKey.Create(
                bodies[pair.BodyA].Entity,
                bodies[pair.BodyB].Entity);
            if (!TryEvaluateEligibility(
                    key, pair.BodyA, pair.BodyB,
                    contacts, contactIndex, persistentProxies, proxyIndexByBody,
                    out bool eligible))
                return false;
            if (!eligible)
            {
                eligibilitySkippedCount++;
                continue;
            }
            pairs[writeIndex++] = pair;
        }
        pairs.ResizeUninitialized(writeIndex);
        return true;
    }

    private static bool TryEvaluateEligibility(
        StableEntityPairKey key,
        int bodyA,
        int bodyB,
        NativeList<PersistentPredictiveContact> contacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex,
        NativeList<PersistentSweptProxy> persistentProxies,
        NativeList<int> proxyIndexByBody,
        out bool eligible)
    {
        eligible = IsEligible(
            key,
            bodyA,
            bodyB,
            contacts.AsArray(),
            contactIndex,
            persistentProxies.AsArray(),
            proxyIndexByBody.AsArray());
        return true;
    }

    internal static bool IsEligible(
        StableEntityPairKey key,
        int bodyA,
        int bodyB,
        NativeArray<PersistentPredictiveContact> contacts,
        NativeParallelHashMap<StableEntityPairKey, int> contactIndex,
        NativeArray<PersistentSweptProxy> persistentProxies,
        NativeArray<int> proxyIndexByBody)
    {
        // O(1) 哈希表查找取代原来的二分查找
        int contactIndexValue = -1;
        bool hasPrev = contactIndex.IsCreated &&
                       contactIndex.TryGetValue(key, out contactIndexValue) &&
                       (uint)contactIndexValue < (uint)contacts.Length;
        PersistentPredictiveContact prev = hasPrev
            ? contacts[contactIndexValue]
            : default;
        if (hasPrev && prev.Lifecycle != PersistentContactLifecycle.Expired)
            return true;
        if (!TryGetProxy(bodyA, persistentProxies, proxyIndexByBody,
                out PersistentSweptProxy proxyA) ||
            !TryGetProxy(bodyB, persistentProxies, proxyIndexByBody,
                out PersistentSweptProxy proxyB))
            return true;
        return ContactPipelineShared.AabbOverlaps(
            proxyA.TightMin, proxyA.TightMax,
            proxyB.TightMin, proxyB.TightMax);
    }

    private static bool TryGetProxy(
        int bodyIndex,
        NativeArray<PersistentSweptProxy> persistentProxies,
        NativeArray<int> proxyIndexByBody,
        out PersistentSweptProxy proxy)
    {
        proxy = default;
        if ((uint)bodyIndex >= (uint)proxyIndexByBody.Length)
            return false;
        int proxyIndex = proxyIndexByBody[bodyIndex];
        if ((uint)proxyIndex >= (uint)persistentProxies.Length)
            return false;
        proxy = persistentProxies[proxyIndex];
        return proxy.IsValid != 0;
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
