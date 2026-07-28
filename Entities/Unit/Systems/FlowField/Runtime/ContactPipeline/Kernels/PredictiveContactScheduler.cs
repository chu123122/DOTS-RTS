using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 持久（P1P6）路径在 timestep 级的预测接触调度。每个 swept-disc 对根据当前轨迹状态分类为生命周期
///（actual / predictive / approaching / dormant）；dormant 对被赋予一个由最近接近时刻推导的唤醒子步。
/// 已分类接触提交至持久 scratch；dormant 对种入按 timestep 的调度。活跃（非 dormant）对就地压实，
/// 使 XPBD 视图不含 dormant 项。
///
/// 纯值函数：所有存储以参数传入。body 数据按约束 body 槽位索引，从并行的 Body/Navigation/Intent/MotionEvidence/StepState 数组读取。
/// </summary>
internal static class PredictiveContactScheduler
{
    /// <summary>
    /// 由原始对列表构造 timestep 预测接触视图。输出：
    ///  - <paramref name="predictiveContactScratch"/>：每对的分类接触（按稳定 key 排序）；
    ///  - <paramref name="persistentPredictiveContacts"/>：启用持久缓存时 scratch 的镜像（清空后回填）；
    ///  - <paramref name="predictiveContactSchedule"/>：dormant 唤醒项（按子步排序）；
    ///  - <paramref name="pairs"/>：就地压实，去掉 dormant 项。
    /// </summary>
    internal static void BuildTimestepSchedule(
        NativeList<ContactConstraint> pairs,
        NativeArray<CrowdBodySnapshot> bodies,
        NativeArray<CrowdMotionEvidence> motionEvidence,
        NativeList<PersistentSweptProxy> persistentSweptProxies,
        NativeList<PersistentPredictiveContact> predictiveContactScratch,
        NativeList<PersistentPredictiveContact> persistentPredictiveContacts,
        NativeList<PredictiveContactScheduleEntry> predictiveContactSchedule,
        NativeReference<int> predictiveContactScheduleCursor,
        uint timestep,
        int substepCount,
        int scheduleStartSubstep,
        bool enableTimestepContactSetCache,
        bool enablePersistentContactCache,
        bool enablePredictiveContacts,
        ref IncrementalContactPipelineStatistics incrementalStatistics)
    {
        int totalSubstepCount = enableTimestepContactSetCache
            ? math.max(1, substepCount)
            : 1;
        scheduleStartSubstep = math.clamp(
            scheduleStartSubstep,
            0,
            totalSubstepCount - 1);
        int remainingSubstepCount = totalSubstepCount - scheduleStartSubstep;

        for (int pairIndex = 0; pairIndex < pairs.Length; pairIndex++)
        {
            ContactConstraint pair = pairs[pairIndex];
            CrowdBodySnapshot bodyASnapshot = bodies[pair.BodyA];
            CrowdMotionEvidence bodyAEvidence = motionEvidence[pair.BodyA];
            CrowdBodySnapshot bodyBSnapshot = bodies[pair.BodyB];
            CrowdMotionEvidence bodyBEvidence = motionEvidence[pair.BodyB];
            StableEntityPairKey key = StableEntityPairKey.Create(
                bodyASnapshot.Entity, bodyBSnapshot.Entity);

            PersistentContactLifecycle lifecycle;
            float3 currentDelta = bodyAEvidence.TrajectoryStart - bodyBEvidence.TrajectoryStart;
            currentDelta.y = 0f;
            float radiusSum = bodyASnapshot.Radius + bodyBSnapshot.Radius;
            if (math.lengthsq(currentDelta) <= radiusSum * radiusSum)
                lifecycle = PersistentContactLifecycle.Actual;
            else if (pair.IsDormant != 0)
                lifecycle = PersistentContactLifecycle.Dormant;
            else if (pair.ContactMode == ContactConstraintMode.Predictive)
                lifecycle = PersistentContactLifecycle.Predictive;
            else
                lifecycle = PersistentContactLifecycle.Approaching;

            // 调度与稳定法线属于中层 InteractionSet 的派生结果。
            // 不读上一帧接触状态，让 A0B1 与 A1B1 只在来源成本上有差异。
            float3 stableNormal = pair.PredictiveNormal;
            sbyte fixedSide = pair.ContactMode == ContactConstraintMode.Predictive
                ? (sbyte)1
                : (sbyte)0;

            PersistentSweptProxy proxyA = default;
            PersistentSweptProxy proxyB = default;
            if (enablePersistentContactCache)
            {
                PersistentStoreLookup.TryFindPersistentProxy(
                    persistentSweptProxies, bodyASnapshot.Entity, out proxyA);
                PersistentStoreLookup.TryFindPersistentProxy(
                    persistentSweptProxies, bodyBSnapshot.Entity, out proxyB);
            }

            ushort firstPossibleSubstep = 0;
            if (lifecycle == PersistentContactLifecycle.Dormant)
            {
                if (!PersistentContactMath.HasRelativeTimestepTrajectory(
                        bodyAEvidence, bodyBEvidence))
                {
                    firstPossibleSubstep = ushort.MaxValue;
                }
                else
                {
                    float closestTime = PersistentContactMath.CalculatePairClosestTime(
                        bodyAEvidence, bodyBEvidence);
                    int closestSubstepOffset = math.clamp(
                        (int)math.floor(closestTime * remainingSubstepCount),
                        0,
                        remainingSubstepCount - 1);
                    // 比最近子步早一格唤醒。留存接触余量是求解器/RVO 偏差的安全预算；更大偏差会触发包络逃逸修复。
                    firstPossibleSubstep = (ushort)(scheduleStartSubstep +
                        math.max(0, closestSubstepOffset - 1));
                }
            }

            PersistentPredictiveContact contact = new PersistentPredictiveContact
            {
                Key = key,
                StableNormal = stableNormal,
                Lifecycle = lifecycle,
                FixedSide = fixedSide,
                FirstPossibleSubstep = firstPossibleSubstep,
                NextCheckSubstep = firstPossibleSubstep,
                LastSeenTimestep = timestep,
                MotionVersionA = proxyA.MotionVersion,
                MotionVersionB = proxyB.MotionVersion
            };
            predictiveContactScratch.Add(contact);

            if (lifecycle == PersistentContactLifecycle.Dormant)
            {
                predictiveContactSchedule.Add(new PredictiveContactScheduleEntry
                {
                    Key = key,
                    Substep = firstPossibleSubstep
                });
            }
        }

        if (predictiveContactScratch.Length > 1)
            predictiveContactScratch.AsArray().Sort(new PersistentPredictiveContactComparer());
        if (predictiveContactSchedule.Length > 1)
            predictiveContactSchedule.AsArray().Sort(new PredictiveContactScheduleEntryComparer());
        predictiveContactScheduleCursor.Value = 0;
        persistentPredictiveContacts.Clear();
        if (enablePersistentContactCache)
            persistentPredictiveContacts.AddRange(predictiveContactScratch.AsArray());

        // dormant 接触放在 B 的 timestep 调度里，不进活跃 XPBD 视图，让 A0/A1 共享约束利用语义。
        int activeWriteIndex = 0;
        for (int pairIndex = 0; pairIndex < pairs.Length; pairIndex++)
        {
            ContactConstraint pair = pairs[pairIndex];
            if (pair.IsDormant != 0)
                continue;
            pairs[activeWriteIndex++] = pair;
        }
        pairs.ResizeUninitialized(activeWriteIndex);
        PersistentContactMath.RefreshCurrentContactStateGauges(
            predictiveContactScratch,
            ref incrementalStatistics,
            activeWriteIndex);
    }
}
}
