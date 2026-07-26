using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Timestep-level predictive-contact scheduling for the persistent (P1P6) path.
/// Each sweep-disc pair is classified into a lifecycle (actual / predictive /
/// approaching / dormant) from its current trajectory state; dormant pairs are
/// given a wake-up substep derived from their closest approach time. The
/// classified contacts are committed to the persistent scratch store and the
/// dormant ones are seeded into the per-timestep schedule. Active (non-dormant)
/// pairs are compacted in place so the XPBD view carries no dormant entries.
///
/// Pure value function: all stores arrive as parameters. Body data is read from
/// the parallel Body/Navigation/Intent/MotionEvidence/StepState arrays indexed
/// by the constraint's body slots.
/// </summary>
internal static class PredictiveContactScheduler
{
    /// <summary>
    /// Builds the timestep predictive-contact view from the raw pair list.
    /// Outputs:
    ///  - <paramref name="predictiveContactScratch"/>: every pair's classified
    ///    contact (sorted by stable key);
    ///  - <paramref name="persistentPredictiveContacts"/>: mirror of scratch
    ///    when the persistent cache is enabled (cleared + refilled);
    ///  - <paramref name="predictiveContactSchedule"/>: dormant wake-up entries
    ///    (sorted by substep);
    ///  - <paramref name="pairs"/>: compacted in place to drop dormant entries.
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
            // 不读取上一帧接触状态，保证 A0B1 与 A1B1 只有来源成本不同。
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
                    // Wake one substep early. The retained contact margin is the safety
                    // budget for solver/RVO deviations; any larger deviation triggers
                    // the envelope-escape repair path.
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

        // Dormant contacts live in B's timestep schedule, not in the active
        // XPBD view, so A0/A1 share the same constraint-utilization semantics.
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
