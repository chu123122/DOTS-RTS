using System;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
public enum PersistentContactLifecycle : byte
{
    Dormant,
    Approaching,
    Predictive,
    Actual,
    // 注：Separating 当前不被任何分类器（ClassifyPersistentNeighborPair /
    // ClassifyPersistentPairP1P6 / UpdatePersistentContactAfterScheduledCheck）
    // 输出。保留枚举值是因为 eligibility filter 以 Lifecycle != Expired 判定
    // eligible，Separating 落入 eligible（保守正确）。未来若实现"分离中"语义
    // 再由分类器产生此状态。
    Separating,
    Expired
}

/// <summary>
/// 跨 timestep 的预测状态。刻意不持久化 XPBD lambda。
/// </summary>
public struct PersistentPredictiveContact
{
    public StableEntityPairKey Key;
    public float3 StableNormal;
    public PersistentContactLifecycle Lifecycle;
    public ContactConstraintMode ContactMode;
    public sbyte FixedSide;
    public byte SoftAvoidanceCandidate;
    public ushort FirstPossibleSubstep;
    public ushort NextCheckSubstep;
    public float ClosestTime;
    public uint LastSeenTimestep;
    public uint MotionVersionA;
    public uint MotionVersionB;
    public uint ClassificationEpoch;
}

public struct PersistentPredictiveContactComparer : IComparer<PersistentPredictiveContact>
{
    public int Compare(PersistentPredictiveContact left, PersistentPredictiveContact right)
    {
        return new StableEntityPairKeyComparer().Compare(left.Key, right.Key);
    }
}

public struct PredictiveContactScheduleEntry
{
    public StableEntityPairKey Key;
    public ushort Substep;
}

public struct PredictiveContactScheduleEntryComparer : IComparer<PredictiveContactScheduleEntry>
{
    public int Compare(PredictiveContactScheduleEntry left, PredictiveContactScheduleEntry right)
    {
        int substepComparison = left.Substep.CompareTo(right.Substep);
        return substepComparison != 0
            ? substepComparison
            : new StableEntityPairKeyComparer().Compare(left.Key, right.Key);
    }
}
}
