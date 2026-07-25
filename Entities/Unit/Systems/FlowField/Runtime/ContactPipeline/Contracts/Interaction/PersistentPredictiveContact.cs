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
    Separating,
    Expired
}

/// <summary>
/// Cross-timestep predictive state. XPBD lambda is deliberately not persisted.
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
