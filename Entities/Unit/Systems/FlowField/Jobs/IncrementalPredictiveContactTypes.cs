using System;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Stable pair identity used by all cross-timestep contact containers.
/// Body indices are intentionally excluded because query order is frame-local.
/// </summary>
public struct StableEntityPairKey : IEquatable<StableEntityPairKey>
{
    public Entity EntityA;
    public Entity EntityB;

    public static StableEntityPairKey Create(Entity first, Entity second)
    {
        return CompareEntity(first, second) <= 0
            ? new StableEntityPairKey { EntityA = first, EntityB = second }
            : new StableEntityPairKey { EntityA = second, EntityB = first };
    }

    public bool Contains(Entity entity)
    {
        return EntityA == entity || EntityB == entity;
    }

    public Entity Other(Entity entity)
    {
        return EntityA == entity ? EntityB : EntityA;
    }

    public bool Equals(StableEntityPairKey other)
    {
        return EntityA == other.EntityA && EntityB == other.EntityB;
    }

    public override bool Equals(object obj)
    {
        return obj is StableEntityPairKey other && Equals(other);
    }

    public override int GetHashCode()
    {
        unchecked
        {
            return (EntityA.GetHashCode() * 397) ^ EntityB.GetHashCode();
        }
    }

    public static int CompareEntity(Entity left, Entity right)
    {
        int indexComparison = left.Index.CompareTo(right.Index);
        return indexComparison != 0
            ? indexComparison
            : left.Version.CompareTo(right.Version);
    }
}

public struct StableEntityPairKeyComparer : IComparer<StableEntityPairKey>
{
    public int Compare(StableEntityPairKey left, StableEntityPairKey right)
    {
        int firstComparison = StableEntityPairKey.CompareEntity(left.EntityA, right.EntityA);
        return firstComparison != 0
            ? firstComparison
            : StableEntityPairKey.CompareEntity(left.EntityB, right.EntityB);
    }
}

/// <summary>
/// Persistent body proxy. Guard bounds prove topology completeness; tight bounds
/// describe the most recently classified prediction horizon.
/// </summary>
public struct PersistentSweptProxy
{
    public Entity Entity;
    public int BodyIndex;
    public float2 TightMin;
    public float2 TightMax;
    public float2 GuardMin;
    public float2 GuardMax;
    public uint MotionVersion;
    public byte IsValid;
}

public struct PersistentSweptProxyComparer : IComparer<PersistentSweptProxy>
{
    public int Compare(PersistentSweptProxy left, PersistentSweptProxy right)
    {
        return StableEntityPairKey.CompareEntity(left.Entity, right.Entity);
    }
}

public struct PersistentNeighborPair
{
    public StableEntityPairKey Key;
    public uint TopologyEpoch;
    public uint LastValidatedTimestep;
}

public struct PersistentNeighborPairComparer : IComparer<PersistentNeighborPair>
{
    public int Compare(PersistentNeighborPair left, PersistentNeighborPair right)
    {
        return new StableEntityPairKeyComparer().Compare(left.Key, right.Key);
    }
}

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
    public sbyte FixedSide;
    public ushort FirstPossibleSubstep;
    public ushort NextCheckSubstep;
    public uint LastSeenTimestep;
    public uint MotionVersionA;
    public uint MotionVersionB;
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

[Flags]
public enum IncrementalBodyDirtyFlags : byte
{
    None = 0,
    Motion = 1 << 0,
    Topology = 1 << 1,
    EntitySet = 1 << 2,
    CorrectedEscape = 1 << 3
}

public struct IncrementalDirtyBody
{
    public int BodyIndex;
    public IncrementalBodyDirtyFlags Flags;
}

public struct IncrementalContactCacheState
{
    public byte IsValid;
    public byte LastUpdateWasFullRebuild;
    public ushort Reserved;
    public uint Timestep;
    public uint TopologyEpoch;
    public int BodyCount;
    public int NeighborPairCount;
    public float GuardMargin;
    public float PredictiveSkin;
    public float TimestepContactMargin;
    public float SoftAvoidanceShell;
}

/// <summary>
/// Independent statistics stream for the new pipeline. Existing Stage3/Fat AABB
/// counters remain available during migration.
/// </summary>
public struct IncrementalContactPipelineStatistics
{
    public const int CurrentSchemaVersion = 2;

    public uint Timestep;

    // Proxy/topology gauges and per-timestep events.
    public int ProxyCount;
    public int TopologyDirtyBodyCount;
    public int MotionDirtyBodyCount;
    public int CorrectedEscapeBodyCount;
    public int LocalProxyQueryCount;

    public int PersistentNeighborPairCount;
    public int NeighborPairAddedCount;
    public int NeighborPairRemovedCount;
    public int NeighborPairRetainedCount;
    public int FullRebuildCount;
    public int IncrementalRepairCount;

    // Work counters: these count evaluations, not final-state pairs.
    public int ReclassifiedPairEvaluationCount;
    public int SweptClassificationEvaluationCount;
    public int ActiveConstraintEvaluationCount;

    // Current-state gauges. They are recomputed from the persistent contact
    // cache / active set and therefore remain bounded by their parent stage.
    public int CurrentSweptContactCount;
    public int CurrentDormantPairCount;
    public int CurrentApproachingPairCount;
    public int CurrentPredictivePairCount;
    public int CurrentActualPairCount;
    public int CurrentActiveConstraintCount;
    public int PeakActiveConstraintCount;

    // Unique timestep events.
    public int ScheduledWakeupCount;
    public int UniqueActivatedPairCount;
    public int UniqueCorrectedPairCount;
    public int ExpiredPairCount;

    public int OraclePairCount;
    public int OracleMissingPairCount;
    public int OracleExtraPairCount;

    // Ratios intentionally combine like-for-like gauges or unique event sets.
    public float CleanProxyRatio;
    public float RetainedNeighborPairRatio;
    public float NeighborToSweptRatio;
    public float SweptToCurrentActiveRatio;
    public float ActivatedToCorrectedRatio;

    public long ProxyValidationNanoseconds;
    public long LocalBroadPhaseNanoseconds;
    public long PairDiffNanoseconds;
    public long SweptClassificationNanoseconds;
    public long ContactActivationNanoseconds;
    public long FallbackNanoseconds;

    public byte UsedIncrementalTopology;
    public byte UsedFullRebuild;
    public byte OracleMismatch;
    public byte Reserved;
}
}
