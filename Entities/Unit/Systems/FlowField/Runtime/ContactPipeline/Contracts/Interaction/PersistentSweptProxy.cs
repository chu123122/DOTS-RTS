using System;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
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
    // 精确分类输入。MotionVersion 由这些字段的逐位相等比较推进，
    // 不再把 32-bit hash 当作正确性依据。
    public float2 TrajectoryStart;
    public float2 TrajectoryEnd;
    public float2 AvoidanceHorizonEnd;
    public float Radius;
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
}
