using System;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 所有跨 timestep 接触容器使用的稳定对身份。Body 索引起的是帧级查询顺序，故被刻意排除。
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
}
