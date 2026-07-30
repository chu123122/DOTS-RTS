using System;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 持久 body proxy。Guard 边界证明拓扑完整性；tight 边界描述最近一次分类的预测视域。
/// </summary>
public struct PersistentSweptProxy
{
    public Entity Entity;
    public int BodyIndex;
    public float2 TightMin;
    public float2 TightMax;
    public float2 GuardMin;
    public float2 GuardMax;
    // 位置圆守护边界（velocity-independent）：仅当物理位置逃出此圆时才标记 topology dirty。
    // 不同于 GuardMin/GuardMax（基于 InteractionEnvelope，速度方向敏感），
    // 转向/加速不会触发拓扑重建，只有真正跑远才会。
    public float2 TopologyGuardMin;
    public float2 TopologyGuardMax;
    // 精确分类输入；MotionVersion 通过这些字段逐位相等推进，不再用 32-bit hash 当正确性依据。
    public float2 TrajectoryStart;
    public float2 TrajectoryEnd;
    public float2 AvoidanceHorizonEnd;
    public float Radius;
    public uint ShapeVersion;
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
