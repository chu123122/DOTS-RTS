using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField
{

/// <summary>
/// Fat AABB 邻居缓存中的单位代理。Entity 用作跨帧稳定身份，BodyIndex 每帧刷新。
/// </summary>
public struct ShadowFatBodyProxy
{
    public Entity Entity;
    public int BodyIndex;
    public float2 FatMin;
    public float2 FatMax;
    public byte IsValid;
}

public struct ShadowEntityPair
{
    public Entity EntityA;
    public Entity EntityB;
}

public struct ShadowFatBodyProxyComparer : IComparer<ShadowFatBodyProxy>
{
    public int Compare(ShadowFatBodyProxy x, ShadowFatBodyProxy y)
    {
        return ShadowEntityOrdering.Compare(x.Entity, y.Entity);
    }
}

public struct ShadowEntityPairComparer : IComparer<ShadowEntityPair>
{
    public int Compare(ShadowEntityPair x, ShadowEntityPair y)
    {
        int first = ShadowEntityOrdering.Compare(x.EntityA, y.EntityA);
        return first != 0
            ? first
            : ShadowEntityOrdering.Compare(x.EntityB, y.EntityB);
    }
}

public static class ShadowEntityOrdering
{
    public static int Compare(Entity a, Entity b)
    {
        int indexComparison = a.Index.CompareTo(b.Index);
        return indexComparison != 0
            ? indexComparison
            : a.Version.CompareTo(b.Version);
    }

    public static ShadowEntityPair CreatePair(Entity a, Entity b)
    {
        return Compare(a, b) <= 0
            ? new ShadowEntityPair { EntityA = a, EntityB = b }
            : new ShadowEntityPair { EntityA = b, EntityB = a };
    }
}

/// <summary>
/// 持久 Fat AABB 缓存状态。Pair 只保存 Broad Phase 候选关系，不保存接触模式或 lambda。
/// </summary>
public struct FatAabbCacheState
{
    public byte IsValid;
    public int AgeFrames;
    public float PredictiveSkin;
    public float SoftAvoidanceShell;
    public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
    public float RvoTimeHorizon;
    public float Margin;
}

/// <summary>
/// Fat Swept AABB / Verlet Neighbor List 的旁路覆盖统计。
/// PreviousFrame 检查跨帧复用，CurrentFrame 检查首 substep 邻居表覆盖后续 substep。
/// </summary>
public struct ShadowNeighborCacheStatistics : IComponentData
{
    public byte CacheEnabled;
    public byte CacheValidAtFrameStart;
    public byte CacheValidAtFrameEnd;
    public int CacheAgeFrames;
    public int CacheValidationCount;
    public int CacheUseCount;
    public int CacheReuseCount;
    public int CacheRebuildCount;
    public int CacheInvalidationCount;
    public int EntitySetInvalidationCount;
    public int BoundsInvalidationCount;
    public int PostSolveInvalidationCount;
    public int FullBroadPhaseFallbackCount;
    public int CachedCandidatePairCount;
    public int CachedNarrowPhasePairCheckCount;
    public int CachePairMappingBuildCount;
    public int CachePairMappingReuseCount;
    public int CorrectedBodyValidationCount;
    public long CachePairMappingNanoseconds;

    // 以下字段保留给已有 Shadow 覆盖测试和历史 JSON，正式缓存面板不再依赖它们。
    public byte PreviousFrameCacheAvailable;
    public int PreviousFrameCacheBodyCount;
    public int PreviousFrameCachePairCount;
    public int CurrentFrameCacheBodyCount;
    public int CurrentFrameCachePairCount;

    public int PreviousFrameCheckCount;
    public int PreviousFrameAuthoritativePairCount;
    public int PreviousFramePairHitCount;
    public int PreviousFramePairMissCount;
    public int PreviousFrameActivePairMissCount;
    public int PreviousFramePredictivePairMissCount;
    public int PreviousFramePreSolveEscapeBodyCount;
    public int PreviousFrameFinalEscapeBodyCount;
    public int PreviousFrameContactDrivenEscapeBodyCount;
    public int PreviousFrameWallDrivenEscapeBodyCount;

    public int CurrentFrameCheckCount;
    public int CurrentFrameAuthoritativePairCount;
    public int CurrentFramePairHitCount;
    public int CurrentFramePairMissCount;
    public int CurrentFrameActivePairMissCount;
    public int CurrentFramePredictivePairMissCount;
    public int CurrentFramePreSolveEscapeBodyCount;
    public int CurrentFrameFinalEscapeBodyCount;
    public int CurrentFrameContactDrivenEscapeBodyCount;
    public int CurrentFrameWallDrivenEscapeBodyCount;

    public long CacheBuildNanoseconds;
    public long ValidationNanoseconds;
}
}
