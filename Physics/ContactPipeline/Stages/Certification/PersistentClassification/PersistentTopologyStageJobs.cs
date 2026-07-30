using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{
[BurstCompile]
internal struct PreparePersistentTopologyPublicationJob : IJob
{
    [ReadOnly] public NativeReference<byte> FullSweepPrepared;
    [ReadOnly] public NativeList<BodyPair> TimestepInteractionPairs;
    public NativeList<PersistentSweptProxy> PreviousProxies;
    public NativeList<PersistentSweptProxy> PersistentProxies;
    public NativeList<int> ProxyIndexByBody;
    public NativeList<PersistentNeighborPair> PersistentPairs;
    public int BodyCount;

    public void Execute()
    {
        if (FullSweepPrepared.Value == 0)
            return;

        PreviousProxies.Clear();
        PreviousProxies.AddRange(PersistentProxies.AsArray());
        PersistentProxies.ResizeUninitialized(BodyCount);
        ProxyIndexByBody.ResizeUninitialized(BodyCount);
        PersistentPairs.ResizeUninitialized(TimestepInteractionPairs.Length);
    }
}

[BurstCompile]
internal struct BuildPersistentProxiesJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeReference<byte> FullSweepPrepared;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
    [ReadOnly] public NativeArray<PersistentSweptProxy> PreviousProxies;
    [ReadOnly] public NativeArray<int> PreviousProxyIndexByBody;
    [NativeDisableParallelForRestriction]
    public NativeArray<PersistentSweptProxy> PersistentProxies;
    public float GuardMargin;
    public float SoftAvoidanceShell;
    public float SoftAvoidanceResponseRate;
    public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
    public float RvoTimeHorizon;

    public void Execute(int bodyIndex)
    {
        if (FullSweepPrepared.Value == 0)
            return;

        CrowdBodySnapshot body = Bodies[bodyIndex];
        PersistentSweptProxy current =
            PersistentProxyBuilder.BuildFromState(
                bodyIndex,
                body,
                MotionEvidence[bodyIndex],
                StepStates[bodyIndex],
                GuardMargin,
                SoftAvoidanceShell,
                SoftAvoidanceResponseRate,
                SoftAvoidanceVelocitySolver,
                RvoTimeHorizon);
        if (TryFindPrevious(bodyIndex, out PersistentSweptProxy previous))
            PersistentProxyBuilder.AssignMotionVersion(ref current, previous);
        PersistentProxies[bodyIndex] = current;
    }

    private bool TryFindPrevious(
        int bodyIndex,
        out PersistentSweptProxy proxy)
    {
        if ((uint)bodyIndex < (uint)PreviousProxyIndexByBody.Length)
        {
            int proxyIndex = PreviousProxyIndexByBody[bodyIndex];
            if ((uint)proxyIndex < (uint)PreviousProxies.Length)
            {
                proxy = PreviousProxies[proxyIndex];
                return proxy.BodyIndex == bodyIndex;
            }
        }
        proxy = default;
        return false;
    }
}

[BurstCompile]
internal struct BuildPersistentProxyIndexJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeReference<byte> FullSweepPrepared;
    [ReadOnly] public NativeArray<PersistentSweptProxy> PersistentProxies;
    [NativeDisableParallelForRestriction]
    public NativeArray<int> ProxyIndexByBody;

    public void Execute(int proxyIndex)
    {
        if (FullSweepPrepared.Value == 0)
            return;
        int bodyIndex = PersistentProxies[proxyIndex].BodyIndex;
        if ((uint)bodyIndex < (uint)ProxyIndexByBody.Length)
            ProxyIndexByBody[bodyIndex] = proxyIndex;
    }
}

[BurstCompile]
internal struct PublishPersistentNeighborPairsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<ContactConstraint> Workset;
    [ReadOnly] public NativeReference<byte> FullSweepPrepared;
    [ReadOnly] public NativeArray<BodyPair> BodyPairs;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeReference<IncrementalContactCacheState> CacheState;
    [NativeDisableParallelForRestriction]
    public NativeArray<PersistentNeighborPair> PersistentPairs;

    public void Execute(int pairIndex)
    {
        if (FullSweepPrepared.Value == 0)
            return;
        BodyPair pair = BodyPairs[pairIndex];
        IncrementalContactCacheState state = CacheState.Value;
        PersistentPairs[pairIndex] = new PersistentNeighborPair
        {
            Key = StableEntityPairKey.Create(
                Bodies[pair.BodyA].Entity,
                Bodies[pair.BodyB].Entity),
            TopologyEpoch = state.TopologyEpoch + 1u,
            LastValidatedTimestep = state.Timestep + 1u
        };
    }
}

[BurstCompile]
internal struct PreparePersistentReusePublicationJob : IJob
{
    [ReadOnly] public NativeReference<ContactPipelineExecutionState>
        RuntimeState;
    [ReadOnly] public NativeReference<IncrementalContactCacheState>
        CacheState;
    [ReadOnly] public NativeList<PersistentSweptProxy> PersistentProxies;
    [ReadOnly] public NativeList<int> PersistentProxyIndexByBody;
    [ReadOnly] public NativeList<IncrementalDirtyBody> DirtyBodies;
    [ReadOnly] public NativeList<PersistentNeighborPair> PersistentPairs;
    public NativeReference<byte> FullSweepPrepared;
    public NativeList<byte> PairWorkset;
    public NativeList<ContactConstraint> MappedPairs;
    public int BodyCount;
    public ContactPipelineConfiguration Configuration;
    public byte Enabled;

    public void Execute()
    {
        PairWorkset.Clear();
        if (Enabled == 0 ||
            RuntimeState.Value.IsValid == 0 ||
            FullSweepPrepared.Value != 0 ||
            DirtyBodies.Length != 0)
            return;

        IncrementalContactCacheState state = CacheState.Value;
        if (state.ContactViewsValid == 0 ||
            !PersistentCacheReusability.IsStructurallyReusable(
                state,
                BodyCount,
                PersistentProxies.Length,
                PersistentProxyIndexByBody.Length,
                Configuration))
            return;

        MappedPairs.ResizeUninitialized(PersistentPairs.Length);
        PairWorkset.ResizeUninitialized(PersistentPairs.Length);
        FullSweepPrepared.Value = 2;
    }
}

[BurstCompile]
internal struct MapPersistentReusePairsJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<byte> Workset;
    [ReadOnly] public NativeReference<byte> FullSweepPrepared;
    [ReadOnly] public NativeArray<PersistentNeighborPair> PersistentPairs;
    [ReadOnly] public NativeParallelHashMap<Entity, int>
        CurrentBodyIndexByEntity;
    [NativeDisableParallelForRestriction]
    public NativeArray<ContactConstraint> MappedPairs;

    public void Execute(int pairIndex)
    {
        if (FullSweepPrepared.Value != 2)
            return;

        StableEntityPairKey key = PersistentPairs[pairIndex].Key;
        if (!CurrentBodyIndexByEntity.TryGetValue(
                key.EntityA, out int bodyA) ||
            !CurrentBodyIndexByEntity.TryGetValue(
                key.EntityB, out int bodyB))
        {
            MappedPairs[pairIndex] = new ContactConstraint
            {
                Definition = new ContactConstraintDefinition
                {
                    BodyA = -1,
                    BodyB = -1
                }
            };
            return;
        }

        MappedPairs[pairIndex] = new ContactConstraint
        {
            Definition = new ContactConstraintDefinition
            {
                BodyA = math.min(bodyA, bodyB),
                BodyB = math.max(bodyA, bodyB)
            }
        };
    }
}

[BurstCompile]
internal struct FinalizePersistentReusePublicationJob : IJob
{
    public NativeReference<ContactPipelineExecutionState> RuntimeState;
    public NativeReference<IncrementalContactCacheState> CacheState;
    [ReadOnly] public NativeReference<byte> FullSweepPrepared;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
#endif

    public void Execute()
    {
        if (RuntimeState.Value.IsValid == 0 ||
            FullSweepPrepared.Value != 1)
            return;

        IncrementalContactCacheState state = CacheState.Value;
        state.Timestep++;
        state.LastUpdateWasFullRebuild = 0;
        CacheState.Value = state;
#if RTS_CONTACT_DIAGNOSTICS
        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
        incremental.Timestep = state.Timestep;
        incremental.UsedFullRebuild = 0;
        IncrementalStatistics.Value = incremental;
#endif
    }
}

[BurstCompile]
internal struct FinalizePersistentTopologyPublicationJob : IJob
{
    [ReadOnly] public NativeReference<byte> FullSweepPrepared;
    public NativeList<PersistentNeighborPair> PersistentPairs;
    public NativeReference<IncrementalContactCacheState> CacheState;
    public NativeReference<uint> PersistentSpatialMembershipEpoch;
    public NativeReference<uint> PersistentIncidentLookupEpoch;
    public ContactPipelineConfiguration Configuration;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<IncrementalContactPipelineStatistics>
        IncrementalStatistics;
#endif
    public int BodyCount;

    public void Execute()
    {
        if (FullSweepPrepared.Value == 0)
            return;

        IncrementalContactCacheState state = CacheState.Value;
        state.IsValid = 1;
        state.LastUpdateWasFullRebuild = 1;
        state.Timestep++;
        state.TopologyEpoch++;
        state.ObstacleVersion = Configuration.ObstacleVersion;
        state.BodyCount = BodyCount;
        state.NeighborPairCount = PersistentPairs.Length;
        state.GuardMargin =
            math.max(0f, Configuration.GuardEnvelopeMargin);
        state.PredictiveSkin =
            math.max(0f, Configuration.PredictiveSkin);
        state.TimestepContactMargin =
            math.max(0f, Configuration.TimestepContactMargin);
        state.SoftAvoidanceShell =
            math.max(0f, Configuration.SoftAvoidanceShell);
        state.SoftAvoidanceResponseRate =
            math.max(0f, Configuration.SoftAvoidanceResponseRate);
        state.RvoTimeHorizon =
            math.max(0f, Configuration.RvoTimeHorizon);
        state.SubstepCount =
            math.max(1, Configuration.SubstepCount);
        state.PredictivePairGenerationEnabled = (byte)(
            Configuration.EnablePredictivePairGeneration ? 1 : 0);
        state.PredictiveContactsEnabled = (byte)(
            Configuration.EnablePredictiveContacts ? 1 : 0);
        state.SoftAvoidanceVelocitySolver =
            (byte)Configuration.SoftAvoidanceVelocitySolver;
        CacheState.Value = state;

        PersistentSpatialMembershipEpoch.Value = 0;
        PersistentIncidentLookupEpoch.Value = 0;

#if RTS_CONTACT_DIAGNOSTICS
        IncrementalContactPipelineStatistics incremental =
            IncrementalStatistics.Value;
        incremental.Timestep = state.Timestep;
        incremental.ProxyCount = BodyCount;
        incremental.PersistentNeighborPairCount = PersistentPairs.Length;
        incremental.NeighborPairAddedCount = PersistentPairs.Length;
        incremental.FullRebuildCount++;
        incremental.UsedFullRebuild = 1;
        IncrementalStatistics.Value = incremental;
#endif
    }
}
}
