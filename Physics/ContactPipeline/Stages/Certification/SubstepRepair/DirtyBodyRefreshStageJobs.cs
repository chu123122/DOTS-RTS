using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;

namespace RTS.Unit.FlowField.Jobs
{
public struct DirtyBodyRefreshResult
{
    public IncrementalBodyDirtyFlags Flags;
    public byte IsReusable;
}

public struct DirtyBodyRefreshSummary
{
    public int TopologyDirtyCount;
    public int MotionDirtyCount;
    public byte EntitySetDirty;
    public byte IsReusable;
}

[BurstCompile]
internal struct RefreshDirtyBodiesJob : IJobParallelForDefer
{
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdMotionEvidence> MotionEvidence;
    [ReadOnly] public NativeArray<CrowdBodyStepState> StepStates;
    [NativeDisableParallelForRestriction]
    public NativeArray<IncrementalDirtyBody> DirtyBodies;
    [NativeDisableParallelForRestriction]
    public NativeArray<byte> DirtyFlagsByBody;
    [NativeDisableParallelForRestriction]
    public NativeArray<PersistentSweptProxy> PersistentProxies;
    [ReadOnly] public NativeArray<int> PersistentProxyIndexByBody;
    [ReadOnly] public NativeReference<IncrementalContactCacheState> CacheState;
    public NativeArray<DirtyBodyRefreshResult> Results;

    public float GuardMargin;
    public float PredictiveSkin;
    public float TimestepContactMargin;
    public float SoftAvoidanceShell;
    public float SoftAvoidanceResponseRate;
    public float RvoTimeHorizon;
    public int SubstepCount;
    public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
    public byte PredictivePairGenerationEnabled;
    public byte PredictiveContactsEnabled;
    public byte Enabled;

    public void Execute(int dirtyIndex)
    {
        DirtyBodyRefreshResult result = default;
        if (Enabled == 0 ||
            !PersistentCacheReusability.IsStructurallyReusable(
                CacheState.Value,
                Bodies.Length,
                PersistentProxies.Length,
                PersistentProxyIndexByBody.Length,
                new PersistentCacheReusability.ConfigurationFingerprint
                {
                    GuardMargin = GuardMargin,
                    PredictiveSkin = PredictiveSkin,
                    TimestepContactMargin = TimestepContactMargin,
                    SoftAvoidanceShell = SoftAvoidanceShell,
                    SoftAvoidanceResponseRate = SoftAvoidanceResponseRate,
                    RvoTimeHorizon = RvoTimeHorizon,
                    SubstepCount = SubstepCount,
                    PredictivePairGenerationEnabled =
                        PredictivePairGenerationEnabled != 0,
                    PredictiveContactsEnabled = PredictiveContactsEnabled != 0,
                    SoftAvoidanceVelocitySolver =
                        SoftAvoidanceVelocitySolver
                }))
        {
            Results[dirtyIndex] = result;
            return;
        }

        IncrementalDirtyBody dirty = DirtyBodies[dirtyIndex];
        int bodyIndex = dirty.BodyIndex;
        if ((uint)bodyIndex >= (uint)Bodies.Length)
        {
            Results[dirtyIndex] = result;
            return;
        }

        IncrementalBodyDirtyFlags refreshed =
            CertificationStageKernel.ClassifyAndUpdatePersistentProxyForBody(
                bodyIndex,
                Bodies[bodyIndex],
                MotionEvidence[bodyIndex],
                StepStates[bodyIndex],
                PersistentProxies,
                PersistentProxyIndexByBody,
                CacheState.Value,
                GuardMargin,
                SoftAvoidanceShell,
                SoftAvoidanceResponseRate,
                SoftAvoidanceVelocitySolver,
                RvoTimeHorizon);
        IncrementalBodyDirtyFlags merged =
            dirty.Flags | refreshed | IncrementalBodyDirtyFlags.Motion;
        dirty.Flags = merged;
        DirtyBodies[dirtyIndex] = dirty;
        DirtyFlagsByBody[bodyIndex] = (byte)merged;
        result.Flags = merged;
        result.IsReusable =
            (byte)((merged & IncrementalBodyDirtyFlags.EntitySet) == 0 ? 1 : 0);
        Results[dirtyIndex] = result;
    }
}

[BurstCompile]
internal struct ReduceDirtyBodyRefreshJob : IJob
{
    [ReadOnly] public NativeArray<IncrementalDirtyBody> DirtyBodies;
    [ReadOnly] public NativeArray<DirtyBodyRefreshResult> Results;
    public NativeReference<DirtyBodyRefreshSummary> Summary;
    public byte Enabled;

    public void Execute()
    {
        DirtyBodyRefreshSummary summary = new DirtyBodyRefreshSummary
        {
            IsReusable = Enabled
        };
        for (int dirtyIndex = 0; dirtyIndex < DirtyBodies.Length; dirtyIndex++)
        {
            DirtyBodyRefreshResult result = Results[dirtyIndex];
            if (result.IsReusable == 0)
                summary.IsReusable = 0;
            if ((result.Flags & IncrementalBodyDirtyFlags.EntitySet) != 0)
                summary.EntitySetDirty = 1;
            if ((result.Flags & IncrementalBodyDirtyFlags.Topology) != 0)
                summary.TopologyDirtyCount++;
            else if ((result.Flags & IncrementalBodyDirtyFlags.Motion) != 0)
                summary.MotionDirtyCount++;
        }
        Summary.Value = summary;
    }
}
}
