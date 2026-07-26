using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Persistent swept-proxy construction and dirty-flag classification for the
/// incremental (P1P6) contact path. Pure value functions: no job state, no
/// instance fields — all inputs arrive as parameters. Shared by the certifier
/// job and unit-testable in isolation.
/// </summary>
internal static class PersistentProxyBuilder
{
    /// <summary>
    /// Builds a swept-proxy envelope from a body's step state. The proxy carries
    /// tight interaction bounds plus an inflated guard band; the avoidance
    /// horizon end is only extended for the RVO solver mode.
    /// </summary>
    internal static PersistentSweptProxy BuildFromState(
        int bodyIndex,
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        float guardMargin,
        float softAvoidanceShell,
        float softAvoidanceResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon)
    {
        PersistentSweptProxy proxy = new PersistentSweptProxy
        {
            Entity = stateSnapshot.Entity,
            BodyIndex = bodyIndex,
            IsValid = (byte)((stateSnapshot.IsInsideSimulationDomain != 0) ? 1 : 0),
            Radius = math.max(0f, stateSnapshot.Radius)
        };
        if (!(stateSnapshot.IsInsideSimulationDomain != 0))
            return proxy;
        proxy.TightMin = stateEvidence.InteractionEnvelopeMin;
        proxy.TightMax = stateEvidence.InteractionEnvelopeMax;
        proxy.GuardMin = proxy.TightMin - math.max(0f, guardMargin);
        proxy.GuardMax = proxy.TightMax + math.max(0f, guardMargin);
        proxy.TrajectoryStart = stateEvidence.TrajectoryStart.xz;
        proxy.TrajectoryEnd = stateEvidence.BaselineEnd.xz;
        proxy.AvoidanceHorizonEnd =
            softSolverMode == SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle &&
            softAvoidanceShell > 0f && softAvoidanceResponseRate > 0f
                ? stateEvidence.TrajectoryStart.xz +
                  stateStep.BaseVelocity.xz * math.max(0f, rvoTimeHorizon)
                : stateEvidence.BaselineEnd.xz;
        proxy.MotionVersion = 1u;
        return proxy;
    }

    /// <summary>
    /// Assigns a monotonic motion version to <paramref name="current"/> based on
    /// whether its trajectory/radius changed against <paramref name="previous"/>.
    /// Stable trajectories inherit the previous version; moving ones bump it.
    /// </summary>
    internal static void AssignMotionVersion(
        ref PersistentSweptProxy current,
        PersistentSweptProxy previous)
    {
        bool same = math.all(current.TrajectoryStart == previous.TrajectoryStart) &&
                    math.all(current.TrajectoryEnd == previous.TrajectoryEnd) &&
                    math.all(current.AvoidanceHorizonEnd ==
                             previous.AvoidanceHorizonEnd) &&
                    current.Radius == previous.Radius;
        current.MotionVersion = same
            ? previous.MotionVersion
            : previous.MotionVersion == uint.MaxValue
                ? 1u
                : previous.MotionVersion + 1u;
    }

    /// <summary>
    /// Compares a body's current state against its cached proxy and, when dirty,
    /// writes the updated proxy back into the persistent store. Returns the
    /// union of dirty flags (entity-set / topology / motion). The cache view is
    /// treated as unreadable when its validity or sizing is inconsistent.
    /// </summary>
    internal static IncrementalBodyDirtyFlags ClassifyAndUpdateForBody(
        int bodyIndex,
        CrowdBodySnapshot stateSnapshot,
        CrowdMotionEvidence stateEvidence,
        CrowdBodyStepState stateStep,
        NativeArray<PersistentSweptProxy> persistentProxies,
        NativeArray<int> proxyIndexByBody,
        IncrementalContactCacheState cacheState,
        float guardMargin,
        float softAvoidanceShell,
        float softAvoidanceResponseRate,
        SoftAvoidanceVelocitySolverMode softSolverMode,
        float rvoTimeHorizon)
    {
        if (cacheState.IsValid == 0 ||
            proxyIndexByBody.Length != cacheState.BodyCount ||
            persistentProxies.Length != cacheState.BodyCount ||
            (uint)bodyIndex >= (uint)proxyIndexByBody.Length)
            return IncrementalBodyDirtyFlags.None;

        int proxyIndex = proxyIndexByBody[bodyIndex];
        if ((uint)proxyIndex >= (uint)persistentProxies.Length)
            return IncrementalBodyDirtyFlags.EntitySet |
                   IncrementalBodyDirtyFlags.Topology |
                   IncrementalBodyDirtyFlags.Motion;

        PersistentSweptProxy previous = persistentProxies[proxyIndex];
        if (previous.Entity != stateSnapshot.Entity)
            return IncrementalBodyDirtyFlags.EntitySet |
                   IncrementalBodyDirtyFlags.Topology |
                   IncrementalBodyDirtyFlags.Motion;

        PersistentSweptProxy current = BuildFromState(
            bodyIndex, stateSnapshot, stateEvidence, stateStep,
            guardMargin, softAvoidanceShell,
            softAvoidanceResponseRate, softSolverMode, rvoTimeHorizon);
        AssignMotionVersion(ref current, previous);
        bool topologyDirty = previous.IsValid != current.IsValid ||
                             previous.Radius != current.Radius ||
                             (current.IsValid != 0 && !ContactPipelineShared.AabbContains(
                                 previous.GuardMin, previous.GuardMax,
                                 current.TightMin, current.TightMax));
        bool motionDirty = topologyDirty || current.MotionVersion != previous.MotionVersion;
        if (!motionDirty)
            return IncrementalBodyDirtyFlags.None;
        if (!topologyDirty)
        {
            // Uniform translation path: slide the previous guard by this
            // frame's tight displacement so steady formation motion never
            // trips the topology guard (AabbContains measures relative, not
            // absolute, drift). Displacement is taken from the tight AABB,
            // not the trajectory intent, so the guard tracks the real hull.
            float2 displacement = current.TightMin - previous.TightMin;
            current.GuardMin = previous.GuardMin + displacement;
            current.GuardMax = previous.GuardMax + displacement;
            // Acceleration, reversal or wall-bounce can move the tight AABB
            // past the translated guard. Re-test containment against the
            // translated guard and, if the tight hull still escapes, fall
            // back to the authoritative topology-rebuild path.
            if (!ContactPipelineShared.AabbContains(
                    current.GuardMin, current.GuardMax,
                    current.TightMin, current.TightMax))
                topologyDirty = true;
        }
        persistentProxies[proxyIndex] = current;
        return topologyDirty
            ? IncrementalBodyDirtyFlags.Motion | IncrementalBodyDirtyFlags.Topology
            : IncrementalBodyDirtyFlags.Motion;
    }
}
}
