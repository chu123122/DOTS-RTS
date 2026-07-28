using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Structural reusability check and configuration fingerprint for the
/// persistent (P1P6) contact cache. The cache can be incrementally patched
/// only when the committed cache state still matches the current body/solver
/// configuration; any drift forces a full rebuild. Pure value functions.
/// </summary>
internal static class PersistentCacheReusability
{
    /// <summary>
    /// Snapshot of the configuration axes that gate cache reuse, captured at
    /// the point of the check.
    /// </summary>
    internal struct ConfigurationFingerprint
    {
        public float GuardMargin;
        public float PredictiveSkin;
        public float TimestepContactMargin;
        public float SoftAvoidanceShell;
        public float SoftAvoidanceResponseRate;
        public float RvoTimeHorizon;
        public int SubstepCount;
        public bool PredictivePairGenerationEnabled;
        public bool PredictiveContactsEnabled;
        public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
    }

    /// <summary>
    /// Whether the cache view can be patched in place: the state must be valid,
    /// sized to the current body count, and every configuration axis must match
    /// the fingerprint recorded when the cache was last built.
    /// </summary>
    internal static bool IsStructurallyReusable(
        IncrementalContactCacheState state,
        int bodyCount,
        int persistentProxyCount,
        int proxyIndexByBodyCount,
        ConfigurationFingerprint config)
    {
        return state.IsValid != 0 &&
               state.BodyCount == bodyCount &&
               persistentProxyCount == bodyCount &&
               proxyIndexByBodyCount == bodyCount &&
               state.GuardMargin == math.max(0f, config.GuardMargin) &&
               state.PredictiveSkin == math.max(0f, config.PredictiveSkin) &&
               state.TimestepContactMargin == math.max(0f, config.TimestepContactMargin) &&
               state.SoftAvoidanceShell == math.max(0f, config.SoftAvoidanceShell) &&
               state.SoftAvoidanceResponseRate == math.max(0f, config.SoftAvoidanceResponseRate) &&
               state.RvoTimeHorizon == math.max(0f, config.RvoTimeHorizon) &&
               state.SubstepCount == math.max(1, config.SubstepCount) &&
               state.PredictivePairGenerationEnabled == (byte)(config.PredictivePairGenerationEnabled ? 1 : 0) &&
               state.PredictiveContactsEnabled == (byte)(config.PredictiveContactsEnabled ? 1 : 0) &&
               state.SoftAvoidanceVelocitySolver == (byte)config.SoftAvoidanceVelocitySolver;
    }
}
}
