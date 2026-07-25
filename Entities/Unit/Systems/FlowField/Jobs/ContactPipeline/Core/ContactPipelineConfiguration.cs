using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Normalized immutable configuration for one contact-pipeline invocation.
/// Serialized legacy names are translated at the BaseFlowMovementSystem boundary;
/// production modules consume this same-step snapshot only.
/// </summary>
public struct ContactPipelineConfiguration
{
    // Identity belongs to the scheduled simulation step, not to persistent-cache age.
    public ulong WorldId;
    public uint SimulationStepId;

    public float DeltaTime;
    public int SubstepCount;
    public int IterationCount;
    public ContactPositionSolverMode ContactPositionSolver;
    public float Compliance;
    public float PredictiveSkin;
    public float SoftAvoidanceResponseRate;
    public float SoftAvoidanceShell;
    public float SettledSoftAvoidanceMultiplier;
    public SoftAvoidanceVelocitySolverMode SoftAvoidanceVelocitySolver;
    public float RvoTimeHorizon;
    public bool EnablePredictivePairGeneration;
    public bool EnablePredictiveContacts;
#if RTS_CONTACT_DIAGNOSTICS
    public bool EnableDiagnostics;
#else
    // A property instead of a false field is deliberate: Burst compiles each Job
    // independently and cannot assume every configuration came from Create().
    // The constant getter makes every diagnostics guard a compile-time branch.
    public bool EnableDiagnostics
    {
        get => false;
        set { }
    }
#endif
    public bool EnablePersistentContactCache;
    public bool EnableTimestepContactSetCache;
    public float GuardEnvelopeMargin;
    public float TimestepContactMargin;

    /// <summary>
    /// Exact-input fingerprint used as one part of the certification evidence.
    /// It is not sufficient by itself: entity mapping and guard containment must
    /// still be verified before a certificate is issued.
    /// </summary>
    public uint CalculateCertificationFingerprint()
    {
        uint flags = 0u;
        flags |= EnablePredictivePairGeneration ? 1u << 0 : 0u;
        flags |= EnablePredictiveContacts ? 1u << 1 : 0u;
        flags |= EnablePersistentContactCache ? 1u << 2 : 0u;
        flags |= EnableTimestepContactSetCache ? 1u << 3 : 0u;
        flags |= (uint)ContactPositionSolver << 8;
        flags |= (uint)SoftAvoidanceVelocitySolver << 16;

        uint first = math.hash(new uint4(
            math.asuint(DeltaTime),
            (uint)math.max(1, SubstepCount),
            (uint)math.max(1, IterationCount),
            math.asuint(Compliance)));
        uint second = math.hash(new uint4(
            math.asuint(PredictiveSkin),
            math.asuint(GuardEnvelopeMargin),
            math.asuint(TimestepContactMargin),
            math.asuint(SoftAvoidanceShell)));
        uint third = math.hash(new uint4(
            math.asuint(SoftAvoidanceResponseRate),
            math.asuint(SettledSoftAvoidanceMultiplier),
            math.asuint(RvoTimeHorizon),
            flags));
        return math.hash(new uint3(first, second, third));
    }

    public static ContactPipelineConfiguration Create(
        ulong worldId,
        uint simulationStepId,
        float deltaTime,
        FlowFieldSettings flowSettings,
        UnitContactSolverSettings solverSettings,
        bool enablePersistentContactCache,
        bool enableTimestepContactSetCache)
    {
        return new ContactPipelineConfiguration
        {
            WorldId = worldId,
            SimulationStepId = simulationStepId,
            DeltaTime = deltaTime,
            SubstepCount = solverSettings.SubstepCount,
            IterationCount = solverSettings.IterationCount,
            ContactPositionSolver = solverSettings.ContactPositionSolver,
            Compliance = solverSettings.Compliance,
            PredictiveSkin = solverSettings.PredictiveSkin,
            SoftAvoidanceResponseRate = flowSettings.SoftAvoidanceResponseRate,
            SoftAvoidanceShell = flowSettings.SoftAvoidanceShell,
            SettledSoftAvoidanceMultiplier = flowSettings.SettledSoftAvoidanceMultiplier,
            SoftAvoidanceVelocitySolver = flowSettings.SoftAvoidanceVelocitySolver,
            RvoTimeHorizon = flowSettings.RvoTimeHorizon,
            EnablePredictivePairGeneration = solverSettings.EnablePredictivePairGeneration,
            EnablePredictiveContacts = solverSettings.EnablePredictiveContacts,
            EnableDiagnostics =
#if RTS_CONTACT_DIAGNOSTICS
                solverSettings.EnableDiagnostics,
#else
                false,
#endif
            EnablePersistentContactCache = enablePersistentContactCache,
            EnableTimestepContactSetCache = enableTimestepContactSetCache,
            // Compatibility translation: the serialized FatAabb margin now means
            // the persistent guarded-proxy envelope margin.
            GuardEnvelopeMargin = solverSettings.PersistentGuardEnvelopeMargin,
            TimestepContactMargin = solverSettings.TimestepContactMargin
        };
    }
}
}
