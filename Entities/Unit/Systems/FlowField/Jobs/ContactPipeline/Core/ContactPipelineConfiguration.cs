namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Normalized runtime configuration for one contact-pipeline invocation.
/// Serialized legacy names are translated at the BaseFlowMovementSystem boundary;
/// production pipeline modules only consume the semantics defined here.
/// </summary>
public struct ContactPipelineConfiguration
{
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
    public bool EnableDiagnostics;
    public bool EnablePersistentContactCache;
    public bool EnableTimestepContactSetCache;
    public float GuardEnvelopeMargin;
    public float TimestepContactMargin;

    public static ContactPipelineConfiguration Create(
        float deltaTime,
        FlowFieldSettings flowSettings,
        UnitContactSolverSettings solverSettings,
        bool enablePersistentContactCache,
        bool enableTimestepContactSetCache)
    {
        return new ContactPipelineConfiguration
        {
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
            EnableDiagnostics = solverSettings.EnableDiagnostics,
            EnablePersistentContactCache = enablePersistentContactCache,
            EnableTimestepContactSetCache = enableTimestepContactSetCache,
            // Compatibility translation: the serialized FatAabb margin now means
            // the persistent guarded-proxy envelope margin.
            GuardEnvelopeMargin = solverSettings.FatAabbCacheMargin,
            TimestepContactMargin = solverSettings.TimestepContactMargin
        };
    }
}
}
