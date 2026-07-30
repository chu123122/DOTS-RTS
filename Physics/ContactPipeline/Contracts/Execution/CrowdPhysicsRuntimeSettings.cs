using Unity.Entities;

namespace RTS.Unit.FlowField
{

public enum SoftAvoidanceVelocitySolverMode : byte
{
    SurfaceVelocityBuffer,
    ReciprocalVelocityObstacle
}

public enum ContactPositionSolverMode : byte
{
    GaussSeidel,
    Jacobi
}

public enum ContactSolverSkipReason : byte
{
    None,
    CertificateUnavailable,
    CertificateNotIssued,
    CertificateStructureNotVerified,
    CertificateInteractionCountInvalid,
    CertificateViewUnavailable,
    CertificateInteractionPairInvalid,
    CertificateSoftPairInvalid,
    CertificateContactPairInvalid,
    CertificateScheduleInvalid,
    CertificateEntityMappingNotVerified,
    CertificateConfigurationNotVerified,
    CertificateTopologyNotVerified,
    CertificateClassificationNotVerified,
    CertificateConsumerViewsNotCommitted,
    CertificateScopeMismatch,
    BodySetMismatch,
    ConfigurationMismatch,
    SoftPairCountMismatch,
    ContactConstraintCountMismatch,
    DormantScheduleCountMismatch
}

public enum ContactHeatmapMode : byte
{
    ContactLoad,
    ContactSetDensity,
    EscapeFallback
}

/// <summary>
/// 单位动态接触 XPBD 求解配置。lambda 在每个 substep 开始时清零。
/// </summary>
public struct UnitContactSolverSettings : IComponentData
{
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
    public bool VisualizeSelectedContacts;
    public float DiagnosticCaptureDuration;
    public float DiagnosticCaptureInterval;
    public bool EnableTimestepContactSetCache;
    public bool EnablePersistentContactCache;
    public float PersistentGuardEnvelopeMargin;
    public float TimestepContactMargin;
    public bool VisualizeContactHeatmap;
    public ContactHeatmapMode ContactHeatmapMode;
    public float DiagnosticSlowMotionScale;
}

}
