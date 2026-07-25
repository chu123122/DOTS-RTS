using System;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Provenance of a certified interaction product. Gameplay consumers must not
/// branch on this value; it exists for certification audit and diagnostics only.
/// </summary>
public enum CertifiedInteractionSourceMode : byte
{
    FullSweep,
    PersistentReuse,
    PersistentRepair,
    PersistentFullRebuild
}

[Flags]
public enum InteractionCertificationFlags : ushort
{
    None = 0,
    StructureVerified = 1 << 0,
    EntityMappingVerified = 1 << 1,
    ConfigurationVerified = 1 << 2,
    TopologyCoverageVerified = 1 << 3,
    ClassificationVerified = 1 << 4,
    ConsumerViewsCommitted = 1 << 5,
    Issued = 1 << 15
}

public enum InteractionCertificateViolationReason : byte
{
    None,
    BaseMotionEnvelopeEscape,
    PredictedContactEnvelopeEscape,
    SolverCorrectionEnvelopeEscape,
    EntitySetChanged,
    ConfigurationChanged,
    MappingFailed,
    RepairCoverageFailed
}

/// <summary>
/// Current authoritative facts presented to the certifier. Candidate persistent
/// containers are deliberately absent from this value.
/// </summary>
public struct InteractionCertificationEvidence
{
    public ulong WorldId;
    public uint SimulationStepId;
    public uint BodySetFingerprint;
    public uint ConfigurationFingerprint;
    public uint TopologyEpoch;
    public uint ClassificationFingerprint;
    public ushort StartSubstep;
    public ushort EndSubstepExclusive;
    public float HorizonDuration;
    public int BodyCount;
}

/// <summary>
/// Scope attached to the compact interaction/contact/schedule views consumed by
/// lower stages. Within this exact scope the views are authoritative and may be
/// consumed without consulting persistent candidate state.
/// </summary>
public struct InteractionCertificate
{
    public ulong WorldId;
    public uint SimulationStepId;
    public uint BodySetFingerprint;
    public uint ConfigurationFingerprint;
    public uint TopologyEpoch;
    public uint ClassificationFingerprint;
    public ushort StartSubstep;
    public ushort EndSubstepExclusive;
    public float HorizonDuration;
    public CertifiedInteractionSourceMode SourceMode;
    public InteractionCertificationFlags Flags;
    public int InteractionPairCount;
    public int SoftPairCount;
    public int ContactConstraintCount;
    public int DormantScheduleCount;

    public bool IsIssued =>
        (Flags & InteractionCertificationFlags.Issued) != 0;

    public bool Covers(ulong worldId, uint stepId, int substep) =>
        IsIssued && WorldId == worldId && SimulationStepId == stepId &&
        substep >= StartSubstep && substep < EndSubstepExclusive;

    public static InteractionCertificate Invalid => default;
}

/// <summary>
/// Evidence emitted when motion or constraint solving leaves the scope proved by
/// the current certificate. Producers report facts only; the certifier remains
/// the sole owner of accept/repair/rebuild and candidate-cache mutation.
/// </summary>
public struct InteractionCertificateViolation
{
    public int BodyIndex;
    public ushort FirstInvalidSubstep;
    public InteractionCertificateViolationReason Reason;
    public float2 ObservedMin;
    public float2 ObservedMax;
}

/// <summary>
/// Container-free description of the certified product. Physical NativeList views
/// stay owned by timestep resources so clean persistent paths need not materialize
/// one large universal interaction array.
/// </summary>
public struct CertifiedInteractionProductDescriptor
{
    public InteractionCertificate Certificate;
    public int SoftPairCount;
    public int ContactConstraintCount;
    public int DormantScheduleCount;
}
}
