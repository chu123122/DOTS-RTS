using System;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 已认证交互产物的来源。调度器可将该证书用作 fail-closed 闸门；下游仿真阶段不得再解读来源模式或访问持久候选状态。
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
    RepairCoverageFailed,
    CertificateScopeMismatch,
    CommittedViewMismatch
}

/// <summary>
/// 当前提供给认证器的权威事实。刻意不包含候选持久容器。
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
/// 紧密的交互/接触/调度视图所附作用域，供下游阶段消费。在该精确作用域内视图为权威，无需访问持久候选状态。
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
    public ContactSolverSkipReason StructureFailure;
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
/// 运动或约束求解离开当前证书证明作用域时上报的证据。产生方只报告事实；接受/修复/重建与候选缓存变更仍由认证器独占。
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
/// 已认证产物的无容器描述。物理 NativeList 视图仍由 timestep 资源持有，从而干净的持久路径不必物化一个巨大的通用交互数组。
/// </summary>
public struct CertifiedInteractionProductDescriptor
{
    public InteractionCertificate Certificate;
    public int SoftPairCount;
    public int ContactConstraintCount;
    public int DormantScheduleCount;
}
}
