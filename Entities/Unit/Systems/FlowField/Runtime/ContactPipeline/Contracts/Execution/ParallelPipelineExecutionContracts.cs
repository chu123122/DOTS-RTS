using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{

public struct SoftAvoidancePairContribution
{
    public float3 VelocityA;
    public float3 VelocityB;
    public byte ActiveA;
    public byte ActiveB;
}

public struct ParallelBodyStageResult
{
    // EscapeCount is authoritative: it drives dirty-body repair and must exist
    // independently of observation.
    public int EscapeCount;
#if RTS_CONTACT_DIAGNOSTICS
    public float Total;
    public float Maximum;
    public float SecondaryTotal;
    public float TertiaryTotal;
    public int Count;
    public int ActivatedCount;
#else
    public float Total { get => default; set { } }
    public float Maximum { get => default; set { } }
    public float SecondaryTotal { get => default; set { } }
    public float TertiaryTotal { get => default; set { } }
    public int Count { get => default; set { } }
    public int ActivatedCount { get => default; set { } }
#endif
}

public struct ActiveIncidentIndexState
{
    public ulong Fingerprint;
    public int PairCount;
    public int BodyCount;
    public int SoftPairCount;
    public int SoftBodyCount;
    public byte IsValid;
    public byte SoftIsValid;
}

/// <summary>
/// Named responsibilities behind the historical P1-P6 implementation labels.
/// The enum is documentation for scheduling boundaries, not mutable runtime state.
/// </summary>
internal enum StagedContactPipelinePhase : byte
{
    Initialize,
    ResolveInteractionSource,
    RepairPersistentTopology,
    BuildSoftAvoidanceView,
    SolveContactConstraints,
    ReconstructVelocity,
    FinalizeTimestep
}

/// <summary>
/// Staged parallel contact pipeline. The historical P1-P6 labels map to the
/// named <see cref="StagedContactPipelinePhase"/> responsibilities above.
/// Independent body/pair work runs in parallel; topology mutation, repair and
/// deterministic compaction remain serialized at explicit phase boundaries.
/// </summary>
}
