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
    // EscapeCount 是权威量：驱动 dirty-body 修复，必须独立于观测字段存在。
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
/// 共享接触管线的阶段职责。该枚举是调度边界文档，并非可变运行时状态。
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
/// 分阶段并行接触管线。独立 body/pair 工作并行运行；拓扑变更、修复与
/// 确定性压缩仍在显式阶段边界保持串行协调。
/// </summary>
}
