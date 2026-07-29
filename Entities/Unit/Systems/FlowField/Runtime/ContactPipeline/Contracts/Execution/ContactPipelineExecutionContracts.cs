using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Profiling.LowLevel.Unsafe;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{

public struct ContactPipelineExecutionState
{
    public byte IsValid;
    public byte RecoveryRequired;
#if RTS_CONTACT_DIAGNOSTICS
    public float PenetrationSum;
    public long SolverStartTimestamp;
    public long IterationStartTimestamp;
    public long IterationAccountedStartNanoseconds;
    public long StageStartTimestamp;
    public long StageAccountedStartNanoseconds;
#else
    public float PenetrationSum;
    public long SolverStartTimestamp { get => default; set { } }
    public long IterationStartTimestamp { get => default; set { } }
    public long IterationAccountedStartNanoseconds { get => default; set { } }
    public long StageStartTimestamp { get => default; set { } }
    public long StageAccountedStartNanoseconds { get => default; set { } }
#endif
}

#if RTS_CONTACT_DIAGNOSTICS
public struct ContactSolverIterationTelemetry
{
    public float MaxViolationBeforeSolve;
    public float AverageViolationBeforeSolve;
    public float TotalWallPositionCorrection;
    public float MaxWallPositionCorrection;
    public float TotalContactPositionCorrection;
    public float MaxContactPositionCorrection;
}

public struct JacobiBlockTelemetry
{
    public float TotalPositionCorrection;
    public float MaxPositionCorrection;
    public int NewlyActivatedPairCount;
    public int NewlyCorrectedPairCount;
    public int SelectedPairCount;
    public int SelectedPairOffset;
}
#endif

/// <summary>
/// GS 与 Jacobi 共用的多 Job 接触管线状态。拓扑、生命周期、包络校验与
/// 回退由短 IJob 协调；可按 body/pair 拆分的阶段保持无冲突并行。
/// 只有 XPBD 接触投影按后端分叉。
/// </summary>
}
