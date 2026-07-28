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

public struct ParallelJacobiExecutionState
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
public struct ParallelJacobiIterationTelemetry
{
    public float MaxViolationBeforeSolve;
    public float AverageViolationBeforeSolve;
    public float TotalWallPositionCorrection;
    public float MaxWallPositionCorrection;
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
/// 多 Job 的 Jacobi 路径。拓扑、生命周期、包络校验与回退仍是串行协调阶段；对评估与 body 收集/应用是无冲突并行阶段。
/// 已选对调试器捕获使用对独占的临时槽位和确定性压缩，不改变求解器后端。
/// </summary>
}
