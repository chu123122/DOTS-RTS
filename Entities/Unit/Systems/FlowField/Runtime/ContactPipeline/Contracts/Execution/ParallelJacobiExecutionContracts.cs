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
#else
    public float PenetrationSum;
    public long SolverStartTimestamp { get => default; set { } }
    public long IterationStartTimestamp { get => default; set { } }
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
/// Multi-job Jacobi path. The topology, lifecycle, envelope validation and fallback
/// remain serial coordination stages; pair evaluation and body gather/apply are
/// conflict-free parallel stages. Selected-pair debugger capture uses pair-exclusive
/// scratch slots and deterministic compaction without changing the solver backend.
/// </summary>
}
