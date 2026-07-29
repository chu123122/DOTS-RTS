using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
/// <summary>帧级协调状态，非游戏或候选数据。</summary>
internal struct ContactPipelineExecutionResources
{
    public NativeReference<ContactPipelineExecutionState> PipelineRuntimeState;
#if RTS_CONTACT_DIAGNOSTICS
    public NativeReference<ContactSolverIterationTelemetry> SolverIterationState;
    public NativeList<JacobiBlockTelemetry> JacobiBlockStatistics;
#endif

    public static ContactPipelineExecutionResources Create(int unitCount)
    {
        return new ContactPipelineExecutionResources
        {
            PipelineRuntimeState = new NativeReference<ContactPipelineExecutionState>(Allocator.TempJob),
#if RTS_CONTACT_DIAGNOSTICS
            SolverIterationState = new NativeReference<ContactSolverIterationTelemetry>(Allocator.TempJob),
            JacobiBlockStatistics = new NativeList<JacobiBlockTelemetry>(math.max((unitCount * 4 + 63) / 64, 1), Allocator.TempJob),
#endif
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = finalReader;
        if (PipelineRuntimeState.IsCreated)
            combined = JobHandle.CombineDependencies(combined, PipelineRuntimeState.Dispose(finalReader));
#if RTS_CONTACT_DIAGNOSTICS
        if (SolverIterationState.IsCreated)
            combined = JobHandle.CombineDependencies(combined, SolverIterationState.Dispose(finalReader));
        if (JacobiBlockStatistics.IsCreated)
            combined = JobHandle.CombineDependencies(combined, JacobiBlockStatistics.Dispose(finalReader));
#endif
        return combined;
    }
}
}
