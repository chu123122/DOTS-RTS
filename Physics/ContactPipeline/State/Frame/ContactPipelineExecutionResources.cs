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

/// <summary>
/// 一个物理 timestep 的唯一缓存所有者。内部资源可跨 substep，
/// 但不会跨帧；具体阶段只能取得自己的能力切片。
/// </summary>
internal struct TimestepCache
{
    public BroadPhaseFrameResources BroadPhase;
    public ContactProductFrameResources Products;
    public ContactClassificationFrameResources Classification;
    public ContactRepairFrameResources Repair;
    public ContactCertificateFrameResources Certificate;
    public SoftAvoidanceFrameResources SoftAvoidance;
    public ConstraintSolverFrameResources Solver;
    public ContactPipelineExecutionResources Execution;

    public static TimestepCache Create(int unitCount)
    {
        return new TimestepCache
        {
            BroadPhase = BroadPhaseFrameResources.Create(unitCount),
            Products = ContactProductFrameResources.Create(unitCount),
            Classification =
                ContactClassificationFrameResources.Create(unitCount),
            Repair = ContactRepairFrameResources.Create(unitCount),
            Certificate = ContactCertificateFrameResources.Create(unitCount),
            SoftAvoidance = SoftAvoidanceFrameResources.Create(unitCount),
            Solver = ConstraintSolverFrameResources.Create(unitCount),
            Execution = ContactPipelineExecutionResources.Create(unitCount)
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = BroadPhase.Dispose(finalReader);
        combined = JobHandle.CombineDependencies(
            combined, Products.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, Classification.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, Repair.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, Certificate.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, SoftAvoidance.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, Solver.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, Execution.Dispose(finalReader));
        return combined;
    }
}
}
