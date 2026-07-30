using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;
using RTS.Unit.FlowField.Systems;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 仅做托管调度组合：本身不作为 Job 调度，也不实现算法；
/// 每个调度的阶段把 NativeContainer 作为直接字段携带，
/// Collections Safety 看得到真实容器边界。
/// </summary>
internal partial struct CrowdContactPipelineScheduler
{
    internal const int ParallelBodyBatchSize = 64;
    internal const int SoftPairBatchSize = 64;
    internal const int JacobiPairBatchSize = 64;

    public ContactPipelineConfiguration Configuration;
    public ContactPipelineLifecycleJob Lifecycle;
    internal CrowdObstacleSnapshot Obstacles;
    internal CrowdStepBodyResources Body;
    internal BroadPhaseCandidateBatch BroadPhaseCandidates;
    internal NarrowPhaseConstraintBatch NarrowPhaseConstraints;
    internal BroadPhaseFrameResources BroadPhase;
    internal NativeList<ContactConstraint> PreviousTimestepContactPairs;
    internal ContactClassificationFrameResources Classification;
    internal ContactRepairFrameResources Repair;
    internal ContactCertificateFrameResources Certificate;
    internal CrossFrameCache Persistent;
    internal SoftAvoidanceFrameResources SoftAvoidanceResources;
    internal ConstraintSolverFrameResources Solver;
    internal ContactPipelineExecutionResources Execution;
    internal ContactDiagnosticsFrameResources Diagnostics;
    internal Entity DiagnosticSelectedEntity;
    internal SimulationDebuggerCaptureMask DebuggerCaptureMask;
    internal int MaximumVisualizedPairs;
    internal NativeList<SimulationDebuggerPairSample> DebuggerSelectedPairs;
    internal NativeReference<SimulationDebuggerUnitSample> DebuggerSelectedUnit;
    internal NativeReference<byte> DebuggerSelectedUnitValid;

    private SoftAvoidanceJob CreateSoftAvoidanceJob() =>
        SoftAvoidanceResources.CreateJob(
            Configuration,
            Obstacles,
            Body,
            NarrowPhaseConstraints,
            Solver,
            Execution,
            Diagnostics);

    private ConstraintSolverJob CreateConstraintSolverJob() =>
        Solver.CreateJob(
            Configuration,
            Obstacles,
            Body,
            NarrowPhaseConstraints,
            Execution,
            Diagnostics,
            DiagnosticSelectedEntity,
            DebuggerCaptureMask,
            MaximumVisualizedPairs,
            DebuggerSelectedPairs,
            DebuggerSelectedUnit,
            DebuggerSelectedUnitValid);

}
}

namespace RTS.Unit.FlowField.Systems
{
/// <summary>
/// Crowd 物理管线的单一装配门面。Gameplay 只提交 step 输入和只读环境快照；
/// 资源切片、阶段 Job 与缓存能力均在此处连接。
/// </summary>
internal static class CrowdPhysicsPipelineComposition
{
    public static Unity.Jobs.JobHandle ScheduleStep(
        RTS.Unit.FlowField.Jobs.ContactPipelineConfiguration configuration,
        RTS.Unit.FlowField.Jobs.CrowdObstacleSnapshot obstacles,
        CrowdStepBodyResources body,
        CrowdPhysicsStepInput input,
        BroadPhaseCandidateBatch broadPhaseCandidates,
        NarrowPhaseConstraintBatch narrowPhaseConstraints,
        CrossFrameCache crossFrameCache,
        TimestepCache timestepCache,
        ContactDiagnosticsFrameResources diagnostics,
        Unity.Entities.Entity diagnosticSelectedEntity,
        RTS.Unit.FlowField.Diagnostics.SimulationDebuggerCaptureMask captureMask,
        int maximumVisualizedPairs,
        Unity.Collections.NativeList<
            RTS.Unit.FlowField.Diagnostics.SimulationDebuggerPairSample>
            debuggerSelectedPairs,
        Unity.Collections.NativeReference<
            RTS.Unit.FlowField.Diagnostics.SimulationDebuggerUnitSample>
            debuggerSelectedUnit,
        Unity.Collections.NativeReference<byte> debuggerSelectedUnitValid,
        Unity.Jobs.JobHandle dependency)
    {
        if (!input.Bodies.IsCreated ||
            input.Bodies.Length != body.Bodies.Length)
            throw new System.ArgumentException(
                "CrowdPhysicsStepInput must match the internal body view.");

        var lifecycle = crossFrameCache.CreateLifecycleJob(
            configuration,
            timestepCache.Execution,
            timestepCache.Solver,
            diagnostics,
            debuggerSelectedPairs);

        var scheduler =
            new RTS.Unit.FlowField.Jobs.CrowdContactPipelineScheduler
        {
            Configuration = configuration,
            Lifecycle = lifecycle,
            Obstacles = obstacles,
            Body = body,
            BroadPhaseCandidates = broadPhaseCandidates,
            NarrowPhaseConstraints = narrowPhaseConstraints,
            BroadPhase = timestepCache.BroadPhase,
            PreviousTimestepContactPairs =
                timestepCache.Products.PreviousTimestepContactPairs,
            Classification = timestepCache.Classification,
            Repair = timestepCache.Repair,
            Certificate = timestepCache.Certificate,
            Persistent = crossFrameCache,
            SoftAvoidanceResources = timestepCache.SoftAvoidance,
            Solver = timestepCache.Solver,
            Execution = timestepCache.Execution,
            Diagnostics = diagnostics,
            DiagnosticSelectedEntity = diagnosticSelectedEntity,
            DebuggerCaptureMask = captureMask,
            MaximumVisualizedPairs = maximumVisualizedPairs,
            DebuggerSelectedPairs = debuggerSelectedPairs,
            DebuggerSelectedUnit = debuggerSelectedUnit,
            DebuggerSelectedUnitValid = debuggerSelectedUnitValid
        };

#if RTS_CONTACT_DIAGNOSTICS
        return scheduler.ScheduleParallelStages(
            timestepCache.Execution.PipelineRuntimeState,
            timestepCache.Execution.SolverIterationState,
            timestepCache.Execution.JacobiBlockStatistics,
            dependency);
#else
        return scheduler.ScheduleParallelStages(
            timestepCache.Execution.PipelineRuntimeState,
            dependency);
#endif
    }
}
}
