using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Unit/Navigation 向 Crowd Physics 提交的唯一只读产品。
/// NavigationState 是管线内部兼容状态，不属于这个接口。
/// </summary>
public readonly struct CrowdPhysicsStepInput
{
    [ReadOnly] public readonly NativeArray<CrowdPhysicsBodyInput> Bodies;

    public CrowdPhysicsStepInput(NativeArray<CrowdPhysicsBodyInput> bodies)
    {
        Bodies = bodies;
    }
}

/// <summary>Solver 写入、Unit Writeback 只读的唯一输出产品。</summary>
public readonly struct CrowdPhysicsStepOutput
{
    public readonly NativeArray<CrowdBodyResult>.ReadOnly Bodies;

    public CrowdPhysicsStepOutput(
        NativeArray<CrowdBodyResult>.ReadOnly bodies)
    {
        Bodies = bodies;
    }
}

}

namespace RTS.Unit.FlowField.Systems
{
internal struct CrowdStepBodyResources
{
    public NativeArray<CrowdPhysicsBodyInput> StepInputs;
    public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdNavigationState> NavigationStates;
    public NativeArray<CrowdMotionIntent> MotionIntents;
    public NativeArray<CrowdMotionEvidence> MotionEvidence;
    public NativeArray<CrowdAvoidanceState> AvoidanceStates;
    public NativeArray<CrowdSolverBodyState> StepStates;
    public NativeList<CrowdBodyResult> Results;
    public NativeParallelHashMap<Entity, int> CurrentBodyIndexByEntity;

    public CrowdPhysicsStepInput Input =>
        new CrowdPhysicsStepInput(StepInputs);

    public CrowdPhysicsStepOutput Output =>
        new CrowdPhysicsStepOutput(Results.AsReadOnly());

    public static CrowdStepBodyResources Create(int unitCount)
    {
        return new CrowdStepBodyResources
        {
            StepInputs = new NativeArray<CrowdPhysicsBodyInput>(
                unitCount,
                Allocator.TempJob,
                NativeArrayOptions.UninitializedMemory),
            Bodies = new NativeArray<CrowdBodySnapshot>(unitCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            NavigationStates = new NativeArray<CrowdNavigationState>(unitCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            MotionIntents = new NativeArray<CrowdMotionIntent>(unitCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            MotionEvidence = new NativeArray<CrowdMotionEvidence>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            AvoidanceStates = new NativeArray<CrowdAvoidanceState>(
                unitCount,
                Allocator.TempJob,
                NativeArrayOptions.ClearMemory),
            StepStates = new NativeArray<CrowdSolverBodyState>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            Results = CreateResults(unitCount),
            CurrentBodyIndexByEntity =
                new NativeParallelHashMap<Entity, int>(
                    math.max(unitCount, 1), Allocator.TempJob)
        };
    }

    private static NativeList<CrowdBodyResult> CreateResults(int unitCount)
    {
        var results =
            new NativeList<CrowdBodyResult>(unitCount, Allocator.TempJob);
        results.ResizeUninitialized(unitCount);
        return results;
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = finalReader;
        combined = JobHandle.CombineDependencies(
            combined, StepInputs.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, Bodies.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, NavigationStates.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, MotionIntents.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, MotionEvidence.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, AvoidanceStates.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, StepStates.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, Results.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(
            combined, CurrentBodyIndexByEntity.Dispose(finalReader));
        return combined;
    }
}
}
