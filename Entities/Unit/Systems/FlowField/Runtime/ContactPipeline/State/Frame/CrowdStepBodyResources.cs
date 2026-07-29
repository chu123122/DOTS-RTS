using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
internal struct CrowdStepBodyResources
{
    public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdNavigationState> NavigationStates;
    public NativeArray<CrowdMotionIntent> MotionIntents;
    public NativeArray<CrowdMotionEvidence> MotionEvidence;
    public NativeArray<CrowdBodyStepState> StepStates;
    public NativeArray<CrowdBodyResult> Results;
    public NativeArray<float2> CollisionFootprints;

    public static CrowdStepBodyResources Create(int unitCount)
    {
        return new CrowdStepBodyResources
        {
            Bodies = new NativeArray<CrowdBodySnapshot>(unitCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            NavigationStates = new NativeArray<CrowdNavigationState>(unitCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            MotionIntents = new NativeArray<CrowdMotionIntent>(unitCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            MotionEvidence = new NativeArray<CrowdMotionEvidence>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            StepStates = new NativeArray<CrowdBodyStepState>(unitCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
            Results = new NativeArray<CrowdBodyResult>(unitCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory),
            CollisionFootprints = new NativeArray<float2>(unitCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory)
        };
    }

    public JobHandle Dispose(JobHandle finalReader)
    {
        JobHandle combined = finalReader;
        combined = JobHandle.CombineDependencies(combined, Bodies.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, NavigationStates.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, MotionIntents.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, MotionEvidence.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, StepStates.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, Results.Dispose(finalReader));
        combined = JobHandle.CombineDependencies(combined, CollisionFootprints.Dispose(finalReader));
        return combined;
    }
}
}
