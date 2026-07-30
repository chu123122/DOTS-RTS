using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 用不可变 body 快照初始化 timestep/substep 拥有的运动状态。每个索引一个写者，可并行。
/// </summary>
[BurstCompile]
internal struct InitializeCrowdStepStateJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdMotionEvidence> MotionEvidence;
    public NativeArray<CrowdSolverBodyState> StepStates;

    public void Execute(int index)
    {
        CrowdBodySnapshot body = Bodies[index];
        CrowdSolverBodyState step = new CrowdSolverBodyState
        {
            IntegratedVelocity = body.IsInsideSimulationDomain != 0
                ? body.Velocity
                : float3.zero,
            SubstepStartPosition = body.Position,
            UnconstrainedPosition = body.Position,
            SolvedPosition = body.Position,
            PreviousSubstepPosition = body.Position
        };
        MotionEvidence[index] = default;
        StepStates[index] = step;
    }
}
}
