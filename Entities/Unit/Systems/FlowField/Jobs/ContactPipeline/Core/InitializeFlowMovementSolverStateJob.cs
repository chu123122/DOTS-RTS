using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Initializes timestep/substep-owned motion state from immutable body snapshots.
/// Each index has one writer and can run in parallel.
/// </summary>
[BurstCompile]
public struct InitializeCrowdStepStateJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    public NativeArray<CrowdMotionEvidence> MotionEvidence;
    public NativeArray<CrowdBodyStepState> StepStates;

    public void Execute(int index)
    {
        CrowdBodySnapshot body = Bodies[index];
        CrowdBodyStepState step = new CrowdBodyStepState
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
