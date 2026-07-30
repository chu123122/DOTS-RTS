using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 在写回 ECS 前，把最终求解结果从可变子步状态中分离出来。
/// </summary>
[BurstCompile]
internal struct BuildCrowdBodyResultsJob : IJobParallelFor
{
    public float DeltaTime;
    [ReadOnly] public NativeArray<CrowdBodySnapshot> Bodies;
    [ReadOnly] public NativeArray<CrowdNavigationState> NavigationStates;
    [ReadOnly] public NativeArray<CrowdSolverBodyState> StepStates;
    public NativeArray<CrowdBodyResult> Results;

    public void Execute(int index)
    {
        CrowdBodySnapshot body = Bodies[index];
        CrowdNavigationState navigation = NavigationStates[index];
        CrowdSolverBodyState step = StepStates[index];

        float3 resultPosition = body.Position;
        float3 resultVelocity = body.IsInsideSimulationDomain != 0
            ? step.IntegratedVelocity
            : float3.zero;
        quaternion resultRotation = body.Rotation;

        if (body.IsInsideSimulationDomain != 0)
        {
            // SolvedPosition is the authoritative result of the complete
            // timestep. Per-substep correction fields may be zero in the final
            // substep even when an earlier substep corrected the body, so they
            // must not gate publication of the solved position.
            resultPosition = step.SolvedPosition;
            resultPosition.y = body.Position.y;
            resultVelocity.y = 0f;

            if (navigation.IsSettled != 0 &&
                math.lengthsq(resultVelocity) <= 0.005f)
            {
                resultVelocity = float3.zero;
            }

            if (math.lengthsq(resultVelocity) > 0.01f)
            {
                quaternion targetRotation = quaternion.LookRotationSafe(
                    math.normalize(resultVelocity),
                    math.up());
                resultRotation = math.slerp(
                    body.Rotation,
                    targetRotation,
                    DeltaTime * 10f);
            }
        }
        else if (navigation.IsSettled != 0)
        {
            resultVelocity = float3.zero;
        }

        Results[index] = new CrowdBodyResult
        {
            Position = resultPosition,
            Velocity = resultVelocity,
            Rotation = resultRotation
        };
    }
}
}