using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 每个 index 只有一个写者，因此可与串行 Gauss-Seidel 接触求解分离并行。
/// </summary>
[BurstCompile]
public struct InitializeFlowMovementSolverStateJob : IJobParallelFor
{
    public NativeArray<FlowMovementFrameState> States;

    public void Execute(int index)
    {
        FlowMovementFrameState state = States[index];
        state.IntegratedVelocity = state.IsInsideGrid
            ? state.CurrentVelocity
            : float3.zero;
        state.StartPosition = state.CurrentPosition;
        state.UnconstrainedPredictedPosition = state.CurrentPosition;
        state.PredictedPosition = state.CurrentPosition;
        state.PreviousSubstepPosition = state.CurrentPosition;
        state.ContactPositionCorrection = float3.zero;
        state.WallPositionCorrection = float3.zero;
        state.SoftAvoidanceVelocity = float3.zero;
        state.WallAvoidanceVelocity = float3.zero;
        state.SoftAvoidanceNeighborCount = 0;
        state.TimestepContactCorrection = float3.zero;
        state.TimestepWallCorrection = float3.zero;
        States[index] = state;
    }
}
}
