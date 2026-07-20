using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using 通用;

/// <summary>
/// 将已经完成单位/墙壁约束迭代的最终位置与回算速度写回单位。
/// </summary>
[BurstCompile]
public partial struct ApplyFlowMovementJob : IJobEntity
{
    public float DeltaTime;
    [ReadOnly] public NativeArray<FlowMovementFrameState> States;

    public void Execute(
        [EntityIndexInQuery] int entityIndex,
        ref LocalTransform transform,
        ref Velocity velocity)
    {
        FlowMovementFrameState state = States[entityIndex];
        if (!state.IsInsideGrid)
        {
            velocity.Value = float3.zero;
            return;
        }

        float3 integratedVelocity = state.IntegratedVelocity;
        bool isHardColliding =
            math.lengthsq(state.ContactPositionCorrection) > 0.0001f ||
            math.lengthsq(state.WallPositionCorrection) > 0.0001f;

        // 停车与位置约束解耦：到达区域内速度足够低时直接归零，
        // 即使仍有穿透修正，也只应用位置投影，不重新产生运动速度。
        if (state.IsSettled && math.lengthsq(integratedVelocity) <= 0.005f)
            integratedVelocity = float3.zero;

        bool shouldMove = math.lengthsq(integratedVelocity) > 0.005f || isHardColliding;
        if (shouldMove)
        {
            float3 newPosition = state.PredictedPosition;
            newPosition.y = state.CurrentPosition.y;
            transform.Position = newPosition;

            integratedVelocity.y = 0;

            if (math.lengthsq(integratedVelocity) > 0.01f)
            {
                quaternion targetRotation = quaternion.LookRotationSafe(math.normalize(integratedVelocity), math.up());
                transform.Rotation = math.slerp(state.CurrentRotation, targetRotation, DeltaTime * 10.0f);
            }
        }
        else if (state.IsSettled)
        {
            integratedVelocity = float3.zero;
        }

        velocity.Value = integratedVelocity;
    }
}
