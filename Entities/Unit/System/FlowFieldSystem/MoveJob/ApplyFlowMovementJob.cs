using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using 通用;

/// <summary>
/// 将预测位置和位置约束修正写回单位，并按积分速度更新朝向。
/// 当前只应用位置投影，约束产生的位移不会反推并修正 Velocity。
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
        float3 positionCorrection = state.PositionCorrection;
        bool isHardColliding = math.lengthsq(positionCorrection) > 0.0001f;

        // 停车与位置约束解耦：到达区域内速度足够低时直接归零，
        // 即使仍有穿透修正，也只应用位置投影，不重新产生运动速度。
        if (state.IsSettled && math.lengthsq(integratedVelocity) <= 0.005f)
            integratedVelocity = float3.zero;

        bool shouldMove = math.lengthsq(integratedVelocity) > 0.005f || isHardColliding;
        if (shouldMove)
        {
            float3 newPosition = state.PredictedPosition;
            if (isHardColliding)
            {
                // 限制单帧最大投影距离，避免深度穿透时出现明显位置跳变。
                const float maxCorrectionPerFrame = 0.15f;
                if (math.lengthsq(positionCorrection) > maxCorrectionPerFrame * maxCorrectionPerFrame)
                    positionCorrection = math.normalize(positionCorrection) * maxCorrectionPerFrame;

                newPosition += positionCorrection;
            }

            newPosition.y = state.CurrentPosition.y;
            transform.Position = newPosition;

            // 速度保持力积分结果；若以后需要物理一致性，应增加投影后的速度回写。
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
