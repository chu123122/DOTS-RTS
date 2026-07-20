using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using 通用;

/// <summary>
/// 合并各类力，通过半隐式欧拉先更新速度，再由新速度生成预测位置。
/// 所有单位的预测结果同时写入快照，供后续约束阶段统一读取。
/// </summary>
[BurstCompile]
public partial struct IntegrateFlowForcesJob : IJobEntity
{
    public float DeltaTime;
    public float3 GridOrigin;
    public float CellRadius;
    public NativeArray<FlowMovementFrameState> States;

    public void Execute([EntityIndexInQuery] int entityIndex)
    {
        FlowMovementFrameState state = States[entityIndex];
        if (!state.IsInsideGrid)
        {
            state.IntegratedVelocity = float3.zero;
            state.PredictedPosition = state.CurrentPosition;
            States[entityIndex] = state;
            return;
        }

        // 独立力与软避让力只在此处汇合，约束阶段不再读取力。
        float3 totalForce = state.IndependentForce + state.SoftAvoidanceForce;
        if (state.Cell.Cost == 0 && math.lengthsq(totalForce) < 0.1f)
        {
            // 单位位于障碍格且合力过小时，补充逃逸力以避免静止在格中心。
            float3 cellCenter = GridOrigin + new float3(
                state.CellPosition.x * CellRadius * 2 + CellRadius,
                state.CurrentPosition.y,
                state.CellPosition.y * CellRadius * 2 + CellRadius);
            float3 escapeDirection = math.normalize(state.CurrentPosition - cellCenter);
            if (math.lengthsq(escapeDirection) < 0.001f)
                escapeDirection = new float3(1, 0, 0);
            totalForce += escapeDirection * state.MoveSpeed * 5.0f;
        }

        if (math.length(totalForce) > state.MaxForce)
            totalForce = math.normalize(totalForce) * state.MaxForce;

        // 半隐式欧拉：v(t+dt) = v(t) + a*dt，x* = x(t) + v(t+dt)*dt。
        float3 integratedVelocity = state.CurrentVelocity + totalForce * DeltaTime;
        if (state.IsSettled)
        {
            integratedVelocity *= math.pow(0.8f, DeltaTime * 60f);
        }

        if (math.length(integratedVelocity) > state.MoveSpeed)
            integratedVelocity = math.normalize(integratedVelocity) * state.MoveSpeed;

        float3 predictedPosition = state.CurrentPosition + integratedVelocity * DeltaTime;
        predictedPosition.y = state.CurrentPosition.y;

        state.IntegratedVelocity = integratedVelocity;
        state.PredictedPosition = predictedPosition;
        States[entityIndex] = state;
    }
}
