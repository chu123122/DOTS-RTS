using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{

/// <summary>
/// 单位在一次移动流水线中的临时状态。
/// 该数据仅存活一帧，由各 Job 按顺序补全，不会写入 ECS 组件长期保存。
/// </summary>
public struct FlowMovementFrameState
{
    // 阶段开始时的单位状态，后续阶段以此为统一基线。
    public Entity Entity;
    public float3 CurrentPosition;
    public quaternion CurrentRotation;
    public float3 CurrentVelocity;
    public float MoveSpeed;
    public float MaxForce;
    public float InverseMass;
    public float Radius;

    // 当前 timestep 的接触预测基线。substep 只消费由这条完整轨迹构建的 ContactSet。
    public float3 TimestepStartPosition;
    public float3 TimestepPredictedPosition;
    public float2 TimestepEnvelopeMin;
    public float2 TimestepEnvelopeMax;
    // B 层 InteractionSet 的完整包络，除接触路径外还覆盖 Soft/RVO horizon。
    // 它与接触包络分开，避免宽 RVO 包络掩盖 XPBD 接触逃逸。
    public float2 TimestepInteractionEnvelopeMin;
    public float2 TimestepInteractionEnvelopeMax;
    public byte TimestepEscaped;
    public float3 TimestepContactCorrection;
    public float3 TimestepWallCorrection;

    // 当前所在流场格及带滞回的到达状态，由独立力阶段计算。
    public int2 CellPosition;
    public FlowFieldCell Cell;
    public bool IsSettled;
    public bool IsInsideGrid;

    // 按流水线依次生成的中间结果。
    public float3 IndependentForce;
    public float3 SoftAvoidanceVelocity;
    public float3 WallAvoidanceVelocity;
    public int SoftAvoidanceNeighborCount;
    public float3 BasePredictedVelocity;
    public float3 IntegratedVelocity;
    public float3 StartPosition;
    public float3 UnconstrainedPredictedPosition;
    public float3 VelocityBeforeContact;
    public float3 PredictedPosition;
    public float3 PreviousSubstepPosition;
    public float3 ContactPositionCorrection;
    public float3 WallPositionCorrection;
}
}
