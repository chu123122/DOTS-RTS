using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 交互认证器消费的、timestep 范围内的权威运动证据。这不是持久候选状态，下游消费者只可上报逃逸。
/// </summary>
public struct CrowdMotionEvidence
{
    public float3 TrajectoryStart;
    public float3 BaselineEnd;
    public float2 ContactEnvelopeMin;
    public float2 ContactEnvelopeMax;
    public float2 InteractionEnvelopeMin;
    public float2 InteractionEnvelopeMax;
    public float3 ContactCorrection;
    public float3 WallCorrection;
    public uint MotionVersion;
    public byte EnvelopeEscaped;
}
}
