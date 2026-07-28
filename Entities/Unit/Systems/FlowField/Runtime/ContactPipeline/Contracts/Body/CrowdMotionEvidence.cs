using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Timestep-scoped authoritative motion evidence consumed by the interaction certifier.
/// This is not persistent candidate state and lower consumers may only report escapes.
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
