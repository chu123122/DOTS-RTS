using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Detached post-solve product consumed by ECS writeback and completed-step observers.
/// </summary>
public struct CrowdBodyResult
{
    public float3 Position;
    public float3 Velocity;
    public quaternion Rotation;
}
}
