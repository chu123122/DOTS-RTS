using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Navigation movement policy. SteeringVelocityError preserves the current controller
/// math: the motion integrator clamps it by Body.MaxAcceleration before integration.
/// </summary>
public struct CrowdMotionIntent
{
    public float3 PreferredVelocity;
    public float3 SteeringVelocityError;
}
}
