using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Immutable body facts captured from ECS at the beginning of one crowd step.
/// Navigation, interaction certification and solver state live in separate arrays.
/// </summary>
public struct CrowdBodySnapshot
{
    public Entity Entity;
    public float3 Position;
    public quaternion Rotation;
    public float3 Velocity;
    public float MoveSpeed;
    public float MaxAcceleration;
    public float InverseMass;
    public float Radius;
    public byte IsInsideSimulationDomain;
}
}
