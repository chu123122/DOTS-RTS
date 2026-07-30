using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 单个 crowd 步开始时从 ECS 采集的不可变 body 事实。导航、交互认证与求解器状态分别存放在独立数组中。
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
    public uint ShapeVersion;
    public byte IsInsideSimulationDomain;
}

/// <summary>
/// Unit/Navigation Adapter 提交给 Crowd Physics 的逐 body 不可变输入。
/// 不暴露 FlowField cell、integration value、pair、cache 或 solver 状态。
/// </summary>
public struct CrowdPhysicsBodyInput
{
    public Entity StableId;
    public float3 Position;
    public quaternion Rotation;
    public float3 Velocity;
    public float3 PreferredVelocity;
    public float3 SteeringVelocityError;
    public float MoveSpeed;
    public float MaxAcceleration;
    public float InverseMass;
    public float Radius;
    public uint ShapeVersion;
    public byte IsInsideSimulationDomain;
    public byte IsSettled;
}
}
