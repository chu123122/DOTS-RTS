using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 求解后的解耦产物，供 ECS 回写与单步结束后的观察者使用。
/// </summary>
public struct CrowdBodyResult
{
    public float3 Position;
    public float3 Velocity;
    public quaternion Rotation;
}
}
