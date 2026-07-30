using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Soft Avoidance 阶段的唯一可变输出；Solver 只读消费。
/// </summary>
public struct CrowdAvoidanceState
{
    public float3 SoftVelocity;
    public float3 WallVelocity;
    public int NeighborCount;
}
}
