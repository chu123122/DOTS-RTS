using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 导航移动策略。SteeringVelocityError 保留当前控制器的数学约定：运动积分器在积分前会用 Body.MaxAcceleration 截断它。
/// </summary>
public struct CrowdMotionIntent
{
    public float3 PreferredVelocity;
    public float3 SteeringVelocityError;
}
}
