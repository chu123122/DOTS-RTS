using Unity.Entities;
using RTS.Unit.FlowField.Systems;

namespace RTS.Gameplay.Physics
{
/// <summary>
/// 只读取本帧 FixedStep 已发布的 PhysicsWorld。该组完成后 Crowd 才提交下一
/// step Transform，因此 ECS Transform、PhysicsWorld 与 ProxyVersion 属于同一版本。
/// </summary>
[WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateAfter(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(CrowdPhysicsSystemGroup))]
public partial class CrowdQuerySystemGroup : ComponentSystemGroup
{
}
}
