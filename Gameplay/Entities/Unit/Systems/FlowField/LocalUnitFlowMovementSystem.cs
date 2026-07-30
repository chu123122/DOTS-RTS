using Unity.Entities;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
using _RePlaySystem.Base; 

/// <summary>
/// 同一 ECS World 内的 Crowd Physics 独立调度阶段。
/// 当前保持在 FlowField bake 之后，以保留既有 timestep 行为。
/// </summary>
[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateAfter(typeof(FlowFieldBakeSystem))]
public partial class CrowdPhysicsSystemGroup : ComponentSystemGroup
{
}

[UpdateInGroup(typeof(CrowdPhysicsSystemGroup))]
public partial class LocalUnitFlowMovementSystem : BaseFlowMovementSystem
{
    protected override void OnCreate()
    {
        base.OnCreate();
        RequireForUpdate<LocalInstance>(); 
    }
}
}
