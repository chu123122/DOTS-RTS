using Unity.Entities;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{
using _RePlaySystem.Base; 

[UpdateInGroup(typeof(SimulationSystemGroup))] 
public partial class LocalUnitFlowMovementSystem : BaseFlowMovementSystem
{
    protected override void OnCreate()
    {
        base.OnCreate();
        RequireForUpdate<LocalInstance>(); 
    }
}
}
