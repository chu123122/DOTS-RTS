using Unity.Entities;
using Unity.NetCode;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{

[UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
public partial class NetCodeUnitFlowMovementSystem : BaseFlowMovementSystem
{
    protected override void OnCreate()
    {
        base.OnCreate();
        RequireForUpdate<NetworkStreamInGame>();
    }
}
}
