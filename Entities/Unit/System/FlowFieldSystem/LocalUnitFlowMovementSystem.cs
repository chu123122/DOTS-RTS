using Unity.Entities;
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