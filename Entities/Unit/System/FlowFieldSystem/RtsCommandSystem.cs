using Entities.Unit.System.FlowFieldSystem;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;

[UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
public partial class RtsCommandSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireForUpdate<FlowFieldGlobalTarget>();
        RequireForUpdate<MoveOrder>();
        RequireForUpdate<NetworkTime>();
    }

    protected override void OnUpdate()
    {
        var networkTime = SystemAPI.GetSingleton<NetworkTime>();
        if (!networkTime.IsFirstTimeFullyPredictingTick) return;

        var gridEntity = SystemAPI.GetSingletonEntity<FlowFieldGlobalTarget>();
        var currentTarget = SystemAPI.GetComponent<FlowFieldGlobalTarget>(gridEntity);
        var moveOrder = EntityManager.GetComponentData<MoveOrder>(gridEntity);
        EntityManager.SetComponentEnabled<MoveOrder>(gridEntity, false);

        if (math.distance(moveOrder.TargetPosition, currentTarget.TargetPosition) < 0.1f)
            return;

        SystemAPI.SetComponent(gridEntity, new FlowFieldGlobalTarget
        {
            TargetPosition = moveOrder.TargetPosition
        });
        RecalculateFlowFieldTag request =
            EntityManager.GetComponentData<RecalculateFlowFieldTag>(gridEntity);
        request.RequestVersion++;
        EntityManager.SetComponentData(gridEntity, request);
        EntityManager.SetComponentEnabled<RecalculateFlowFieldTag>(gridEntity, true);
    }
}
