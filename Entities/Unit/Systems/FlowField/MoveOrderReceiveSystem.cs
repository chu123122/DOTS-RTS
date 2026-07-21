using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;
using RTS.Unit.FlowField.Jobs;

namespace RTS.Unit.FlowField.Systems
{

[WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
[UpdateInGroup(typeof(PredictedSimulationSystemGroup), OrderFirst = true)]
[UpdateBefore(typeof(RtsCommandSystem))]
public partial struct MoveOrderReceiveSystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<FlowFieldGlobalTarget>();
    }

    public void OnUpdate(ref SystemState state)
    {
        bool hasMoveOrder = false;
        float3 targetPosition = float3.zero;
        var ecb = new EntityCommandBuffer(Allocator.Temp);

        foreach (var (request, _, requestEntity) in
                 SystemAPI.Query<RefRO<RequestMoveOrderRPC>, RefRO<ReceiveRpcCommandRequest>>()
                     .WithEntityAccess())
        {
            targetPosition = request.ValueRO.TargetPosition;
            hasMoveOrder = true;
            ecb.DestroyEntity(requestEntity);
        }

        if (hasMoveOrder)
        {
            Entity flowFieldEntity = SystemAPI.GetSingletonEntity<FlowFieldGlobalTarget>();
            state.EntityManager.SetComponentData(flowFieldEntity, new MoveOrder
            {
                TargetPosition = targetPosition
            });
            state.EntityManager.SetComponentEnabled<MoveOrder>(flowFieldEntity, true);
        }

        ecb.Playback(state.EntityManager);
        ecb.Dispose();
    }
}
}
