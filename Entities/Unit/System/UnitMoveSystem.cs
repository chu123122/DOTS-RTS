using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Transforms;
using UnityEngine;
using 通用;

[UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
public partial class UnitMoveSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireForUpdate<NetworkStreamInGame>();
    }

    protected override void OnUpdate()
    {
        var deltaTime = SystemAPI.Time.DeltaTime;
        foreach (var (transform, 
                     movePosition, 
                     moveSpeed,
                     entity) in
                 SystemAPI.Query<RefRW<LocalTransform>,
                         RefRW<UnitMoveTargetPosition>,
                         RefRO<UnitMoveSpeed>>()
                     .WithEntityAccess().WithAll<Simulate>())
        {
            float3 moveTarget = movePosition.ValueRO.Value;
            if (movePosition.ValueRO.GetMoveInput)
            {
                moveTarget.y = transform.ValueRO.Position.y;
                float3 moveDirection = math.normalize(moveTarget - transform.ValueRO.Position);
                float3 moveVector = moveDirection * moveSpeed.ValueRO.Value * deltaTime;
                transform.ValueRW.Position += moveVector;
                transform.ValueRW.Rotation = quaternion.LookRotationSafe(moveDirection, math.up());
                if (math.distance(transform.ValueRO.Position, moveTarget) < 0.1f)
                {
                    movePosition.ValueRW.GetMoveInput = false;
                }
            }
            /*else if (trackEntity.ValueRO.Entity != Entity.Null && !movePosition.ValueRO.GetMoveInput)
            {
                if (math.distance(transform.ValueRO.Position, moveTarget) < 0.5f)continue;
                
                moveTarget = EntityManager.GetComponentData<LocalTransform>(trackEntity.ValueRO.Entity).Position;
                moveTarget.y = transform.ValueRO.Position.y;
                float3 moveDirection = math.normalize(moveTarget - transform.ValueRO.Position);
                float3 moveVector = moveDirection * moveSpeed.ValueRO.Value * deltaTime;
                transform.ValueRW.Position += moveVector;
                transform.ValueRW.Rotation = quaternion.LookRotationSafe(moveDirection, math.up());
                
            }*/
           
        }
        
    }
}