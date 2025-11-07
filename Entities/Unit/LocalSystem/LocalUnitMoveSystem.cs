using _RePlaySystem.Base;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Transforms;
using UnityEngine;
using 通用;

[WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation)]
public partial class LocalUnitMoveSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireForUpdate<LocalInstance>();
    }

    protected override void OnUpdate()
    {
        var deltaTime = SystemAPI.Time.DeltaTime;
        foreach (var (transform, 
                     movePosition, 
                     moveSpeed,_,entity) in
                 SystemAPI.Query<RefRW<LocalTransform>,
                         RefRO<UnitMoveTargetPosition>,
                         RefRO<UnitMoveSpeed>,
                         RefRO<LocalInstance>>()
                     .WithEntityAccess())
        {
       
            float3 moveTarget = movePosition.ValueRO.Value;
            moveTarget.y = transform.ValueRO.Position.y;
            if (math.distance(transform.ValueRO.Position, moveTarget) < 0.1f) continue;

            float3 moveDirection = math.normalize(moveTarget - transform.ValueRO.Position);
            float3 moveVector = moveDirection * moveSpeed.ValueRO.Value * deltaTime;
            transform.ValueRW.Position += moveVector;
            transform.ValueRW.Rotation = quaternion.LookRotationSafe(moveDirection, math.up());
        }
    }
}