using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using UnityEngine;

namespace Entities._Common
{
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup), OrderLast = true)]
    [UpdateAfter(typeof(CalculateFrameDamageSystem))]
    public partial struct ApplyDamageSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NetworkTime>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var currentTick = SystemAPI.GetSingleton<NetworkTime>().ServerTick;
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (var (currentHealthPoint,
                         damageThisTicks,
                         entity) in 
                     SystemAPI.Query<RefRW<HealthPointData>,
                         DynamicBuffer<DamageThisTick>>().
                         WithEntityAccess().WithAll<Simulate>())
            {
               
                if(!damageThisTicks.GetDataAtTick(currentTick,out var damageThisTick))continue;
                if(damageThisTick.Tick!=currentTick)continue;
                currentHealthPoint.ValueRW.CurrentHp -= damageThisTick.Value;
                if (currentHealthPoint.ValueRO.CurrentHp <= 0)
                {
                    ecb.AddComponent(entity,new DestroyEntityTag());
                }
            }
            ecb.Playback(state.EntityManager);
        }
    }
}