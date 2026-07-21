using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using UnityEngine;

namespace Entities._Common
{
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup), OrderLast = true)]
    [UpdateAfter(typeof(CalculateFrameDamageSystem))]
    public partial struct ApplyDamageSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
        }

        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (var (currentHealthPoint,
                         damageThisTicks,
                         entity) in 
                     SystemAPI.Query<RefRW<HealthPointData>,
                         DynamicBuffer<DamageThisTick>>().
                         WithEntityAccess())
            {
               
                if (damageThisTicks.IsEmpty) continue;
                DamageThisTick damageThisTick = damageThisTicks[0];
                currentHealthPoint.ValueRW.CurrentHp -= damageThisTick.Value;
                damageThisTicks.Clear();
                if (currentHealthPoint.ValueRO.CurrentHp <= 0)
                {
                    ecb.AddComponent(entity,new DestroyEntityTag());
                }
            }
            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }
    }
}
