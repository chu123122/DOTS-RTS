using Entities._Common;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Physics;
using Unity.Physics.Systems;
using UnityEngine;
using RTS.Unit.Components;

namespace TMG.NFE_Tutorial 
{
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateAfter(typeof(PhysicsSimulationGroup))]
    public partial struct DamageOnTriggerSystem : ISystem 
    {

        public void OnCreate(ref SystemState state) 
        {
            state.RequireForUpdate<SimulationSingleton>();
            state.RequireForUpdate<EndSimulationEntityCommandBufferSystem.Singleton>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state) 
        {
            var ecbSingleton = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>();
            var damageOnTriggerJob = new DamageOnTriggerJob 
            {
                DamageOnTriggerLookup = SystemAPI.GetComponentLookup<DamageOnTrigger>(true),
                AlreadyDamagedLookup = SystemAPI.GetBufferLookup<AlreadyDamagedEntity>(true),
                DamageBufferLookup = SystemAPI.GetBufferLookup<DamageBufferElement>(true),
                QueryProxyLookup =
                    SystemAPI.GetComponentLookup<CrowdQueryProxy>(true),
                ECB = ecbSingleton.CreateCommandBuffer(state.WorldUnmanaged),
            };

            var simulationSingleton = SystemAPI.GetSingleton<SimulationSingleton>();
            state.Dependency = damageOnTriggerJob.Schedule(simulationSingleton, state.Dependency);
        }
    }

    public struct DamageOnTriggerJob : ITriggerEventsJob 
    {
        [ReadOnly] public ComponentLookup<DamageOnTrigger> DamageOnTriggerLookup;
        [ReadOnly] public BufferLookup<AlreadyDamagedEntity> AlreadyDamagedLookup;
        [ReadOnly] public BufferLookup<DamageBufferElement> DamageBufferLookup;
        [ReadOnly] public ComponentLookup<CrowdQueryProxy> QueryProxyLookup;
        public EntityCommandBuffer ECB;
        
        public void Execute(TriggerEvent triggerEvent) 
        {

            Entity damageDealingEntity;
            Entity damageReceivingEntity;

            if (DamageBufferLookup.HasBuffer(triggerEvent.EntityA) && DamageOnTriggerLookup.HasComponent(triggerEvent.EntityB)) 
            {
                damageReceivingEntity = triggerEvent.EntityA;
                damageDealingEntity = triggerEvent.EntityB;
            }
            else if (DamageOnTriggerLookup.HasComponent(triggerEvent.EntityA) && DamageBufferLookup.HasBuffer(triggerEvent.EntityB)) 
            {
                damageDealingEntity = triggerEvent.EntityA;
                damageReceivingEntity = triggerEvent.EntityB;
            }
            else 
            {
                return;
            }
            ECB.AppendToBuffer(damageDealingEntity, new AlreadyDamagedEntity { Value = damageReceivingEntity });
            Debug.LogError($"触发Trigger事件，攻击单位：{damageDealingEntity},被攻击单位：{damageReceivingEntity}");
            
            var alreadyDamagedBuffer = AlreadyDamagedLookup[damageDealingEntity];
            foreach (var alreadyDamagedEntity in alreadyDamagedBuffer)
            {
                if (alreadyDamagedEntity.Value.Equals(damageReceivingEntity))
                {
                    Debug.LogError($"已添加伤害到该单位：{damageReceivingEntity}故跳过");
                    return;
                }
            }

            var damageOnTrigger = DamageOnTriggerLookup[damageDealingEntity];
            uint queryProxyVersion =
                QueryProxyLookup.HasComponent(damageReceivingEntity)
                    ? QueryProxyLookup[damageReceivingEntity].ProxyVersion
                    : 0u;
            ECB.AppendToBuffer(
                damageReceivingEntity,
                CreateDamageElement(
                    damageOnTrigger.Value,
                    queryProxyVersion));
           
            
        }

        public static DamageBufferElement CreateDamageElement(
            int damage,
            uint queryProxyVersion)
        {
            return new DamageBufferElement
            {
                Value = damage,
                QueryProxyVersion = queryProxyVersion
            };
        }
    }
}
