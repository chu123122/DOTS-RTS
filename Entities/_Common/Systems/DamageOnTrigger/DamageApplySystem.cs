/*using Entities._Common;
using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using Unity.Physics.Systems;
using UnityEngine;

namespace TMG.NFE_Tutorial
{
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateAfter(typeof(DamageOnTriggerSystem))]
    public partial struct DamageApplySystem:ISystem
    {
        private NativeHashSet<Entity> _entities;
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NetworkTime>();
            state.RequireForUpdate<EndSimulationEntityCommandBufferSystem.Singleton>();
            _entities = new NativeHashSet<Entity>(100, Allocator.Persistent);
        }

        public void OnDestroy(ref SystemState state)
        {
            _entities.Dispose();
        }

        public void OnUpdate(ref SystemState state)
        {
            var networkTime = SystemAPI.GetSingleton<NetworkTime>();
            if(!networkTime.IsFirstTimeFullyPredictingTick)return;
            
            var ecbSingleton = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>();
            var ecb = ecbSingleton.CreateCommandBuffer(state.WorldUnmanaged);
            foreach (var (alreadyDamagedEntity,
                         damageOnTrigger,
                         entity) in 
                     SystemAPI.Query<DynamicBuffer<AlreadyDamagedEntity>,
                     RefRO<DamageOnTrigger>>().
                         WithEntityAccess().WithNone<AlreadyApplyDamageTag>())
            {
                if(alreadyDamagedEntity.Length==0)return;
                if(_entities.Contains(entity))return;
                
                NativeHashSet<Entity> receiveDamageEntities = new NativeHashSet<Entity>(alreadyDamagedEntity.Length,Allocator.Temp);
                foreach (AlreadyDamagedEntity damagedEntity in alreadyDamagedEntity)
                {
                    receiveDamageEntities.Add(damagedEntity.Value);
                }
                
                foreach (var receiveDamageEntity in receiveDamageEntities)
                {
                    Debug.Log($"计算子弹单位：{entity}对单位{receiveDamageEntity}的伤害，在{state.World}");
                    ecb.AppendToBuffer(receiveDamageEntity,new DamageBufferElement(){Value = damageOnTrigger.ValueRO.Value});
                }
                ecb.AddComponent(entity,new AlreadyApplyDamageTag());
                _entities.Add(entity);
            }
            //   ecb.Playback(state.EntityManager);
        }
    }
}*/