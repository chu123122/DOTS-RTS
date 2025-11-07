using System.Drawing;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Physics;
using Unity.Transforms;
using Utils;
using 通用;
using Color = UnityEngine.Color;

namespace Entities.Unit.System
{
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
    public partial struct UnitAttackTriggerSystem:ISystem
    {
        private CollisionFilter _collisionFilter;

        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NetworkTime>();
            state.RequireForUpdate<PhysicsWorldSingleton>();
            state.RequireForUpdate<NetworkStreamInGame>();

            _collisionFilter = new CollisionFilter
            {
                BelongsTo = ~0u,
                CollidesWith = 1 << 1,
                GroupIndex = 0
            };
        }
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var networkTime = SystemAPI.GetSingleton<NetworkTime>();
            if(!networkTime.IsFirstTimeFullyPredictingTick)return;
            
            var  physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
            var  collisionWorld = physicsWorld.CollisionWorld;
            
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (var (localTransform,
                         attackDistance,
                         entity) in 
                     SystemAPI.Query<RefRO<LocalTransform>,
                             RefRO<AttackDistance>>().
                         WithEntityAccess().WithAll<Simulate,IsUserUnitTag>())
            {
                float3 sphereCenter = localTransform.ValueRO.Position;
                NativeList<DistanceHit> hits = new NativeList<DistanceHit>(Allocator.Temp);

                bool haveAttackTarget = collisionWorld.OverlapSphere(
                    sphereCenter,
                    attackDistance.ValueRO.Distance,
                    ref hits,
                    _collisionFilter
                );
                CheckSphere(haveAttackTarget, hits, entity, state.EntityManager,ecb);
                // if (state.World.IsServer())
                // {
                DebugDrawing.DrawWireCircleXZ(sphereCenter, attackDistance.ValueRO.Distance, Color.red);
                // }
                // else
                // {
                //     DebugDrawing.DrawWireCircleXZ(sphereCenter, attackDistance.ValueRO.Distance, Color.red);
                // }
            }
            ecb.Playback(state.EntityManager);
        }
        
        private void CheckSphere(bool isInSphere, NativeList<DistanceHit> hits, Entity entity,
            EntityManager entityManager,EntityCommandBuffer ecb)
        {
            
            if (isInSphere && hits.Length > 1)
            {
                float minDistance = 1000f;
                foreach (var hit in hits)
                {
                    var  physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
                    Entity entityInPhy = physicsWorld.Bodies[hit.RigidBodyIndex].Entity;
                    if (entity.Equals(entityInPhy)) continue;
                    float distance = math.distance(entityManager.GetComponentData<LocalTransform>(entityInPhy).Position,
                        entityManager.GetComponentData<LocalTransform>(entity).Position);
                    if (minDistance > distance)
                    {
                        //Debug.LogWarning($"{entityManager.World} 的单位攻击范围里检测到敌对单位");
                        minDistance = distance;
                        ecb.SetComponent(entity, new AttackEntity() { Entity = entityInPhy });
                       // Debug.Log($"单位{entity}最近的<color=red>攻击</color>单位是：{entityInPhy}");
                    }
                }
            }
            else
            {
                ecb.SetComponent(entity, new AttackEntity() { Entity = Entity.Null });
               // Debug.Log($"单位{entity}<color=red>攻击</color>范围中无单位");
            }
           
        }
    }
}