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

[UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
public partial struct TrackTriggerSystem : ISystem
{

    private CollisionFilter _collisionFilter;

    public void OnCreate(ref SystemState state)
    {
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
        var  physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
        var  collisionWorld = physicsWorld.CollisionWorld;

        var ecb = new EntityCommandBuffer(Allocator.Temp);
        foreach (var (localTransform, 
                     trackDistance, 
                     entity) in
                 SystemAPI.Query<RefRO<LocalTransform>,
                         RefRO<TrackDistance>>()
                     .WithEntityAccess().WithAll<IsUserUnitTag,Simulate>())
        {
            // 球体位置
            float3 sphereCenter = localTransform.ValueRO.Position;

            NativeList<DistanceHit> hits = new NativeList<DistanceHit>(Allocator.Temp);

            bool haveTrackTarget = collisionWorld.OverlapSphere(
                sphereCenter,
                trackDistance.ValueRO.Distance,
                ref hits,
                _collisionFilter
            );
            CheckSphere(haveTrackTarget, hits, entity, state.EntityManager,ecb);
            // if (state.World.IsClient())
            // {
            DebugDrawing.DrawWireCircleXZ(sphereCenter, trackDistance.ValueRO.Distance, Color.green);
            // }
        }
        ecb.Playback(state.EntityManager);
    }

    public void OnDestroy(ref SystemState state)
    {
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
                    minDistance = distance;
                    ecb.SetComponent(entity, new TrackEntity() { Entity = entityInPhy });
                   // Debug.Log($"单位{entity}最近的<color=blue>追踪</color>单位是：{entityInPhy}");
                }
            }
        }
        else
        {
            ecb.SetComponent(entity, new TrackEntity() { Entity = Entity.Null });
           // Debug.Log($"单位{entity}<color=blue>追踪</color>范围中无单位");
        }
      
    }
}