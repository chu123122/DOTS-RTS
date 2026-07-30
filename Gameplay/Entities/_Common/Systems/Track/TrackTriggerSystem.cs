using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using RTS.Unit.Components;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using RTS.Gameplay.Physics;

[WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
[UpdateInGroup(typeof(CrowdQuerySystemGroup))]
public partial struct TrackTriggerSystem : ISystem
{

    private CollisionFilter _collisionFilter;

    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<PhysicsWorldSingleton>();

        _collisionFilter = CrowdQueryCollisionFilters.UnitOverlap;
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var  physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
        var  collisionWorld = physicsWorld.CollisionWorld;
        ComponentLookup<CrowdQueryProxy> queryProxyLookup =
            SystemAPI.GetComponentLookup<CrowdQueryProxy>(true);

        var ecb = new EntityCommandBuffer(Allocator.Temp);
        foreach (var (trackDistance,
                     sourceProxy,
                     entity) in
                 SystemAPI.Query<RefRO<TrackDistance>,
                         RefRO<CrowdQueryProxy>>()
                     .WithEntityAccess().WithAll<IsUserUnitTag>())
        {
            CrowdQueryProxy sourceQueryProxy = sourceProxy.ValueRO;
            uint queryProxyVersion =
                sourceQueryProxy.CrowdStepVersion ==
                sourceQueryProxy.ProxyVersion
                    ? sourceQueryProxy.ProxyVersion
                    : 0u;
            int sourceBodyIndex =
                physicsWorld.GetRigidBodyIndex(entity);
            bool hasPublishedSourceBody =
                queryProxyVersion != 0 &&
                (uint)sourceBodyIndex <
                (uint)physicsWorld.NumBodies;
            float3 sphereCenter = hasPublishedSourceBody
                ? physicsWorld.Bodies[sourceBodyIndex].WorldFromBody.pos
                : float3.zero;

            NativeList<DistanceHit> hits = new NativeList<DistanceHit>(Allocator.Temp);

            if (hasPublishedSourceBody)
            {
                collisionWorld.OverlapSphere(
                    sphereCenter,
                    trackDistance.ValueRO.Distance,
                    ref hits,
                    _collisionFilter);
            }
            PublishClosestTarget(
                physicsWorld,
                hits,
                entity,
                sphereCenter,
                queryProxyVersion,
                queryProxyLookup,
                ecb);
            hits.Dispose();
        }
        ecb.Playback(state.EntityManager);
        ecb.Dispose();
    }

    public void OnDestroy(ref SystemState state)
    {
    }

    private static void PublishClosestTarget(
        PhysicsWorld physicsWorld,
        NativeList<DistanceHit> hits,
        Entity sourceEntity,
        float3 sourcePosition,
        uint queryProxyVersion,
        ComponentLookup<CrowdQueryProxy> queryProxyLookup,
        EntityCommandBuffer ecb)
    {
        float closestDistanceSq = float.MaxValue;
        Entity closest = Entity.Null;
        foreach (DistanceHit hit in hits)
        {
            Entity entityInPhysics =
                physicsWorld.Bodies[hit.RigidBodyIndex].Entity;
            if (sourceEntity == entityInPhysics ||
                !queryProxyLookup.HasComponent(entityInPhysics))
                continue;

            CrowdQueryProxy targetProxy =
                queryProxyLookup[entityInPhysics];
            if (targetProxy.ProxyVersion != queryProxyVersion)
                continue;

            float3 targetPosition =
                physicsWorld.Bodies[hit.RigidBodyIndex].WorldFromBody.pos;
            float distanceSq =
                math.distancesq(sourcePosition, targetPosition);
            if (distanceSq >= closestDistanceSq)
                continue;

            closestDistanceSq = distanceSq;
            closest = entityInPhysics;
        }

        ecb.SetComponent(sourceEntity, new TrackEntity
        {
            Entity = closest,
            QueryProxyVersion = queryProxyVersion
        });
    }
}
