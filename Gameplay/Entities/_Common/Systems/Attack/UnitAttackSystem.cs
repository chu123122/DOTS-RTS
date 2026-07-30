using System.Diagnostics;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using RTS.Gameplay.Physics;
using RTS.Unit.Components;

namespace Entities.Unit.System
{
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
    [UpdateInGroup(typeof(CrowdQuerySystemGroup))]
    [UpdateAfter(typeof(UnitAttackTriggerSystem))]
    public partial struct UnitAttackSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<BeginSimulationEntityCommandBufferSystem.Singleton>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var ecbSingleton = SystemAPI.GetSingleton<BeginSimulationEntityCommandBufferSystem.Singleton>();
            var ecb = ecbSingleton.CreateCommandBuffer(state.WorldUnmanaged);

            state.Dependency = new UnitAttackJob
            {
                ECB = ecb.AsParallelWriter(),
                TransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true),
                QueryProxyLookup =
                    SystemAPI.GetComponentLookup<CrowdQueryProxy>(true),
                CurrentTime = SystemAPI.Time.ElapsedTime
            }.ScheduleParallel(state.Dependency);
        }
    }
}

[BurstCompile]
public partial struct UnitAttackJob : IJobEntity
{
    [ReadOnly] public double CurrentTime;
    [ReadOnly] public ComponentLookup<LocalTransform> TransformLookup;
    [ReadOnly] public ComponentLookup<CrowdQueryProxy> QueryProxyLookup;

    public EntityCommandBuffer.ParallelWriter ECB;

    private void Execute(ref AttackCoolDown attackCoolDown, in AttackProperties attackProperties,
        in AttackDamage attackDamage, in AttackEntity beAttackEntity, Entity attackEntity,
        in CrowdQueryProxy sourceProxy,
        [ChunkIndexInQuery] int sortKey)
    {
        if (beAttackEntity.QueryProxyVersion == 0 ||
            sourceProxy.ProxyVersion != beAttackEntity.QueryProxyVersion ||
            !TransformLookup.HasComponent(beAttackEntity.Entity) ||
            !QueryProxyLookup.HasComponent(beAttackEntity.Entity) ||
            QueryProxyLookup[beAttackEntity.Entity].ProxyVersion !=
                beAttackEntity.QueryProxyVersion)
            return;
        if (CurrentTime < attackCoolDown.NextAttackTime) return;

        float3 spawnPosition = TransformLookup[attackEntity].Position + attackProperties.FirePointOffset;
        float3 targetPosition = TransformLookup[beAttackEntity.Entity].Position;
        
        Entity newAttack = ECB.Instantiate(sortKey, attackProperties.AttackPrefab);
        LocalTransform newAttackTransform = LocalTransform.FromPositionRotation(spawnPosition,
            quaternion.LookRotationSafe(targetPosition - spawnPosition, math.up()));
        newAttackTransform.Scale = 0.3f;
        ECB.SetComponent(sortKey, newAttack, newAttackTransform);

        attackCoolDown.NextAttackTime =
            CurrentTime + math.max(0f, attackProperties.CooldownSeconds);
    }
}
