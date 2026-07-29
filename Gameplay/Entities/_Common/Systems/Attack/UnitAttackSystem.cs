using System.Diagnostics;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Transforms;

namespace Entities.Unit.System
{
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
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

    public EntityCommandBuffer.ParallelWriter ECB;

    private void Execute(ref AttackCoolDown attackCoolDown, in AttackProperties attackProperties,
        in AttackDamage attackDamage, in AttackEntity beAttackEntity, Entity attackEntity,
        [ChunkIndexInQuery] int sortKey)
    {
        if(!TransformLookup.HasComponent(beAttackEntity.Entity))return;
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
