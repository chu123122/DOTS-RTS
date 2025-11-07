using System.Diagnostics;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Transforms;

namespace Entities.Unit.System
{
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
    public partial struct UnitAttackSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NetworkTime>();
            state.RequireForUpdate<BeginSimulationEntityCommandBufferSystem.Singleton>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var networkTime = SystemAPI.GetSingleton<NetworkTime>();
            if(!networkTime.IsFirstTimeFullyPredictingTick)return;
            
            var ecbSingleton = SystemAPI.GetSingleton<BeginSimulationEntityCommandBufferSystem.Singleton>();
            var ecb = ecbSingleton.CreateCommandBuffer(state.WorldUnmanaged);

            state.Dependency = new UnitAttackJob
            {
                ECB = ecb.AsParallelWriter(),
                TransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true),
                CurrentTick = networkTime.ServerTick,
                NetworkTime = networkTime
            }.ScheduleParallel(state.Dependency);
        }
    }
}

[BurstCompile]
[WithAll(typeof(Simulate))]
public partial struct UnitAttackJob : IJobEntity
{
    [ReadOnly] public NetworkTime NetworkTime;
    [ReadOnly] public NetworkTick CurrentTick;
    [ReadOnly] public ComponentLookup<LocalTransform> TransformLookup;

    public EntityCommandBuffer.ParallelWriter ECB;

    private void Execute(ref DynamicBuffer<AttackCoolDown> attackCoolDown, in AttackProperties attackProperties,
        in AttackDamage attackDamage, in AttackEntity beAttackEntity, Entity attackEntity,
        [ChunkIndexInQuery] int sortKey)
    {
        if(!TransformLookup.HasComponent(beAttackEntity.Entity))return;
        bool canAttack = false;
        for (uint i = 0u; i < NetworkTime.SimulationStepBatchSize; i++)
        {
            var testTick = CurrentTick;
            testTick.Subtract(i);
            if (!attackCoolDown.GetDataAtTick(testTick, out var cooldownExpirationTick))
            {
                cooldownExpirationTick.Value = NetworkTick.Invalid;
            }
            // 判断是否可以攻击
            if (cooldownExpirationTick.Value==NetworkTick.Invalid || CurrentTick.IsNewerThan(cooldownExpirationTick.Value))
            {
                canAttack = true;
                break;
            }
        }
        if (!canAttack) return;

        float3 spawnPosition = TransformLookup[attackEntity].Position + attackProperties.FirePointOffset;
        float3 targetPosition = TransformLookup[beAttackEntity.Entity].Position;
        
        Entity newAttack = ECB.Instantiate(sortKey, attackProperties.AttackPrefab);
        LocalTransform newAttackTransform = LocalTransform.FromPositionRotation(spawnPosition,
            quaternion.LookRotationSafe(targetPosition - spawnPosition, math.up()));
        newAttackTransform.Scale = 0.3f;
        ECB.SetComponent(sortKey, newAttack, newAttackTransform);

        var newCooldownTick = CurrentTick;
        newCooldownTick.Add(attackProperties.CooldownTickCount);

        attackCoolDown.AddCommandData(new AttackCoolDown() { Tick = CurrentTick, Value = newCooldownTick });
    }
}