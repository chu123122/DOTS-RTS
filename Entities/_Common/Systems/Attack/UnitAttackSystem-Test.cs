
 /*
 using Unity.Entities;
 using Unity.Mathematics;
 using Unity.NetCode;
 using Unity.Transforms;
 using UnityEngine;

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

         public void OnUpdate(ref SystemState state)
         {
             var networkTime = SystemAPI.GetSingleton<NetworkTime>();
             if (!networkTime.IsFirstTimeFullyPredictingTick) return;
             
             var ecbSingleton = SystemAPI.GetSingleton<BeginSimulationEntityCommandBufferSystem.Singleton>();
             var ecb = ecbSingleton.CreateCommandBuffer(state.WorldUnmanaged);
             
             var currentTick = networkTime.ServerTick;
             
             foreach (var (attackCoolDown,
                          attackProperties,
                          beAttackEntity,
                          attackEntity) in 
                      SystemAPI.Query<DynamicBuffer<AttackCoolDown>,
                              RefRO<AttackProperties>,
                              RefRO<AttackEntity>>().
                          WithEntityAccess().WithAll<Simulate>())
             {
                 if (!SystemAPI.HasComponent<LocalTransform>(beAttackEntity.ValueRO.Entity)) continue;
                 bool canAttack = false;
                 for (uint i = 0u; i < networkTime.SimulationStepBatchSize; i++)
                 {
                     var testTick = currentTick;
                     testTick.Subtract(i);
                     if (!attackCoolDown.GetDataAtTick(testTick, out var cooldownExpirationTick))
                     {
                         cooldownExpirationTick.Value = NetworkTick.Invalid;
                     }
                     // 判断是否可以攻击
                     if (cooldownExpirationTick.Value==NetworkTick.Invalid || currentTick.IsNewerThan(cooldownExpirationTick.Value))
                     {
                         canAttack = true;
                         break;
                     }
                 }
                
                
                 if (!canAttack) continue;

                 // 获取攻击生成点和目标位置
                 float3 spawnPosition = SystemAPI.GetComponent<LocalTransform>(attackEntity).Position +
                                        attackProperties.ValueRO.FirePointOffset;
                 float3 targetPosition = SystemAPI.GetComponent<LocalTransform>(beAttackEntity.ValueRO.Entity).Position;

                 // 实例化攻击实体
                 Entity newAttack = ecb.Instantiate(attackProperties.ValueRO.AttackPrefab);
                 LocalTransform newAttackTransform = LocalTransform.FromPositionRotation(spawnPosition,
                     quaternion.LookRotationSafe(targetPosition - spawnPosition, math.up()));
                 newAttackTransform.Scale = 0.3f; // 设置攻击实例的大小
                 ecb.SetComponent(newAttack, newAttackTransform);

                 // 更新冷却时间
                 var newCooldownTick = currentTick;
                 newCooldownTick.Add(attackProperties.ValueRO.CooldownTickCount);
                 attackCoolDown.AddCommandData(new AttackCoolDown() { Tick = currentTick, Value = newCooldownTick });
             }
         }
     }
 }
 */
