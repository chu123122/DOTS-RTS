using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Physics;
using Unity.Transforms;
using _RePlaySystem.Base;
using UnityEngine;

namespace 通用
{
    [UpdateInGroup(typeof(SimulationSystemGroup), OrderFirst = true)]
    public partial class InitializeUnitSystem : SystemBase
    {
        public Action<int, float3> OnCreateHealthBar;

        protected override void OnUpdate()
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);

            foreach (var (
                         physicsMass,
                         rtsTeam, 
                         localTransform, 
                         entity) in
                     SystemAPI.Query<
                             RefRW<PhysicsMass>, 
                             RefRO<RtsTeam>, 
                             RefRO<LocalTransform>>()
                         .WithAll<IsNewCreatingTag>()
                         .WithEntityAccess())
            {
           

                int unitId = -1;
                bool readyToInitialize = false;
                bool isLocalReplay = false;

                bool haveGhost = SystemAPI.HasComponent<GhostInstance>(entity);
                bool haveLocal = SystemAPI.HasComponent<LocalInstance>(entity);
                Debug.Log(
                    $"Entity: {entity.Index}, " +
                    $"HasGhost: {haveGhost}, " +
                    $"HasLocal: {haveLocal}");
                // --- 检查 1: 联机模式 (GhostInstance) ---
                if (haveGhost)
                {
                    unitId = SystemAPI.GetComponent<GhostInstance>(entity).ghostId;
                    readyToInitialize = true;
                }
                // --- 检查 2: 回放/本地模式 (LocalInstance) ---
                else if (haveLocal)
                {
                    unitId = SystemAPI.GetComponent<LocalInstance>(entity).Id;
                    readyToInitialize = true;
                    isLocalReplay = true;
                }
                else
                {
                    continue; // 等待组件同步
                }

                if (readyToInitialize)
                {
                    // 【核心修复】解决物理报错
                    if (isLocalReplay)
                    {
                        // 回放单位是纯代码控制的，删除 PhysicsMass，
                        // 这样 NetCode 就不会把它当成 Dynamic 物体验证了
                        ecb.RemoveComponent<PhysicsMass>(entity);
                        ecb.RemoveComponent<PhysicsVelocity>(entity); 
                    }
                    else
                    {
                        // 联机单位：锁定惯性
                        physicsMass.ValueRW.InverseInertia = float3.zero;
                        physicsMass.ValueRW.InverseMass = 0; // 确保是 Kinematic
                    }

                    // UI 和 队伍颜色
                    var teamColor = rtsTeam.ValueRO.Value switch
                    {
                        TeamType.Blue => new float4(0, 0, 1, 1),
                        TeamType.Red => new float4(1, 0, 0, 1),
                        _ => new float4(1)
                    };

                    OnCreateHealthBar?.Invoke(unitId, localTransform.ValueRO.Position);
                    ecb.RemoveComponent<IsNewCreatingTag>(entity);
                }
            }

            ecb.Playback(EntityManager);
            ecb.Dispose();
        }
    }
}