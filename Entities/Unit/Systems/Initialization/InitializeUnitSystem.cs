using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Physics;
using Unity.Physics.GraphicsIntegration;
using Unity.Transforms;
using _RePlaySystem.Base;
using RTS.Unit.Components;

namespace RTS.Unit.Systems.Initialization
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
                         localTransform, 
                         entity) in
                     SystemAPI.Query<
                             RefRW<PhysicsMass>, 
                             RefRO<LocalTransform>>()
                         .WithAll<IsNewCreatingTag>()
                         .WithEntityAccess())
            {
           

                int unitId = -1;
                bool readyToInitialize = false;
                bool isLocalReplay = false;

                bool haveGhost = SystemAPI.HasComponent<GhostInstance>(entity);
                bool haveLocal = SystemAPI.HasComponent<LocalInstance>(entity);
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
                        // 本地单位由 Flow Field 直接写 LocalTransform，不再交给
                        // Unity Physics 积分或图形插值。若只删除 Mass/Velocity，
                        // PhysicsGraphicalSmoothing 仍会用旧缓存写 LocalToWorld，
                        // 表现为单位在固定位置持续上下浮动。
                        ecb.RemoveComponent<PhysicsMass>(entity);
                        ecb.RemoveComponent<PhysicsVelocity>(entity);
                        if (SystemAPI.HasComponent<PhysicsGraphicalSmoothing>(entity))
                            ecb.RemoveComponent<PhysicsGraphicalSmoothing>(entity);
                        if (SystemAPI.HasComponent<PhysicsGraphicalInterpolationBuffer>(entity))
                            ecb.RemoveComponent<PhysicsGraphicalInterpolationBuffer>(entity);
                    }
                    else
                    {
                        // 联机单位：锁定惯性
                        physicsMass.ValueRW.InverseInertia = float3.zero;
                        physicsMass.ValueRW.InverseMass = 0; // 确保是 Kinematic
                    }

                    OnCreateHealthBar?.Invoke(unitId, localTransform.ValueRO.Position);
                    ecb.RemoveComponent<IsNewCreatingTag>(entity);
                }
            }

            ecb.Playback(EntityManager);
            ecb.Dispose();
        }
    }
}
