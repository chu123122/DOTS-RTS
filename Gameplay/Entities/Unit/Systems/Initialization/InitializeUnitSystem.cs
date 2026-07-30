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
                // 检查 1：联机模式（GhostInstance）
                if (haveGhost)
                {
                    unitId = SystemAPI.GetComponent<GhostInstance>(entity).ghostId;
                    readyToInitialize = true;
                }
                // 检查 2：回放/本地模式（LocalInstance）
                else if (haveLocal)
                {
                    unitId = SystemAPI.GetComponent<LocalInstance>(entity).Id;
                    readyToInitialize = true;
                    isLocalReplay = true;
                }
                else
                {
                    continue; // 等组件同步
                }

                if (readyToInitialize)
                {
                    if (!SystemAPI.HasComponent<CrowdDiscShape>(entity))
                    {
                        PhysicsCollider collider =
                            SystemAPI.HasComponent<PhysicsCollider>(entity)
                                ? SystemAPI.GetComponent<PhysicsCollider>(entity)
                                : default;
                        CrowdShapeSourceState source =
                            CrowdShapeAdapter.CaptureSource(
                                collider,
                                localTransform.ValueRO);
                        ecb.AddComponent(entity, new CrowdDiscShape
                        {
                            Radius = CrowdShapeAdapter.CalculateRadius(
                                collider,
                                localTransform.ValueRO),
                            Version = 1
                        });
                        ecb.AddComponent(entity, source);
                    }
                    if (!SystemAPI.HasComponent<CrowdQueryProxy>(entity))
                        ecb.AddComponent<CrowdQueryProxy>(entity);

                    // 核心修复：解决物理报错
                    if (isLocalReplay)
                    {
                        // 本地单位由 Flow Field 直接写 LocalTransform，不再交给
                        // Unity Physics 积分或图形插值。仅删 Mass/Velocity 不够，
                        // PhysicsGraphicalSmoothing 仍会用旧缓存写 LocalToWorld，
                        // 导致单位在固定位置上下抖动。
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
                        physicsMass.ValueRW.InverseMass = 0; // 设为 Kinematic
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
