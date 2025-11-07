using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Physics;
using Unity.Transforms;

namespace 通用
{
    [UpdateInGroup(typeof(SimulationSystemGroup),OrderFirst = true)]
    public partial class InitializeUnitSystem:SystemBase
    {
        public Action<int,float3> OnCreateHealthBar;
        protected override void OnUpdate()
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (var (physicsMass,rtsTeam,localTransform,newCharacterEntity) in
                     SystemAPI.Query<RefRW<PhysicsMass>,
                             RefRO<RtsTeam>,
                             RefRO<LocalTransform>>().
                             WithEntityAccess().WithAny<NewBasicUnitTag>())
            {
                physicsMass.ValueRW.InverseInertia[0] = 0;
                physicsMass.ValueRW.InverseInertia[1] = 0;
                physicsMass.ValueRW.InverseInertia[2] = 0;

                var teamColor = rtsTeam.ValueRO.Value switch
                {
                    TeamType.Blue => new float4(0,0,1,1),
                    TeamType.Red => new float4(1,0,0,1),
                    _ => new float4(1)
                };
                int ghostId = EntityManager.GetComponentData<GhostInstance>(newCharacterEntity).ghostId;
                OnCreateHealthBar?.Invoke(ghostId,localTransform.ValueRO.Position);
                //ecb.SetComponent(newCharacterEntity,new URPMaterialPropertyBaseColor(){Value = teamColor});
                ecb.RemoveComponent<NewBasicUnitTag>(newCharacterEntity);
            }
            ecb.Playback(EntityManager);
        }
    }
}