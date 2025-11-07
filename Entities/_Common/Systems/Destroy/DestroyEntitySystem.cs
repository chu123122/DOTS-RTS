using Entities._Common;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Transforms;

namespace TMG.NFE_Tutorial
{
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup),OrderLast = true)]
    public partial struct DestroyEntitySystem:ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<BeginSimulationEntityCommandBufferSystem.Singleton>();
            state.RequireForUpdate<NetworkTime>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var networkTime = SystemAPI.GetSingleton<NetworkTime>();
            if (!networkTime.IsFirstTimeFullyPredictingTick) return;
            
            var ecbSingleton = SystemAPI.GetSingleton<BeginSimulationEntityCommandBufferSystem.Singleton>();
            var ecb = ecbSingleton.CreateCommandBuffer(state.WorldUnmanaged);
            
            foreach (var (localTransform,
                         entity) in 
                     SystemAPI.Query<RefRW<LocalTransform>>().
                         WithEntityAccess().WithAll<Simulate,DestroyEntityTag>())
            {
                if (state.WorldUnmanaged.IsServer())
                {
                    ecb.DestroyEntity(entity);
                }
                else
                {
                  //  localTransform.ValueRW.Position = new float3(1000, 1000, 1000);
                }
            }
        }
    }
}