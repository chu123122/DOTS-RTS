using Entities._Common;
using Unity.Burst;
using Unity.Entities;
using Unity.NetCode;

namespace TMG.NFE_Tutorial
{
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
    public partial struct DestroyOnTimerSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<EndSimulationEntityCommandBufferSystem.Singleton>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var ecbSingleton = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>();
            var ecb = ecbSingleton.CreateCommandBuffer(state.WorldUnmanaged);

            double currentTime = SystemAPI.Time.ElapsedTime;

            foreach (var (destroyAtTime, entity) in SystemAPI.Query<DestroyAtTime>()
                         .WithNone<DestroyEntityTag>().WithEntityAccess())
            {
                if (currentTime >= destroyAtTime.Value)
                {
                    ecb.AddComponent<DestroyEntityTag>(entity);
                }
            }
        }
    }
}
