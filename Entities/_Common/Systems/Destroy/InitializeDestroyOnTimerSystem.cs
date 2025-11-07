using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;

namespace TMG.NFE_Tutorial
{
    public partial struct InitializeDestroyOnTimerSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NetworkTime>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            var simulationTickRate = NetCodeConfig.Global.ClientServerTickRate.SimulationTickRate;
            var currentTick = SystemAPI.GetSingleton<NetworkTime>().ServerTick;

            foreach (var (destroyOnTimer,
                         entity) in 
                     SystemAPI.Query<DestroyOnTimer>()
                         .WithEntityAccess().WithNone<DestroyAtTick>())
            {
                var lifetimeInTicks = (uint)(destroyOnTimer.Value * simulationTickRate);
                var targetTick = currentTick;
                targetTick.Add(lifetimeInTicks);
                ecb.AddComponent(entity, new DestroyAtTick { Value = targetTick });
            }

            ecb.Playback(state.EntityManager);
        }
    }
}