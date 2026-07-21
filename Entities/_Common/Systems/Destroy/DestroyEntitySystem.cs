using Entities._Common;
using Unity.Entities;
using Unity.NetCode;

namespace TMG.NFE_Tutorial
{
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup),OrderLast = true)]
    public partial struct DestroyEntitySystem:ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<BeginSimulationEntityCommandBufferSystem.Singleton>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var ecbSingleton = SystemAPI.GetSingleton<BeginSimulationEntityCommandBufferSystem.Singleton>();
            var ecb = ecbSingleton.CreateCommandBuffer(state.WorldUnmanaged);
            
            foreach (var (_, entity) in
                     SystemAPI.Query<DestroyEntityTag>().WithEntityAccess())
            {
                ecb.DestroyEntity(entity);
            }
        }
    }
}
