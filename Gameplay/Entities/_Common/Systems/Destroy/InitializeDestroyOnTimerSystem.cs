using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace TMG.NFE_Tutorial
{
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
    public partial struct InitializeDestroyOnTimerSystem : ISystem
    {
        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            double currentTime = SystemAPI.Time.ElapsedTime;

            foreach (var (destroyOnTimer,
                         entity) in 
                     SystemAPI.Query<DestroyOnTimer>()
                         .WithEntityAccess().WithNone<DestroyAtTime>())
            {
                ecb.AddComponent(entity, new DestroyAtTime
                {
                    Value = currentTime + math.max(0f, destroyOnTimer.Value)
                });
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }
    }
}
