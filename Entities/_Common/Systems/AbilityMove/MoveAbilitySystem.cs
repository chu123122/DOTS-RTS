using Unity.Burst;
using Unity.Entities;
using Unity.NetCode;
using Unity.Transforms;

namespace TMG.NFE_Tutorial
{
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
    public partial struct MoveAbilitySystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NetworkTime>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var networkTime = SystemAPI.GetSingleton<NetworkTime>();
            if (!networkTime.IsFirstTimeFullyPredictingTick) return;
            
            var deltaTime = SystemAPI.Time.DeltaTime;
            foreach (var (transform, moveSpeed) in SystemAPI.Query<RefRW<LocalTransform>, AbilityMoveSpeed>()
                         .WithAll<Simulate>())
            {
                transform.ValueRW.Position += transform.ValueRW.Forward() * moveSpeed.Value * deltaTime;
            }
        }
    }
}