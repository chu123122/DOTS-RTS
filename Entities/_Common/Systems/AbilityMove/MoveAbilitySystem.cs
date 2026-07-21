using Unity.Burst;
using Unity.Entities;
using Unity.NetCode;
using Unity.Transforms;

namespace TMG.NFE_Tutorial
{
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
    public partial struct MoveAbilitySystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var deltaTime = SystemAPI.Time.DeltaTime;
            foreach (var (transform, moveSpeed) in
                     SystemAPI.Query<RefRW<LocalTransform>, AbilityMoveSpeed>())
            {
                transform.ValueRW.Position += transform.ValueRW.Forward() * moveSpeed.Value * deltaTime;
            }
        }
    }
}
