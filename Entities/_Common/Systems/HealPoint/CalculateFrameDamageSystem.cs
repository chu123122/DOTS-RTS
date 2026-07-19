using Unity.Entities;
using Unity.NetCode;

namespace Entities._Common
{
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup), OrderLast = true)]
    public partial struct CalculateFrameDamageSystem:ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NetworkTime>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var currentTick = SystemAPI.GetSingleton<NetworkTime>().ServerTick;
            foreach (var (damageBuffer,damageThisTickBuffer) in 
                     SystemAPI.Query<DynamicBuffer<DamageBufferElement>,
                         DynamicBuffer<DamageThisTick>>().WithAll<Simulate>())
            {
                if (damageBuffer.IsEmpty)
                {
                    damageThisTickBuffer.Clear();
                }
                else
                {
                    var totalDamage = 0;
                    foreach (var damage in damageBuffer)
                    {
                        totalDamage += damage.Value;
                    }

                    damageThisTickBuffer.Clear();
                    damageThisTickBuffer.Add(new DamageThisTick { Tick = currentTick, Value = totalDamage });
                    damageBuffer.Clear();
                }
            }
        }
    }
}
