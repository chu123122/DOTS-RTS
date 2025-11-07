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
                    damageThisTickBuffer.AddCommandData(new DamageThisTick { Tick = currentTick, Value = 0 });
                }
                else
                {
                    var totalDamage = 0;
                    if (damageThisTickBuffer.GetDataAtTick(currentTick, out var damageThisTick))
                    {
                        totalDamage = damageThisTick.Value;
                    }

                    foreach (var damage in damageBuffer)
                    {
                        totalDamage += damage.Value;
                    }

                    damageThisTickBuffer.AddCommandData(new DamageThisTick { Tick = currentTick, Value = totalDamage });
                    damageBuffer.Clear();
                }
            }
        }
    }
}