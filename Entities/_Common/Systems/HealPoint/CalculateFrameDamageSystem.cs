using Unity.Entities;
using Unity.NetCode;

namespace Entities._Common
{
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup), OrderLast = true)]
    public partial struct CalculateFrameDamageSystem:ISystem
    {
        public void OnCreate(ref SystemState state)
        {
        }

        public void OnUpdate(ref SystemState state)
        {
            foreach (var (damageBuffer,damageThisTickBuffer) in 
                     SystemAPI.Query<DynamicBuffer<DamageBufferElement>,
                         DynamicBuffer<DamageThisTick>>())
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
                    damageThisTickBuffer.Add(new DamageThisTick { Value = totalDamage });
                    damageBuffer.Clear();
                }
            }
        }
    }
}
