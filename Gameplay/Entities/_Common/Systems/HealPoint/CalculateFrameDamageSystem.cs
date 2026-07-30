using Unity.Entities;
using Unity.NetCode;
using RTS.Unit.Components;

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
            ComponentLookup<CrowdQueryProxy> queryProxyLookup =
                SystemAPI.GetComponentLookup<CrowdQueryProxy>(true);
            foreach (var (damageBuffer,
                         damageThisTickBuffer,
                         entity) in
                     SystemAPI.Query<DynamicBuffer<DamageBufferElement>,
                         DynamicBuffer<DamageThisTick>>()
                         .WithEntityAccess())
            {
                if (damageBuffer.IsEmpty)
                {
                    damageThisTickBuffer.Clear();
                }
                else
                {
                    var totalDamage = 0;
                    bool hasQueryProxy =
                        queryProxyLookup.HasComponent(entity);
                    uint currentProxyVersion = hasQueryProxy
                        ? queryProxyLookup[entity].ProxyVersion
                        : 0u;
                    foreach (var damage in damageBuffer)
                    {
                        if (IsDamageVersionCurrent(
                                damage,
                                hasQueryProxy,
                                currentProxyVersion))
                            totalDamage += damage.Value;
                    }

                    damageThisTickBuffer.Clear();
                    damageThisTickBuffer.Add(new DamageThisTick { Value = totalDamage });
                    damageBuffer.Clear();
                }
            }
        }

        public static bool IsDamageVersionCurrent(
            DamageBufferElement damage,
            bool hasQueryProxy,
            uint currentProxyVersion)
        {
            return hasQueryProxy
                ? damage.QueryProxyVersion != 0 &&
                  damage.QueryProxyVersion == currentProxyVersion
                : damage.QueryProxyVersion == 0;
        }
    }
}
