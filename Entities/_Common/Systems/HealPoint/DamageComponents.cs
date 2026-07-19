using Unity.Entities;
using Unity.NetCode;

namespace Entities._Common
{
    public struct HealthPointData : IComponentData
    {
        [GhostField]public int MaximumHp;
        [GhostField]public int CurrentHp;
    }
    
    [GhostComponent(PrefabType = GhostPrefabType.AllPredicted)]
    public struct DamageBufferElement : IBufferElementData
    {
        public int Value;
    }
    public struct DamageThisTick : IBufferElementData
    {
        public NetworkTick Tick;
        public int Value;
    }
    
    public struct DestroyEntityTag:IComponentData{}
}
