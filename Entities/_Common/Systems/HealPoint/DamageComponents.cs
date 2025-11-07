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
    [GhostComponent(PrefabType = GhostPrefabType.AllPredicted,OwnerSendType = SendToOwnerType.SendToNonOwner)]
    public struct DamageThisTick : ICommandData
    {
        public NetworkTick Tick { get; set; }
        public int Value;
    }
    
    public struct DestroyEntityTag:IComponentData{}
}