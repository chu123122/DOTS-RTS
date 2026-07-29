using Unity.Entities;

public struct DamageOnTrigger : IComponentData
{
    public int Value;
}

public struct AlreadyDamagedEntity : IBufferElementData
{
    public Entity Value;
}

public struct AlreadyApplyDamageTag : IComponentData{}