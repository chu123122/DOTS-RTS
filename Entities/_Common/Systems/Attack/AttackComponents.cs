using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;

public struct AttackDistance : IComponentData
{
    public float Distance;
}

public struct AttackEntity : IComponentData
{
    public Entity Entity;
}

[GhostComponent]
public struct AttackDamage : IComponentData
{
    [GhostField] public int Damage;
}

public struct AttackProperties : IComponentData
{
    public float3 FirePointOffset;
    public uint CooldownTickCount;
    public Entity AttackPrefab;
}

public struct AttackCoolDown : ICommandData
{
    public NetworkTick Tick { get; set; }
    public NetworkTick Value;
}