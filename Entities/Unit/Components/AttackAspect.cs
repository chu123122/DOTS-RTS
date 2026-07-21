using Unity.Entities;

namespace RTS.Unit.Components
{

public readonly partial struct AttackAspect : IAspect
{
    private readonly RefRO<AttackDamage> _attackDamage;
    private readonly RefRO<AttackEntity> _attackEntity;
    private readonly RefRO<AttackProperties> _attackCooldownTick;
        
    private readonly RefRO<AttackCoolDown> _attackCooldownTargetTick;

    public int AttackDamage => _attackDamage.ValueRO.Damage;
    public Entity AttackEntity => _attackEntity.ValueRO.Entity;
    public bool CantAttack=>true;
    public float CooldownSeconds => _attackCooldownTick.ValueRO.CooldownSeconds;
    public AttackCoolDown CooldownState => _attackCooldownTargetTick.ValueRO;
}
}
