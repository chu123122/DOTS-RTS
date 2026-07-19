using Unity.Entities;
using 通用;

public readonly partial struct AttackAspect : IAspect
{
    private readonly RefRO<AttackDamage> _attackDamage;
    private readonly RefRO<AttackEntity> _attackEntity;
    private readonly RefRO<AttackProperties> _attackCooldownTick;
        
    private readonly RefRO<AttackCoolDown> _attackCooldownTargetTick;

    public int AttackDamage => _attackDamage.ValueRO.Damage;
    public Entity AttackEntity => _attackEntity.ValueRO.Entity;
    public bool CantAttack=>true;
    public uint CoolDownTicks => _attackCooldownTick.ValueRO.CooldownTickCount;
    public AttackCoolDown CooldownState => _attackCooldownTargetTick.ValueRO;
}
