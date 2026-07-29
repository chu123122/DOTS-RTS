using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;
using UnityEngine.Serialization;

namespace 通用
{
    public class AttackAbilityAuthoring:MonoBehaviour
    {
        public GameObject attackPrefab;
        public int attackDamage;
        public float attackDistance;
        public float attackCooldown;
        public float3 firePointOffset;

        public class Baker:Baker<AttackAbilityAuthoring>
        {
            public override void Bake(AttackAbilityAuthoring abilityAuthoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent(entity,new AttackDamage(){Damage =abilityAuthoring.attackDamage });
                AddComponent(entity,new AttackDistance(){Distance = abilityAuthoring.attackDistance});
                AddComponent(entity,new AttackEntity(){Entity = Entity.Null});
                AddComponent(entity,new AttackProperties()
                {
                    CooldownSeconds = math.max(0f, abilityAuthoring.attackCooldown),
                    AttackPrefab = GetEntity(abilityAuthoring.attackPrefab,TransformUsageFlags.Dynamic),
                    FirePointOffset = abilityAuthoring.firePointOffset
                });
                AddComponent(entity, new AttackCoolDown { NextAttackTime = 0d });
            }
        }
    }
}
