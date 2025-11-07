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

        public NetCodeConfig netCodeConfig;
        private int SimulationTickRate => netCodeConfig.ClientServerTickRate.SimulationTickRate;

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
                    CooldownTickCount = (uint) (abilityAuthoring.attackCooldown*abilityAuthoring.SimulationTickRate),
                    AttackPrefab = GetEntity(abilityAuthoring.attackPrefab,TransformUsageFlags.Dynamic),
                    FirePointOffset = abilityAuthoring.firePointOffset
                });
                AddBuffer<AttackCoolDown>(entity);
            }
        }
    }
}