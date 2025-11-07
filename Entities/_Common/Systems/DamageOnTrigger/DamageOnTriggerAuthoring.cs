using Unity.Entities;
using UnityEngine;
using UnityEngine.Serialization;

namespace TMG.NFE_Tutorial
{
    public class DamageOnTriggerAuthoring : MonoBehaviour
    {
         public int damageOnTrigger;

        public class DamageOnTriggerBaker : Baker<DamageOnTriggerAuthoring>
        {
            public override void Bake(DamageOnTriggerAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent(entity, new DamageOnTrigger { Value = authoring.damageOnTrigger });
                AddBuffer<AlreadyDamagedEntity>(entity);
            }   
        }
    }
}