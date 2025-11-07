using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;

namespace Entities._Common
{
    public class HealthPointAuthoring : MonoBehaviour
    {
        public int maxHp;

        public class Baker : Baker<HealthPointAuthoring>
        {
            public override void Bake(HealthPointAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent(entity, new HealthPointData()
                {
                    MaximumHp = authoring.maxHp, CurrentHp = authoring.maxHp
                });
                AddBuffer<DamageBufferElement>(entity);
                AddBuffer<DamageThisTick>(entity);
            }
        }
    }
}