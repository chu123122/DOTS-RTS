using Unity.Entities;
using UnityEngine;

namespace TMG.NFE_Tutorial
{
    public class AbilityMoveSpeedAuthoring : MonoBehaviour
    {
        public float abilityMoveSpeed;

        public class AbilityMoveSpeedBaker : Baker<AbilityMoveSpeedAuthoring>
        {
            public override void Bake(AbilityMoveSpeedAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent(entity, new AbilityMoveSpeed { Value = authoring.abilityMoveSpeed });
            }
        }
    }
}