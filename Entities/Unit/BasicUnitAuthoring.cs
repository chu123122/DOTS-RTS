using Unity.Entities;
using Unity.Mathematics;
using Unity.Rendering;
using UnityEngine;
using UnityEngine.Serialization;

namespace 通用
{
    public class BasicUnitAuthoring : MonoBehaviour
    {
        public float moveSpeed;

        public class Baker : Baker<BasicUnitAuthoring>
        {
            public override void Bake(BasicUnitAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent<BasicUnitTag>(entity);
                AddComponent<NewBasicUnitTag>(entity);
                AddComponent<RtsTeam>(entity);

                AddComponent<UnitMoveTargetPosition>(entity);
                AddComponent(entity, new Velocity { Value = new float3(0, 0, 0) });
                AddComponent(entity, new UnitMovementSettings { MaxForce = 20f, RotationSpeed = 10f });
                AddComponent(entity, new UnitMoveSpeed { Value = authoring.moveSpeed });
                AddComponent(entity, new UnitSelected { Value = false });

                AddComponent<IsUserUnitTag>(entity);
            }
        }
    }
}