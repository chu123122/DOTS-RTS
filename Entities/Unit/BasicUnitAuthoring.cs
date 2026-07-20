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
        [Min(0f)] public float contactInverseMass = 1f;

        public class Baker : Baker<BasicUnitAuthoring>
        {
            public override void Bake(BasicUnitAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent<BasicUnitTag>(entity);
                AddComponent<IsNewCreatingTag>(entity);
                AddComponent<RtsTeam>(entity);

                AddComponent(entity, new Velocity { Value = new float3(0, 0, 0) });
                AddComponent(entity, new FlowArrivalState { IsSettled = false });
                AddComponent(entity, new UnitMovementSettings { MaxForce = 20f, RotationSpeed = 10f });
                AddComponent(entity, new UnitMoveSpeed { Value = authoring.moveSpeed });
                AddComponent(entity, new UnitContactBody
                {
                    InverseMass = math.max(0f, authoring.contactInverseMass)
                });
                AddComponent(entity, new UnitSelected { Value = false });

                AddComponent<IsUserUnitTag>(entity);
            }
        }
    }
}
