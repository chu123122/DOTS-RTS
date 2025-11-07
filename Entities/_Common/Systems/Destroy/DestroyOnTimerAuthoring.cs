using Unity.Entities;
using Unity.NetCode;
using UnityEngine;
using UnityEngine.Serialization;

namespace TMG.NFE_Tutorial
{
    public class DestroyOnTimerAuthoring : MonoBehaviour
    {
        public float destroyOnTimer;

        public class DestroyOnTimerBaker : Baker<DestroyOnTimerAuthoring>
        {
            public override void Bake(DestroyOnTimerAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent(entity, new DestroyOnTimer { Value = authoring.destroyOnTimer });
            }
        }
    }
  
}