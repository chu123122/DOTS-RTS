using Unity.Entities;
using UnityEngine;
using UnityEngine.Serialization;
using RTS.Unit.Components;
using 通用;

namespace RTS.Unit.Authoring
{
    public class RtsUnitPrefabsAuthoring:MonoBehaviour
    {
        public GameObject unitPrefab;
        public GameObject localUnitPrefab;
        class Baker:Baker<RtsUnitPrefabsAuthoring>
        {
            public override void Bake(RtsUnitPrefabsAuthoring authoring)
            {
                var prefabContainerEntity = GetEntity(TransformUsageFlags.None);
                AddComponent(prefabContainerEntity,new RtsPrefabs()
                {
                    Entity = GetEntity(authoring.unitPrefab,TransformUsageFlags.Dynamic)
                });
                
                var localPrefabContainerEntity = GetEntity(TransformUsageFlags.None);
                AddComponent(localPrefabContainerEntity,new RtsLocalPrefabs()
                {
                    Entity = GetEntity(authoring.localUnitPrefab,TransformUsageFlags.Dynamic)
                });
            }
        }
    }
}