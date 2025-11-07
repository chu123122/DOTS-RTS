using Test.BuildingSystem;
using Unity.Entities;
using UnityEngine;

namespace Entities.Building.Authoring
{
    public class BarracksPerfabAuthoring : MonoBehaviour
    {
        public GameObject buildingObject; 
        private class BuildingAuthoringBaker : Baker<BarracksPerfabAuthoring>
        {
            public override void Bake(BarracksPerfabAuthoring perfabAuthoring)
            {
                var entity = GetEntity(TransformUsageFlags.None);
                AddComponent(entity, new BarracksPerfabComponent
                {
                    BuildingEntity = GetEntity(perfabAuthoring.buildingObject, TransformUsageFlags.Dynamic),
                });
            }
        }
    }

    public struct BarracksPerfabComponent : IComponentData
    {
        public Entity BuildingEntity;
    }
}