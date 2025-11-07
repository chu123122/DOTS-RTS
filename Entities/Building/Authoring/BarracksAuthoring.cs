using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using 通用;

namespace Entities.Building.Authoring
{
    public class BarracksAuthoring : BuildingAuthoring 
    {
        private class BarracksAuthoringBaker : Baker<BarracksAuthoring>
        {
            
            public override void Bake(BarracksAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent<BasicBuildingTag>(entity);
                AddComponent<NewBasicBuildingTag>(entity);
                AddComponent<RtsTeam>(entity);
                AddComponent(entity, new BuildingHpCompoent
                {
                    BuildingHpValue = authoring.maxHp
                });
                AddComponent(entity, new BuildingSizeComponent 
                {
                    BuildingSize= authoring.size
                });
            }
        }
    }
}