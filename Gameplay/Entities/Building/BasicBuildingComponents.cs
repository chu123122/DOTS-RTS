using Unity.Entities;
using Unity.Mathematics;

namespace Entities.Building
{
    public struct BasicBuildingTag : IComponentData
    {
    }

    public struct NewBasicBuildingTag : IComponentData
    {
    }

    public struct BuildingHpCompoent : IComponentData
    {
        public float BuildingHpValue;
    }

    public struct BuildingSizeComponent : IComponentData
    {
        public float2 BuildingSize;
    }
}