using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;

namespace Entities.Building
{
    public enum BuildingType
    {
        Barracks,
    }
    public struct RequestSpawnBuildingRPC :  IRpcCommand
    {
        public float3 Position;
        public BuildingType BuildingType;
    }
}