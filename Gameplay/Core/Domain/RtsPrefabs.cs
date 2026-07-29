using Unity.Entities;

namespace 通用
{
    public struct RtsPrefabs:IComponentData
    {
        public Entity Entity;
    }
    public struct RtsLocalPrefabs:IComponentData
    {
        public Entity Entity;
    }
}