using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;

namespace 通用
{
    public  struct RequestSpawnUnitRPC : IRpcCommand
    {
        public float3 Position;
    }
    
}