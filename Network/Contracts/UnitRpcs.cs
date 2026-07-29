using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;

namespace RTS.Unit.Components
{
    public  struct RequestSpawnUnitRPC : IRpcCommand
    {
        public float3 Position;
    }

    public struct RequestMoveOrderRPC : IRpcCommand
    {
        public float3 TargetPosition;
    }
    
}
