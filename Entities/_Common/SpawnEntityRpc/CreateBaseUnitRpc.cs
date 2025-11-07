using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using 通用;

namespace Entities._Common.SpawnEntityRpc
{
    public class CreateBaseUnitRpc:ICreateEntityRpc
    {
        private readonly float3 _position;
        public CreateBaseUnitRpc(float3 position)
        {
            _position=position;
        }

        public void CreateEntityRpc()
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            var rpcEntity=ecb.CreateEntity();
            ecb.AddComponent(rpcEntity,new SendRpcCommandRequest());
            ecb.AddComponent(rpcEntity,new RequestSpawnUnitRPC()
            {
                Position = _position,
            });
            ecb.Playback(World.DefaultGameObjectInjectionWorld.EntityManager);
        }
    }
}