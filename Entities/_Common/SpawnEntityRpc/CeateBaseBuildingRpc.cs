using Entities.Building;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;
using 通用;

namespace Entities._Common.SpawnEntityRpc
{
    public class CreateBaseBuildingRpc : ICreateEntityRpc
    {
        private readonly float3 _position;

        private readonly BuildingType _buildingType;
        public CreateBaseBuildingRpc(float3 position, BuildingType buildingType)
        {
            _position = position;
            _buildingType = buildingType;
        }

        public void CreateEntityRpc()
        {
            Debug.Log("发送RPC");
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            var rpcEntity = ecb.CreateEntity();
            ecb.AddComponent(rpcEntity, new SendRpcCommandRequest());
            ecb.AddComponent(rpcEntity, new RequestSpawnBuildingRPC()
            {
                Position = _position,
                BuildingType = _buildingType,
            });
            ecb.Playback(World.DefaultGameObjectInjectionWorld.EntityManager);
        }
    }
}