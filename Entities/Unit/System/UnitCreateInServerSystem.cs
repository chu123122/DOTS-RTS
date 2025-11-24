using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using Unity.Transforms;
using UnityEngine;
using 通用;

namespace Entities.Unit.System
{
    [WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
    public partial class UnitCreateInServerSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (var (
                         spawnUnitRPC
                         , receiveRpc,
                         entity)in
                     SystemAPI.Query<
                         RefRO<RequestSpawnUnitRPC>,
                         RefRO<ReceiveRpcCommandRequest>>().
                         WithEntityAccess())
            {
                ecb.DestroyEntity(entity);
                var unitPrefabQuery = EntityManager.CreateEntityQuery(typeof(RtsPrefabs));
                Entity unitPrefabEntity = unitPrefabQuery.GetSingleton<RtsPrefabs>().Entity;

                var unit = ecb.Instantiate(unitPrefabEntity);
                var clientId = SystemAPI.GetComponent<NetworkId>(receiveRpc.ValueRO.SourceConnection).Value;
                LocalTransform localTransform = LocalTransform.FromPosition(spawnUnitRPC.ValueRO.Position);
                localTransform.Scale = 0.5f;
                ecb.SetComponent(unit, new GhostOwner { NetworkId = clientId });
                ecb.SetComponent(unit, localTransform);
            }

            ecb.Playback(EntityManager);
        }
    }
}