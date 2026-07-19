using System;
using System.Collections.Generic;
using DefaultNamespace;
using Entities._Common.SpawnEntityRpc;
using Unity.Entities;
using Unity.NetCode;

namespace Entities._Common
{
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation)]
    public partial class ClientHelpSystem : ServiceSystemBase<ClientHelpSystem>
    {
        private readonly Dictionary<int, Entity> _entitiesInClientWorld = new();

        protected override void OnUpdate()
        {
        }

        public void SendSpawnCreateEntityRpc(ICreateEntityRpc createEntityRpc)
        {
            createEntityRpc.CreateEntityRpc();
        }

        public Entity GetEntityByIndexInClientWorld(int ghostId)
        {
            if (_entitiesInClientWorld.TryGetValue(ghostId, out Entity entityInDic))
                return entityInDic;
            foreach (var (ghostInstance, entity) in SystemAPI.Query<RefRO<GhostInstance>>().WithEntityAccess())
            {
                if (ghostInstance.ValueRO.ghostId == ghostId)
                {
                    _entitiesInClientWorld.Add(ghostId, entity);
                    return entity;
                }
            }

            throw new InvalidOperationException($"无法查找到对应id:{ghostId}的Entity在本地世界");
        }
    }
}
