using System;
using System.Collections.Generic;
using DefaultNamespace;
using Entities._Common.SpawnEntityRpc;
using Unity.Entities;
using _RePlaySystem.Base;

namespace Entities._Common
{
    [WorldSystemFilter(WorldSystemFilterFlags.LocalSimulation)]
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

        public Entity GetEntityByIndexInClientWorld(int localId)
        {
            if (_entitiesInClientWorld.TryGetValue(localId, out Entity entityInDic))
                return entityInDic;
            foreach (var (localInstance, entity) in
                     SystemAPI.Query<RefRO<LocalInstance>>().WithEntityAccess())
            {
                if (localInstance.ValueRO.Id == localId)
                {
                    _entitiesInClientWorld.Add(localId, entity);
                    return entity;
                }
            }

            throw new InvalidOperationException($"无法查找到对应id:{localId}的Entity在本地世界");
        }
    }
}
