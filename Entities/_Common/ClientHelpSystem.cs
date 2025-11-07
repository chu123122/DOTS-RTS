using System;
using System.Collections.Generic;
using _RePlaySystem.Base;
using DefaultNamespace;
using Entities._Common.SpawnEntityRpc;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using 通用;

namespace Entities._Common
{
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation)]
    public partial class ClientHelpSystem : ServiceSystemBase<ClientHelpSystem>
    {
        private readonly Dictionary<int, Entity> _entitiesInClientWorld = new();

        private readonly Dictionary<int, Entity> _entitiesInLocalWorld = new();

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

        public Entity GetEntityByIndexInLocalWorld(int ghostId)
        {
            if (_entitiesInLocalWorld.TryGetValue(ghostId, out Entity entityInDic))
                return entityInDic;
            foreach (var (ghostInstance, entity) in SystemAPI.Query<RefRO<LocalInstance>>().WithEntityAccess())
            {
                if (ghostInstance.ValueRO.Id == ghostId)
                {
                    _entitiesInLocalWorld.Add(ghostId, entity);
                    return entity;
                }
            }

            throw new InvalidOperationException($"无法查找到对应id:{ghostId}的Entity在本地世界");
        }
    }
}