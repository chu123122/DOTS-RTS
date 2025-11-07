using System.Collections.Generic;
using DefaultNamespace;
using Entities._Common;
using Entities._Common.SpawnEntityRpc;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Transforms;
using UnityEngine;
using Utils;
using 通用;

namespace _RePlaySystem.Base
{
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.LocalSimulation)]
    public partial class RequestCommandRpcSystem : ServiceSystemBase<RequestCommandRpcSystem>
    {
        private readonly Dictionary<string, BlobAssetReference<SelectedUnitsBlob>> _blobAssetCache
            = new();

        protected override void OnUpdate()
        {
        }

        public void SendInputCommand(PlayerInputCommand command)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            var rpc = ecb.CreateEntity();
            ecb.AddComponent(rpc, new SendRpcCommandRequest());
            ecb.AddComponent(rpc, new RequestSendCommandRpc()
            {
                CommandData = command.ToCommandData(),
            });
            ecb.Playback(EntityManager);

            //DebugSystem.Log(command.ToString(), "乐酌");
            //获取服务端最新的输入存储
           // this.GetService<RequestReplayComponentSystem>().SendRequestRePlayComponentRpc();
        }

        public void ExecutedInputCommandInLocal(PlayerInputCommand command)
        {
            DebugSystem.Log($"{command.Type.ToString()}", "乐酌");
            if (command.Type == InputCommandType.Move)
            {
                foreach (int ghostId in command.Units.Value.UnitEntityIndex.ToArray())
                {
                    Entity entity = this.GetService<ClientHelpSystem>().GetEntityByIndexInLocalWorld(ghostId);
                    EntityManager.SetComponentData(entity, new UnitMoveTargetPosition()
                    {
                        Value = command.Position
                    });
                    EntityManager.SetComponentData(entity, new UnitSelected()
                    {
                        Value = true
                    });
                }
            }
            else if (command.Type == InputCommandType.Create)
            {
                var ecb = new EntityCommandBuffer(Allocator.Temp);
                var unitPrefabQuery = EntityManager.CreateEntityQuery(typeof(RtsLocalPrefabs));
                Entity unitPrefabEntity = unitPrefabQuery.GetSingleton<RtsLocalPrefabs>().Entity;

                var unit = ecb.Instantiate(unitPrefabEntity);
                int[] ids = command.Units.Value.UnitEntityIndex.ToArray();
                LocalTransform localTransform = LocalTransform.FromPosition(command.Position);
                localTransform.Scale = 0.5f;
                
                ecb.AddComponent(unit, new LocalInstance
                    { Id = command.Units.Value.UnitEntityIndex[0] }); //TODO:待后续修改
                ecb.SetComponent(unit, localTransform);

                ecb.Playback(World.DefaultGameObjectInjectionWorld.EntityManager);
            }
        }

        public PlayerInputCommand CreateInputCommand(InputCommandType type, float3 position,
            List<int> ghostIds, int playerId = 0)
        {
            var networkTime = SystemAPI.GetSingleton<NetworkTime>();
            // 创建缓存 key
            string cacheKey = string.Join(",", ghostIds);
            // 尝试从缓存获取
            if (!_blobAssetCache.TryGetValue(cacheKey, out var units))
            {
                units = CreateBlobAssetReference(ghostIds.ToArray());
                _blobAssetCache[cacheKey] = units;
            }

            return new PlayerInputCommand()
            {
                Tick = networkTime.ServerTick,
                Type = type,
                PlayerNetWorkId = playerId,
                Position = position,
                Units = units
            };
        }

        public BlobAssetReference<SelectedUnitsBlob> CreateBlobAssetReference(int[] selectedEntities)
        {
            BlobBuilder builder = new BlobBuilder(Allocator.Temp);

            ref SelectedUnitsBlob root = ref builder.ConstructRoot<SelectedUnitsBlob>();
            BlobBuilderArray<int> array = builder.Allocate(ref root.UnitEntityIndex, selectedEntities.Length);
            for (int i = 0; i < selectedEntities.Length; i++)
            {
                array[i] = selectedEntities[i];
            }

            var blobAssetReference = builder.CreateBlobAssetReference<SelectedUnitsBlob>(Allocator.Persistent);
            builder.Dispose();
            return blobAssetReference;
        }

        protected override void OnDestroy()
        {
            base.OnDestroy();
            foreach (var reference in _blobAssetCache.Values)
            {
                if (reference.IsCreated)
                {
                    reference.Dispose();
                }
            }

            _blobAssetCache.Clear();
        }
    }
}