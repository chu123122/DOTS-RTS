/*using System.Linq;
using Newtonsoft.Json;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;
using Utils;
using JsonSerializer = System.Text.Json.JsonSerializer;

namespace _RePlaySystem.Base
{
    [WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
    public partial struct ResponseReplayComponentSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<RtsReplayComponent>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (var (sourceRpc, _, entity) in
                     SystemAPI.Query<RefRO<ReceiveRpcCommandRequest>,
                         RefRO<RequestReplayComponentRpc>>().WithEntityAccess())
            {
                ecb.DestroyEntity(entity);
                var responseRpc = ecb.CreateEntity();
                var replayQuery = state.EntityManager.CreateEntityQuery(typeof(RtsReplayComponent));
                var replayComponent = replayQuery.GetSingleton<RtsReplayComponent>();
                RtsReplayDataComponent dataComponent = SerializedReplayComponent(replayComponent);
                ecb.AddComponent(responseRpc, new ResponseReplayComponentRpc()
                    { RtsReplayComponent = dataComponent });
                ecb.AddComponent(responseRpc, new SendRpcCommandRequest()
                    { TargetConnection = sourceRpc.ValueRO.SourceConnection });
               
            }
            ecb.Playback(state.EntityManager);
        }

        private RtsReplayDataComponent SerializedReplayComponent(RtsReplayComponent replayComponent)
        {
            JsonConvert.DefaultSettings = () => new JsonSerializerSettings {
                Converters = { new Float3Converter(), new NetworkTickConverter(),new FixedString128BytesConverter() }
            };
            
            PlayerInputCommand[] commands = replayComponent.CommandHistory.AsArray().ToArray();
            PlayerInputCommandData[] commandDatas=PlayerInputCommand.ToCommandDatas(commands);
            FixedString4096Bytes json = JsonConvert.SerializeObject(commandDatas);
            
            RtsReplayDataComponent replayDataComponent = new RtsReplayDataComponent()
            {
                CommandDataHistory = json,
                CurrentReplayTick = replayComponent.CurrentReplayTick
            };
            return replayDataComponent;
        }
    }
}*/