/*using DefaultNamespace;
using Newtonsoft.Json;
using QFramework;
using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using UnityEngine;
using Utils;
using JsonSerializer = System.Text.Json.JsonSerializer;

namespace _RePlaySystem.Base
{
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.LocalSimulation)]
    public partial class RequestReplayComponentSystem : ServiceSystemBase<RequestReplayComponentSystem>,ICanSendEvent
    {
        public RtsReplayComponent ReplayComponent;

        protected override void OnCreate()
        {
            base.OnCreate();
            RequireForUpdate<NetworkId>();
        }

        protected override void OnUpdate()
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (var (_, responseRpc, entity)in SystemAPI.Query
                     <RefRO<ReceiveRpcCommandRequest>,
                         RefRO<ResponseReplayComponentRpc>>().
                         WithEntityAccess())
            {
                ecb.DestroyEntity(entity);
                RtsReplayComponent replayComponent=DeSerializeReplayComponent(responseRpc.ValueRO.RtsReplayComponent);
                ReplayComponent = replayComponent;
            }

            ecb.Playback(EntityManager);
        }


        public void SendRequestRePlayComponentRpc()
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            var rpcEntity = ecb.CreateEntity();

            Entity networkIdEntity = EntityManager.CreateEntityQuery(typeof(NetworkId)).GetSingletonEntity();
            ecb.AddComponent(rpcEntity, new SendRpcCommandRequest()
                { TargetConnection = networkIdEntity });
            ecb.AddComponent(rpcEntity, new RequestReplayComponentRpc());
            ecb.Playback(EntityManager);
        }

        private RtsReplayComponent DeSerializeReplayComponent(RtsReplayDataComponent replayData)
        {
            JsonConvert.DefaultSettings = () => new JsonSerializerSettings {
                Converters = { new Float3Converter(), new NetworkTickConverter(),new FixedString128BytesConverter()}
            };
            string json = replayData.CommandDataHistory.ToString() == ""
                ? "[]" : replayData.CommandDataHistory.ToString();
            PlayerInputCommandData[] commandDatas =
                JsonConvert.DeserializeObject<PlayerInputCommandData[]>(json);
            PlayerInputCommand[] commands = PlayerInputCommandData.ToCommands(commandDatas);
            
            var nativeArray = new NativeArray<PlayerInputCommand>(commands, Allocator.Temp);
            var nativeList = new NativeList<PlayerInputCommand>(Allocator.Persistent);
            nativeList.AddRange(nativeArray);
            
            return new RtsReplayComponent()
            {
                CommandHistory = nativeList,
                CurrentReplayTick = replayData.CurrentReplayTick
            };
        }

        public IArchitecture GetArchitecture()
        {
            return MainGameArchitecture.Interface;
        }
    }
}*/