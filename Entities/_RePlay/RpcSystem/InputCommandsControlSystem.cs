using DefaultNamespace;
using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using UnityEngine;

namespace _RePlaySystem.Base
{
    [WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
    public partial class InputCommandsControlSystem : SystemBase, IServiceSystemLocator
    {
        protected override void OnUpdate()
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (var (_, requestSendCommandRpc, requestEntity) in
                     SystemAPI.Query<RefRO<ReceiveRpcCommandRequest>,
                         RequestSendCommandRpc>().WithEntityAccess())
            {
                ecb.DestroyEntity(requestEntity);

                var command = requestSendCommandRpc.CommandData.ToCommand();
                SendInputCommand(command);
            }

            ecb.Playback(EntityManager);
        }


        private void SendInputCommand(PlayerInputCommand command)
        {
            var serverWorld = World;
            var replayQuery = serverWorld.EntityManager.CreateEntityQuery(typeof(NetWorkDataContainer));
            Entity netWorkDataContainerEntity = replayQuery.GetSingletonEntity();
            NetWorkDataContainer netWorkDataContainer =
                EntityManager.GetComponentData<NetWorkDataContainer>(netWorkDataContainerEntity);
            RtsReplayComponent replayComponent = netWorkDataContainer.ReplayDataComponent.ToReplayComponent();

            if (command.Type == InputCommandType.Create)
            {
                replayComponent.CommandHistory.Add(command);
                int ghostInstanceId = netWorkDataContainer.Id;
                EntityManager.SetComponentData(netWorkDataContainerEntity, new NetWorkDataContainer()
                {
                    Id = ghostInstanceId + 1, 
                    ReplayDataComponent = replayComponent.ToRePlayDataComponent()
                });
            }
            else
            {
                replayComponent.CommandHistory.Add(command);
                EntityManager.SetComponentData(netWorkDataContainerEntity, new NetWorkDataContainer()
                {
                    Id = netWorkDataContainer.Id,
                    ReplayDataComponent = replayComponent.ToRePlayDataComponent()
                });
            }
            Debug.Log($"记录指令：{command.ToString()}");

            replayComponent.CommandHistory.Dispose();
        }
    }
}