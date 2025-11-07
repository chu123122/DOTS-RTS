using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using Utils;

namespace _RePlaySystem.Base
{
    [WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
    public partial class CommandRecordingSystem : SystemBase
    {
        protected override void OnCreate()
        {
            RequireForUpdate<NetworkTime>();
            
            Entity entity = EntityManager.CreateEntity();
            RtsReplayComponent replayComponent = new RtsReplayComponent()
            {
                CommandHistory = new NativeList<PlayerInputCommand>(1000,Allocator.Persistent),
                CurrentReplayTick = new NetworkTick(0),
            };

            EntityManager.AddComponentData(entity, replayComponent);
        }

        protected override void OnUpdate()
        {
            var networkTime = SystemAPI.GetSingleton<NetworkTime>();
            foreach (var replayComponent in SystemAPI.Query<RefRW<RtsReplayComponent>>())
            {
                replayComponent.ValueRW.CurrentReplayTick = networkTime.ServerTick;
            }
        }
       

        protected override void OnDestroy()
        {
            foreach (var replayComponent in SystemAPI.Query<RefRW<RtsReplayComponent>>())
            {
                foreach (var command in  replayComponent.ValueRW.CommandHistory)
                {
                    if(command.Units.IsCreated)
                        command.Units.Dispose();
                }
               
                replayComponent.ValueRW.CommandHistory.Dispose();
            }
        }
    }
}