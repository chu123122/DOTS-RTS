using DefaultNamespace;
using Entities._Common;
using Entities._Common.SpawnEntityRpc;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;
using Utils;
using _RePlaySystem.Base; // 引用 RTSCommandType

namespace _RePlaySystem.Base
{
    // 这个系统运行在客户端，负责发送 RPC，同时也是录制本地操作的最佳时机
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.LocalSimulation)]
    public partial class RequestCommandRpcSystem : ServiceSystemBase<RequestCommandRpcSystem>
    {
        protected override void OnCreate()
        {
            base.OnCreate();
            // 需要获取录制状态
            RequireForUpdate<ReplaySystemState>();
        }

        protected override void OnUpdate()
        {
            // 不需要每帧逻辑
        }

        public void SendInputCommand(InputCommandType type, float3 position, int playerId = 0)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            
            // 1. 发送 RPC (保持不变)
            var rpcEntity = ecb.CreateEntity();
            ecb.AddComponent(rpcEntity, new SendRpcCommandRequest());
            ecb.AddComponent(rpcEntity, new RequestSendCommandRpc()
            {
                CommandData = new PlayerInputCommandData
                {
                    Type = type,
                    Position = position,
                    PlayerNetWorkId = playerId,
                    Units = default 
                },
            });

            ecb.Playback(EntityManager);
            ecb.Dispose();

            // 2. 【新增】客户端本地录制逻辑
            // 检查当前是否处于录制模式
            if (SystemAPI.TryGetSingletonEntity<ReplaySystemState>(out var stateEntity))
            {
                var state = SystemAPI.GetComponent<ReplaySystemState>(stateEntity);
                
                if (state.IsRecording)
                {
                    // 获取客户端世界的 Buffer
                    var buffer = SystemAPI.GetBuffer<ReplayCommandElement>(stateEntity);
                    
                    // 计算相对时间
                    double currentOffset = SystemAPI.Time.ElapsedTime - state.RecordingStartTime;

                    buffer.Add(new ReplayCommandElement
                    {
                        TimeOffset = currentOffset,
                        Type = (RTSCommandType)type, // 确保 Enum 对应
                        Position = position,
                        UnitCount = 1
                    });

                    Debug.Log($"[Client Recording] Saved {(RTSCommandType)type} at {currentOffset:F2}s. Buffer Count: {buffer.Length}");
                }
            }
        }
    }
}