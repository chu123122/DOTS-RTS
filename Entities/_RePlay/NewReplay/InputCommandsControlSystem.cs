using DefaultNamespace;
using Entities.Unit.System.FlowFieldSystem;
using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using 通用; 

namespace _RePlaySystem.Base
{
    [WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
    public partial class InputCommandsControlSystem : SystemBase
    {
        protected override void OnCreate()
        {
         //   RequireForUpdate<ReplaySystemState>();
            RequireForUpdate<RtsLocalPrefabs>();
        }

        protected override void OnUpdate()
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);

            foreach (var (_, requestSendCommandRpc, requestEntity) in
                     SystemAPI.Query<RefRO<ReceiveRpcCommandRequest>, RequestSendCommandRpc>()
                         .WithEntityAccess())
            {
                ecb.DestroyEntity(requestEntity);
                var commandData = requestSendCommandRpc.CommandData;

                // 只执行，不录制
                ExecuteCommandImmediate(ref ecb, commandData);
            }

            ecb.Playback(EntityManager);
            ecb.Dispose();
        }

        private void ExecuteCommandImmediate(ref EntityCommandBuffer ecb, PlayerInputCommandData command)
        {
            // 【补充】完整的执行逻辑，防止只有录制没有实际生成
            if (command.Type == InputCommandType.Create)
            {
                var prefabEntity = SystemAPI.GetSingleton<RtsLocalPrefabs>().Entity;
                var unit = ecb.Instantiate(prefabEntity);
                
                var transform = LocalTransform.FromPosition(command.Position);
                transform.Scale = 0.5f;
                ecb.SetComponent(unit, transform);
                
                // 确保加上本地标记 (如果需要)
                ecb.AddComponent(unit, new LocalInstance { Id = command.PlayerNetWorkId });
            }
            else if (command.Type == InputCommandType.Move)
            {
                // 服务端可能不跑 FlowField，但如果是 Host 模式，这里可能需要设置 FlowFieldGlobalTarget
                // 如果你是 Client-Server 分离且服务端纯运算，这里通常只需要转发
                // 但为了 Demo (Host模式)，我们可以设置一下：
                if (SystemAPI.TryGetSingletonEntity<FlowFieldGlobalTarget>(out var gridEntity))
                {
                    ecb.SetComponent(gridEntity, new FlowFieldGlobalTarget { TargetPosition = command.Position });
                    ecb.AddComponent<RecalculateFlowFieldTag>(gridEntity);
                }
            }
        }
    }
}