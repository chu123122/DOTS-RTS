using Entities.Building.Authoring;
using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using Unity.Transforms;
using UnityEngine;
using 通用;

namespace Entities.Building.System
{
    // 该系统负责在服务器端接收到 RequestSpawnBuildingRPC 时创建兵营（Barracks）
    [WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
    public partial class BarracksCreateInServerSystem : SystemBase
    {
        // OnUpdate 每帧在服务器模拟中被调用
        protected override void OnUpdate()
        {
            // 创建一个实体命令缓冲区（Entity Command Buffer），用于记录实体创建和销毁的操作
            var ecb = new EntityCommandBuffer(Allocator.Temp);

            // 查询具有 RequestSpawnBuildingRPC 和 ReceiveRpcCommandRequest 组件的实体
            foreach (var (spawnBuildingRPC, receiveRpc, entity) in SystemAPI
                         .Query<RefRO<RequestSpawnBuildingRPC>, ReceiveRpcCommandRequest>()
                         .WithEntityAccess())
            {
                // 检查请求的建筑类型是否为兵营（Barracks）
                if (spawnBuildingRPC.ValueRO.BuildingType == BuildingType.Barracks)
                {
                    // 销毁原始请求的实体（可能是一个临时的占位符实体）
                    ecb.DestroyEntity(entity);

                    // 获取兵营预制件实体
                    Entity barracksPrefabEntity = EntityManager.CreateEntityQuery(typeof(BarracksPerfabComponent))
                        .GetSingleton<BarracksPerfabComponent>().BuildingEntity;
                    
                    // 在缓冲区中实例化兵营预制件
                    var barracks = ecb.Instantiate(barracksPrefabEntity);

                    // 获取源连接的客户端 ID
                    var clientId = SystemAPI.GetComponent<NetworkId>(receiveRpc.SourceConnection).Value;
                    
                    // 设置实例化的兵营实体的 GhostOwner 组件，以标识所属客户端
                    ecb.SetComponent(barracks, new GhostOwner { NetworkId = clientId });

                    // 设置兵营的初始位置
                    ecb.SetComponent(barracks, LocalTransform.FromPosition(spawnBuildingRPC.ValueRO.Position));
                }
            }

            // 执行缓冲区中的所有命令
            ecb.Playback(EntityManager);

            // 释放 ECB 资源
            ecb.Dispose();
        }
    }
}
