using System;
using System.Collections.Generic;
using _RePlaySystem.Base;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using Unity.Transforms;
using UnityEngine;
using 通用;

namespace 服务器
{
    [WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
    public partial struct ServerProcessGameEntityRequestSystem:ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<RtsPrefabs>();
            var builder = new EntityQueryBuilder(Allocator.Temp).WithAll<RtsTeamRequest, ReceiveRpcCommandRequest>();
            state.RequireForUpdate(state.GetEntityQuery(builder));
        }

        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (var (teamRequest,requestSource,requestEntity) in SystemAPI.Query<RtsTeamRequest,ReceiveRpcCommandRequest>().WithEntityAccess())
            {
                ecb.DestroyEntity(requestEntity);
                ecb.AddComponent<NetworkStreamInGame>(requestSource.SourceConnection);

                var requestTeamType = teamRequest.Value;
                if (requestTeamType == TeamType.AutoAssign) requestTeamType = TeamType.Blue;

                var clientId = SystemAPI.GetComponent<NetworkId>(requestSource.SourceConnection).Value;

                Debug.Log($"服务器已分配客户端ID:{clientId}到{requestTeamType.ToString()}队伍上");
                
                var unit=ecb.CreateEntity();
                ecb.AddComponent(unit,new GhostOwner{NetworkId=clientId});
                ecb.AddComponent(unit,new RtsTeam{Value= requestTeamType});
                
                ecb.AppendToBuffer(requestSource.SourceConnection,new LinkedEntityGroup{Value = unit });
            } 
            ecb.Playback(state.EntityManager);
            
        }
    }
}