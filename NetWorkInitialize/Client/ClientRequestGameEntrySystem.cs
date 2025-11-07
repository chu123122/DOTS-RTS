using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;
using 通用;

namespace 中间值
{
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation|WorldSystemFilterFlags.ThinClientSimulation)]
    public partial struct ClientRequestGameEntrySystem:ISystem
    {
        private EntityQuery _pendingNetworkQuery;
        public void OnCreate(ref SystemState state)
        {
            var builder = new EntityQueryBuilder(Allocator.Temp).WithAll<NetworkId>().WithNone<NetworkStreamInGame>();
            _pendingNetworkQuery = state.GetEntityQuery(builder);
            state.RequireForUpdate(_pendingNetworkQuery);
            state.RequireForUpdate<ClientTeamRequest>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var requestTeam = SystemAPI.GetSingleton<ClientTeamRequest>().Value;
           
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            var pendingNetworkIds = _pendingNetworkQuery.ToEntityArray(Allocator.Temp);

            foreach (var pendingNetworkId in pendingNetworkIds)
            {
                ecb.AddComponent<NetworkStreamInGame>(pendingNetworkIds);
                var requestTeamRPCEntity = ecb.CreateEntity();
                ecb.AddComponent(requestTeamRPCEntity,new RtsTeamRequest() {Value = requestTeam });
                ecb.AddComponent(requestTeamRPCEntity,new SendRpcCommandRequest(){TargetConnection = pendingNetworkId});
                
            }
            
            ecb.Playback(state.EntityManager);
        }
    }
}