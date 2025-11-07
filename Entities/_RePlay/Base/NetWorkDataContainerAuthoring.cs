using Unity.Entities;
using Unity.NetCode;
using UnityEngine;
using UnityEngine.Serialization;

namespace _RePlaySystem.Base
{
    public class NetWorkDataContainerAuthoring:MonoBehaviour
    {
        public class Baker:Baker<NetWorkDataContainerAuthoring>
        {
            public override void Bake(NetWorkDataContainerAuthoring authoring)
            {
                var container = GetEntity(TransformUsageFlags.None);
                AddComponent(container,new NetWorkDataContainer()
                {
                    Id = 2,
                    ReplayDataComponent=new RtsReplayDataComponent()
                    {
                        CommandDataHistory = "",
                        CurrentReplayTick = new NetworkTick(0)
                    }
                });
            }
        }
    }
     [GhostComponent]
    public struct NetWorkDataContainer : IComponentData
    {
        [GhostField]public int Id; 
        [GhostField]public RtsReplayDataComponent ReplayDataComponent;
    }
}