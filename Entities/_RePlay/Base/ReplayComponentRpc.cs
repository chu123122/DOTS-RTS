using Unity.Collections;
using Unity.Entities;
using Unity.NetCode;

namespace _RePlaySystem.Base
{
    public struct RequestReplayComponentRpc : IRpcCommand
    {
        
    }
    public struct ResponseReplayComponentRpc : IRpcCommand
    {
        public RtsReplayDataComponent RtsReplayComponent;
    }

  
}