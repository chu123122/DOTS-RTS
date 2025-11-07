
using Unity.NetCode;

namespace _RePlaySystem.Base
{
    public struct RequestSendCommandRpc:IRpcCommand
    {
        public PlayerInputCommandData CommandData;
    }

    public struct RequestExecutedCommandRpc : IRpcCommand
    {
        public PlayerInputCommandData CommandData;
    }

  

  
}