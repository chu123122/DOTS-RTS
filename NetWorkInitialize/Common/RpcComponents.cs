using Unity.NetCode;

namespace 通用
{
   public struct RtsTeamRequest : IRpcCommand
   {
      public TeamType Value;
   }
}