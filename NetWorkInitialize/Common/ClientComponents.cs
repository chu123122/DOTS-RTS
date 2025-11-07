using Unity.Entities;
using 通用;

namespace 中间值
{

    public struct ClientTeamRequest : IComponentData
    {
        public TeamType Value;
        //
    }
}