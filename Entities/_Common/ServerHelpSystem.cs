using DefaultNamespace;
using Unity.Entities;

namespace Entities._Common
{
    [WorldSystemFilter(WorldSystemFilterFlags.ServerSimulation)]
    public partial class ServerHelpSystem:ServiceSystemBase<ServerHelpSystem>
    {
        public World ServerWorld;
        protected override void OnCreate()
        {
            base.OnCreate();
            ServerWorld = World;
        }

        protected override void OnUpdate()
        {
        }
    }
}