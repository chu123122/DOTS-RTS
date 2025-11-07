using Unity.Entities;
using UnityEngine;

namespace 客户端
{
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation)]
    public partial class InitializeMainCameraSystem:SystemBase
    {
        protected override void OnCreate()
        {
            RequireForUpdate<MainCameraTag>();
        }

        protected override void OnUpdate()
        {
            Enabled = false;
            var cameraEntity = SystemAPI.GetSingletonEntity<MainCameraTag>();
            EntityManager.SetComponentData(cameraEntity,new MainCameraComponents(){Value = Camera.main});
        }
    }
}