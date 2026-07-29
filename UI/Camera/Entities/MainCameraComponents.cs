using Unity.Entities;
using UnityEngine;

namespace 客户端
{
    public class MainCameraComponents:IComponentData
    {
        public Camera Value;
    }

    public struct MainCameraTag:IComponentData {}
}