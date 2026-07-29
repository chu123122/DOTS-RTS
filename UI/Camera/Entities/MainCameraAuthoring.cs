using Unity.Entities;
using UnityEngine;

namespace 客户端
{
    public class MainCameraAuthoring : MonoBehaviour
    {
        public class MainCameraBaker : Baker<MainCameraAuthoring>
        {
            public override void Bake(MainCameraAuthoring authoring)
            {
                if(IsServer())return;
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponentObject(entity, new MainCameraComponents()
                {
                    Value = Camera.main
                });
                AddComponent(entity,new MainCameraTag());
            }
        }
    }
}