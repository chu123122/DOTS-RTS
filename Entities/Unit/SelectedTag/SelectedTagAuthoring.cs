using Unity.Entities;
using Unity.Rendering;
using UnityEngine;
using UnityEngine.Serialization;

namespace 通用
{
    public class SelectedTagAuthoring:MonoBehaviour
    {
        class Baker:Baker<SelectedTagAuthoring>
        {
            public override void Bake(SelectedTagAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.None);
                AddComponent(entity,new Disabled());
                AddComponent(entity,new SelectedTag());
            }
        }
    }

    public struct SelectedTag:IComponentData
    {
        
    }
     
}