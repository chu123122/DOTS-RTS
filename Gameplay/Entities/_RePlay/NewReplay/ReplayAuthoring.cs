using Unity.Entities;
using UnityEngine;

namespace _RePlaySystem.Base
{
    public class ReplayAuthoring : MonoBehaviour
    {
        public class Baker : Baker<ReplayAuthoring>
        {
            public override void Bake(ReplayAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.None);

                // 添加状态组件
                AddComponent(entity, new ReplaySystemState
                {
                    IsRecording = false, // 默认不录制
                    IsPlaying = false,
                    PlaybackIndex = 0
                });

                // 添加 Buffer 组件 (初始为空)
                AddBuffer<ReplayCommandElement>(entity);
            }
        }
    }
}