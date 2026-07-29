using Unity.Entities;
using UnityEngine;
using UnityEngine.Serialization;

namespace Entities._Common.Systems.Track
{
    public class TrackAbilityAuthoring:MonoBehaviour
    {
        public float trackDistance;
        public class Baker:Baker<TrackAbilityAuthoring>
        {
            public override void Bake(TrackAbilityAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent(entity,new TrackDistance(){Distance = authoring.trackDistance});
                AddComponent(entity,new TrackEntity(){Entity = Entity.Null});
            }
        }
    }
}