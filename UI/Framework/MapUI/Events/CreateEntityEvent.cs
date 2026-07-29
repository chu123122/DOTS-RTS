using Unity.Entities;
using UnityEngine.Serialization;

namespace UI.MapUI.Events
{
    public struct CreateEntityEvent
    {
        public bool IsEnemy;
        public MapUIEntityType Type;
        public Entity Entity;
    }
}