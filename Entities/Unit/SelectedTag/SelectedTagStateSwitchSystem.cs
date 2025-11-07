using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;

namespace 通用
{
    [UpdateInGroup(typeof(PresentationSystemGroup))]
    public partial struct SelectedTagStateSwitchSystem:ISystem
    {
        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (var (_,entity) in 
                    SystemAPI.Query<RefRO<SelectedTag>>().
                        WithEntityAccess().WithAll<Disabled>())
            {
                Entity parent=state.EntityManager.GetComponentData<Parent>(entity).Value;
                bool selected = state.EntityManager.GetComponentData<UnitSelected>(parent).Value;
                if(selected)ecb.RemoveComponent<Disabled>(entity);
            }

            foreach (var (_,entity) in 
                     SystemAPI.Query<RefRO<SelectedTag>>().
                         WithEntityAccess())
            {
                Entity parent=state.EntityManager.GetComponentData<Parent>(entity).Value;
                bool selected = state.EntityManager.GetComponentData<UnitSelected>(parent).Value;
                if(!selected)ecb.AddComponent<Disabled>(entity);
            }
            ecb.Playback(state.EntityManager);
        }
    }
}