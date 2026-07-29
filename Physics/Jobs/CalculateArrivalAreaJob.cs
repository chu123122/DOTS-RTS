using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using RTS.Unit.Components;
using RTS.Unit.FlowField;
using RTS.Unit.FlowField.Diagnostics;

namespace RTS.Unit.FlowField.Jobs
{

/// <summary>
/// 读取单位运行时碰撞体的地面投影尺寸，并换算当前队伍需要的流场到达区域半径。
/// </summary>
[BurstCompile]
public partial struct CalculateUnitCollisionFootprintJob : IJobEntity
{
    [ReadOnly] public ComponentLookup<PhysicsCollider> PhysicsColliderLookup;
    public float FallbackCellSize;
    public NativeArray<float2> CollisionFootprints;

    public void Execute(
        Entity entity,
        [EntityIndexInQuery] int entityIndex,
        in LocalTransform transform)
    {
        float2 footprint = new float2(FallbackCellSize);

        if (PhysicsColliderLookup.TryGetComponent(entity, out PhysicsCollider physicsCollider) &&
            physicsCollider.IsValid)
        {
            float uniformScale = math.max(math.abs(transform.Scale), 0.0001f);
            var colliderTransform = new RigidTransform(transform.Rotation, float3.zero);
            Aabb aabb = physicsCollider.Value.Value.CalculateAabb(colliderTransform, uniformScale);
            float3 size = aabb.Max - aabb.Min;
            footprint = math.max(size.xz, new float2(0.0001f));
        }

        CollisionFootprints[entityIndex] = footprint;
    }
}
}
