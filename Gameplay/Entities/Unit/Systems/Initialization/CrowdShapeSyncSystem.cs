using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using RTS.Unit.Components;
using RTS.Unit.FlowField.Systems;

namespace RTS.Unit.Systems.Initialization
{
internal struct CrowdShapeSourceState : IComponentData
{
    public BlobAssetReference<Collider> Collider;
    public float UniformScale;
}

internal static class CrowdShapeAdapter
{
    internal static CrowdShapeSourceState CaptureSource(
        PhysicsCollider collider,
        LocalTransform transform) =>
        new CrowdShapeSourceState
        {
            Collider = collider.Value,
            UniformScale = math.abs(transform.Scale)
        };

    internal static float CalculateRadius(
        PhysicsCollider collider,
        LocalTransform transform)
    {
        if (!collider.IsValid)
            return 0f;

        float scale = math.max(math.abs(transform.Scale), 0.0001f);
        var colliderTransform = new RigidTransform(
            transform.Rotation,
            float3.zero);
        Aabb aabb = collider.Value.Value.CalculateAabb(
            colliderTransform,
            scale);
        return math.max(
            0f,
            math.cmax((aabb.Max - aabb.Min).xz) * 0.5f);
    }

    internal static uint NextVersion(uint current) =>
        current == uint.MaxValue ? 1u : current + 1u;
}

/// <summary>
/// 同步 Unity Physics collider/scale 到 Crowd 的圆形碰撞产品。
/// 旋转和位置不是 shape source；只有 collider 或 scale 变化才推进版本。
/// </summary>
[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateAfter(typeof(InitializeUnitSystem))]
[UpdateBefore(typeof(FlowFieldBakeSystem))]
public partial class CrowdShapeSyncSystem : SystemBase
{
    protected override void OnUpdate()
    {
        var addSourceCommands = new EntityCommandBuffer(Allocator.Temp);
        foreach (var (shape, transform, entity) in
                 SystemAPI.Query<
                         RefRW<CrowdDiscShape>,
                         RefRO<LocalTransform>>()
                     .WithAll<CrowdQueryProxy>()
                     .WithNone<CrowdShapeSourceState>()
                     .WithEntityAccess())
        {
            PhysicsCollider collider =
                EntityManager.HasComponent<PhysicsCollider>(entity)
                    ? EntityManager.GetComponentData<PhysicsCollider>(entity)
                    : default;
            float radius = CrowdShapeAdapter.CalculateRadius(
                collider,
                transform.ValueRO);
            CrowdDiscShape value = shape.ValueRO;
            value.Radius = radius;
            value.Version = value.Version == 0
                ? 1u
                : CrowdShapeAdapter.NextVersion(value.Version);
            shape.ValueRW = value;
            addSourceCommands.AddComponent(
                entity,
                CrowdShapeAdapter.CaptureSource(
                    collider,
                    transform.ValueRO));
        }
        addSourceCommands.Playback(EntityManager);
        addSourceCommands.Dispose();

        foreach (var (shape, source, transform, entity) in
                 SystemAPI.Query<
                         RefRW<CrowdDiscShape>,
                         RefRW<CrowdShapeSourceState>,
                         RefRO<LocalTransform>>()
                     .WithAll<CrowdQueryProxy>()
                     .WithEntityAccess())
        {
            PhysicsCollider collider =
                EntityManager.HasComponent<PhysicsCollider>(entity)
                    ? EntityManager.GetComponentData<PhysicsCollider>(entity)
                    : default;
            CrowdShapeSourceState currentSource =
                CrowdShapeAdapter.CaptureSource(
                    collider,
                    transform.ValueRO);
            CrowdShapeSourceState previousSource = source.ValueRO;
            if (currentSource.Collider.Equals(previousSource.Collider) &&
                currentSource.UniformScale == previousSource.UniformScale)
                continue;

            CrowdDiscShape value = shape.ValueRO;
            value.Radius = CrowdShapeAdapter.CalculateRadius(
                collider,
                transform.ValueRO);
            value.Version = CrowdShapeAdapter.NextVersion(value.Version);
            shape.ValueRW = value;
            source.ValueRW = currentSource;
        }
    }
}
}
