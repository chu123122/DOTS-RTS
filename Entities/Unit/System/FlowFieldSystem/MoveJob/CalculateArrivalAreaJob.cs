using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;

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

/// <summary>
/// 根据所有单位碰撞体的 XZ 投影总面积以及最大单体跨度，计算正方形到达区域的格子半径。
/// </summary>
[BurstCompile]
public struct CalculateArrivalAreaJob : IJob
{
    [ReadOnly] public NativeArray<float2> CollisionFootprints;
    public float CellSize;
    public NativeReference<int> ArrivalEnterDistance;

    public void Execute()
    {
        float totalFootprintArea = 0f;
        float maximumFootprintSpan = 0f;

        for (int i = 0; i < CollisionFootprints.Length; i++)
        {
            float2 footprint = CollisionFootprints[i];
            totalFootprintArea += footprint.x * footprint.y;
            maximumFootprintSpan = math.max(maximumFootprintSpan, math.cmax(footprint));
        }

        float safeCellSize = math.max(CellSize, 0.0001f);
        float requiredWorldSide = math.max(math.sqrt(totalFootprintArea), maximumFootprintSpan);
        int requiredCellDiameter = math.max(1, (int)math.ceil(requiredWorldSide / safeCellSize));

        // 八邻域 Integration 半径 r 在开放区域对应 (2r + 1) × (2r + 1) 个格子。
        ArrivalEnterDistance.Value = (int)math.ceil((requiredCellDiameter - 1) * 0.5f);
    }
}
