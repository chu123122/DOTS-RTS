using Unity.Physics;

namespace RTS.Gameplay.Physics
{
/// <summary>
/// Unity Physics 只承担场景查询和非单位动态交互。
/// Crowd Unit 之间的 locomotion 接触由 Crowd Physics 独占。
/// </summary>
public static class CrowdQueryCollisionFilters
{
    public const uint Ground = 1u << 0;
    public const uint Unit = 1u << 1;
    public const uint Obstacle = 1u << 2;

    public static CollisionFilter UnitOverlap => new CollisionFilter
    {
        BelongsTo = ~0u,
        CollidesWith = Unit,
        GroupIndex = 0
    };

    public static CollisionFilter ObstacleOverlap => new CollisionFilter
    {
        BelongsTo = ~0u,
        CollidesWith = Obstacle,
        GroupIndex = 0
    };
}
}
