using Unity.Entities;

public struct TrackDistance : IComponentData
{
    public float Distance;
}

public struct TrackEntity : IComponentData
{
    public Entity Entity;
}