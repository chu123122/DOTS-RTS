using Unity.Entities;

public struct DestroyOnTimer : IComponentData
{
    public float Value;
}

public struct DestroyAtTime : IComponentData
{
    public double Value;
}
