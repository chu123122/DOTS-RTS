using Unity.Entities;
using Unity.NetCode;

public struct DestroyOnTimer : IComponentData
{
    public float Value;
}

[GhostComponent]
public struct DestroyAtTick : IComponentData
{
    [GhostField] public NetworkTick Value;
}