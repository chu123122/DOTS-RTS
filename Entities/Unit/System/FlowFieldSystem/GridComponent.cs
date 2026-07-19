using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

public struct FlowFieldCell
{
    public byte Cost; 
    public ushort IntegrationValue; 
    public byte BestDirectionIndex;
}

public struct FlowFieldGrid : IComponentData
{
    public float3 GridOrigin;   
    public int2 GridDimensions; 
    public float CellRadius;     
    
    public NativeArray<FlowFieldCell> Grid;
}
public struct FlowFieldSettings : IComponentData
{
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
}

public struct FlowFieldGlobalTarget : IComponentData
{
    public float3 TargetPosition; 
}

/// <summary>
/// 只在一份完整流场发布后递增，移动和可视化系统据此读取稳定快照。
/// </summary>
public struct FlowFieldRuntimeState : IComponentData
{
    public uint ActiveVersion;
}

public struct FlowFieldVisualizationSettings : IComponentData
{
    public bool Visible;
    public bool ShowCost;
    public bool ShowDirections;
    public byte PixelsPerCell;
    public float HeightOffset;
    public float Opacity;
}

public struct UnitSpatialMap : IComponentData
{
    public NativeParallelMultiHashMap<int, Entity> Map;
}
