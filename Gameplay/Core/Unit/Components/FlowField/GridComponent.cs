using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField
{

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
    public NativeArray<FlowFieldCell> PendingGrid;
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
/// 单次全局移动订单。组件启用时由 RtsCommandSystem 消费，随后立即禁用。
/// </summary>
public struct MoveOrder : IComponentData, IEnableableComponent
{
    public float3 TargetPosition;
}

/// <summary>
/// 下达 MoveOrder 时的选中单位快照。消费命令时不能改查实时 UnitSelected，
/// 否则输入与预测更新的时序差会把订单绑到之后的选择上。
/// </summary>
public struct MoveOrderSelectionElement : IBufferElementData
{
    public Entity Entity;
}

/// <summary>
/// 只在一份完整流场发布后递增，移动和可视化系统据此读取稳定快照。
/// </summary>
public struct FlowFieldRuntimeState : IComponentData
{
    public uint ActiveVersion;
    public uint ActiveRequestVersion;
}

/// <summary>
/// Cost 只随障碍物布局变化，改目标点不影响这个状态。
/// 动态墙壁变化时把 IsDirty 置 true，并请求一次流场重算。
/// </summary>
public struct FlowFieldCostState : IComponentData
{
    public bool IsDirty;
    public uint CostVersion;
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
}
