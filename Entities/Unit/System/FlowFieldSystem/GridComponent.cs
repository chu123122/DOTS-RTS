using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

// --- 存储在每个网格单元中的数据 ---
public struct FlowFieldCell
{
    // 静态数据 (在 "烘焙" 障碍物时设置一次)
    public byte Cost; // 0 = 不可通行 (障碍物), 1 = 可通行 (默认)

    // 动态数据 (在寻路时计算)
    
    // 积分场 (Integration Field)
    // 从目标点开始的 "代价" 或 "距离"。
    // 我们用 ushort (0-65535) 来节省空间，MaxValue 代表 "未访问"。
    public ushort IntegrationValue; 

    // 向量场 (Vector Field)
    // 存储一个指向 "成本最低" 邻居的方向。
    // 用一个 byte (0-7) 索引来代表 8 个方向 (N, NE, E, SE, S, SW, W, NW)
    // 0xFF (255) 代表 "无方向" (例如障碍物或目标点本身)
    public byte BestDirectionIndex;
}

// --- 单例组件: 持有整个网格 ---
// (将这个组件 Add 到一个 "FlowFieldManager" Entity 上)
public struct FlowFieldGrid : IComponentData
{
    // 网格元数据
    public float3 GridOrigin;   // 世界空间的 (0,0,0) 点
    public int2 GridDimensions; // 单元格数量 (例如 200x200)
    public float CellRadius;     // 每个单元格的半径 (例如 0.5f)
    
    // 核心数据 (必须在 Job 完成后 Dispose)
    // 我们将把 2D 网格展平 (Flatten) 为 1D 数组
    public NativeArray<FlowFieldCell> Grid;
}
public struct FlowFieldSettings : IComponentData
{
    public float3 GridOrigin;
    public int2 GridDimensions;
    public float CellRadius;
}

// 放在 FlowFieldGridSystem.cs 或单独文件
public struct FlowFieldGlobalTarget : IComponentData
{
    public float3 TargetPosition; // 当前地图流动的汇聚点
    // public int TargetHash;     // (可选) 用于版本控制，防止旧单位跟随新场
}