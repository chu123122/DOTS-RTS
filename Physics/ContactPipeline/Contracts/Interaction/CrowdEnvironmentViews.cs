using Unity.Collections;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 无容器的栅格几何体，可安全嵌入 Burst Job。NativeArray 仍是直接 Job 字段，保证 Collections Safety 看不到嵌套 NativeContainer。
/// </summary>
public readonly struct FlowGridGeometry
{
    public readonly float3 Origin;
    public readonly int2 Dimensions;
    public readonly float CellRadius;

    public FlowGridGeometry(float3 origin, int2 dimensions, float cellRadius)
    {
        Origin = origin;
        Dimensions = dimensions;
        CellRadius = cellRadius;
    }

    public bool Contains(int2 cell) =>
        cell.x >= 0 && cell.x < Dimensions.x &&
        cell.y >= 0 && cell.y < Dimensions.y;

    public int FlatIndex(int2 cell) =>
        FlatIndex(cell, Dimensions);

    public static int FlatIndex(int2 cell, int2 dimensions) =>
        cell.y * dimensions.x + cell.x;

    public int2 WorldToCell(float3 position)
        => WorldToCell(position, Origin, CellRadius);

    public static int2 WorldToCell(
        float3 position,
        float3 origin,
        float cellRadius)
    {
        float cellSize = cellRadius * 2f;
        float3 local = position - origin;
        return new int2(
            (int)(local.x / cellSize),
            (int)(local.z / cellSize));
    }

    public float3 CellCenter(int2 cell, float y) =>
        Origin + new float3(
            cell.x * CellRadius * 2f + CellRadius,
            y,
            cell.y * CellRadius * 2f + CellRadius);
}

/// <summary>
/// 与 NavigationField 分离的 Crowd 碰撞占据单元。
/// </summary>
public struct CrowdObstacleCell
{
    public byte IsBlocked;
}

/// <summary>
/// step 开始时发布的只读障碍快照；版本参与跨帧缓存失效。
/// NativeArray 只在调度门面使用，具体 Job 仍接收直接容器字段。
/// </summary>
public readonly struct CrowdObstacleSnapshot
{
    [ReadOnly] public readonly NativeArray<CrowdObstacleCell> Cells;
    public readonly FlowGridGeometry Geometry;
    public readonly uint Version;

    public CrowdObstacleSnapshot(
        NativeArray<CrowdObstacleCell> cells,
        FlowGridGeometry geometry,
        uint version)
    {
        Cells = cells;
        Geometry = geometry;
        Version = version;
    }
}

/// <summary>只解释碰撞占据，不再读取 FlowField cost/integration 语义。</summary>
public static class GridObstacleView
{
    public static bool IsBlocked(
        NativeArray<CrowdObstacleCell> cells,
        FlowGridGeometry geometry,
        int2 cell)
    {
        return cells.IsCreated && geometry.Contains(cell) &&
               cells[geometry.FlatIndex(cell)].IsBlocked != 0;
    }

    public static float3 CellCenter(
        FlowGridGeometry geometry,
        int2 cell,
        float y) => geometry.CellCenter(cell, y);
}
}
