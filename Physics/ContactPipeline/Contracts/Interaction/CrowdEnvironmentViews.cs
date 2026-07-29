using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField;

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
        FlowFieldUtils.GetFlatIndex(cell, Dimensions);

    public int2 WorldToCell(float3 position) =>
        FlowFieldUtils.WorldToCell(position, Origin, CellRadius);

    public float3 CellCenter(int2 cell, float y) =>
        Origin + new float3(
            cell.x * CellRadius * 2f + CellRadius,
            y,
            cell.y * CellRadius * 2f + CellRadius);
}

/// <summary>
/// 共享 FlowField 存储之上的导航语义。导航代码应使用该 API 而非解读碰撞策略。
/// </summary>
public static class FlowNavigationView
{
    public static bool TryRead(
        NativeArray<FlowFieldCell> cells,
        FlowGridGeometry geometry,
        int2 cell,
        out FlowFieldCell value)
    {
        if (!cells.IsCreated || !geometry.Contains(cell))
        {
            value = default;
            return false;
        }

        value = cells[geometry.FlatIndex(cell)];
        return true;
    }

    public static bool IsReachable(FlowFieldCell cell) =>
        cell.Cost != 0 && cell.IntegrationValue != ushort.MaxValue;
}

/// <summary>
/// 同一后端 cells 之上的碰撞环境语义。软墙/硬墙阶段依赖 IsBlocked/CellCenter，而非导航代价。
/// 未来障碍后端可在不影响导航意图生成的前提下替换此实现。
/// </summary>
public static class GridObstacleView
{
    public static bool IsBlocked(
        NativeArray<FlowFieldCell> cells,
        FlowGridGeometry geometry,
        int2 cell)
    {
        return cells.IsCreated && geometry.Contains(cell) &&
               cells[geometry.FlatIndex(cell)].Cost == 0;
    }

    public static float3 CellCenter(
        FlowGridGeometry geometry,
        int2 cell,
        float y) => geometry.CellCenter(cell, y);
}
}
