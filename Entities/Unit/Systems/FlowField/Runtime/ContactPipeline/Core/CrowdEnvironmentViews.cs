using Unity.Collections;
using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Container-free grid geometry that is safe to embed in Burst jobs. NativeArray
/// storage remains a direct job field so Collections Safety never sees a nested
/// NativeContainer.
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
/// Navigation semantics over the shared FlowField storage. Navigation code should
/// use this API instead of interpreting collision policy.
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
/// Collision-environment semantics over the same backing cells. Soft-wall and
/// hard-wall stages depend on IsBlocked/CellCenter, not on navigation costs.
/// A future obstacle backend can replace this implementation without changing
/// navigation intent generation.
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
