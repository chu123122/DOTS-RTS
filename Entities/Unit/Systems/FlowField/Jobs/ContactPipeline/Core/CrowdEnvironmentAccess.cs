using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Semantic environment access shared by serial contact stages. The backing grid
/// remains unchanged; callers no longer interpret FlowField cost directly.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private FlowGridGeometry EnvironmentGeometry =>
        new FlowGridGeometry(GridOrigin, GridDimensions, CellRadius);

    private bool IsObstacleCell(int2 cell) =>
        GridObstacleView.IsBlocked(Grid, EnvironmentGeometry, cell);

    private float3 ObstacleCellCenter(int2 cell, float y) =>
        GridObstacleView.CellCenter(EnvironmentGeometry, cell, y);
}
}
