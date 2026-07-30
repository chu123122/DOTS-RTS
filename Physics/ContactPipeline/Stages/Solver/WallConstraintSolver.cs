using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// 静态障碍投影，与对拓扑与生命周期解耦。当前后端是 grid 障碍视图；导航 cell 代价不在本阶段的公开语义里。
/// </summary>
public partial struct ConstraintSolverJob
{
    private void SolveWallConstraintIteration(
        bool trackCorrectedBodies,
        out float totalPositionCorrection,
        out float maxPositionCorrection)
    {
        if (trackCorrectedBodies)
            ResetCorrectedBodyTracking();

        totalPositionCorrection = 0f;
        maxPositionCorrection = 0f;
        if (!Grid.IsCreated)
            return;

        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdSolverBodyState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0) || stateSnapshot.InverseMass <= 0f)
                continue;

            int2 currentCell = EnvironmentGeometry.WorldToCell(
                stateStep.SolvedPosition);

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    int2 checkCell = currentCell + new int2(x, y);
                    if (!GridObstacleView.IsBlocked(Grid, EnvironmentGeometry, checkCell))
                        continue;

                    float3 wallPosition = GridObstacleView.CellCenter(EnvironmentGeometry,
                        checkCell,
                        stateStep.SolvedPosition.y);
                    float3 delta = stateStep.SolvedPosition - wallPosition;
                    delta.y = 0f;
                    float distance = math.length(delta);
                    float hardDistance =
                        CellRadius + math.max(0f, stateSnapshot.Radius);
                    if (distance >= hardDistance)
                        continue;

                    float3 normal = distance > 0.00001f
                        ? delta / distance
                        : ContactPipelineMath.DeterministicFallbackNormal(
                            bodyIndex,
                            EnvironmentGeometry.FlatIndex(checkCell));
                    float3 correction =
                        normal * ((hardDistance - distance) * 0.5f);
                    stateStep.SolvedPosition += correction;
                    stateStep.SolvedPosition.y = stateSnapshot.Position.y;
                    stateStep.WallCorrection += correction;
                    stateStep.TimestepWallCorrection += correction;

                    float correctionLength = math.length(correction);
                    totalPositionCorrection += correctionLength;
                    maxPositionCorrection = math.max(
                        maxPositionCorrection,
                        correctionLength);
                    if (trackCorrectedBodies)
                        MarkCorrectedBody(bodyIndex);
                }
            }

            StepStates[bodyIndex] = stateStep;
        }
    }
}
}
