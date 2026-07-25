using Unity.Mathematics;
using RTS.Unit.FlowField;

namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Static obstacle projection, separate from pair topology and lifecycle. The
/// current backend is the grid obstacle view; navigation-cell costs are not part
/// of this stage's public semantics.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
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
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            if (!(stateSnapshot.IsInsideSimulationDomain != 0) || stateSnapshot.InverseMass <= 0f)
                continue;

            int2 currentCell = EnvironmentGeometry.WorldToCell(
                stateStep.SolvedPosition);

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    int2 checkCell = currentCell + new int2(x, y);
                    if (!IsObstacleCell(checkCell))
                        continue;

                    float3 wallPosition = ObstacleCellCenter(
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
                        : DeterministicFallbackNormal(
                            bodyIndex,
                            EnvironmentGeometry.FlatIndex(checkCell));
                    float3 correction =
                        normal * ((hardDistance - distance) * 0.5f);
                    stateStep.SolvedPosition += correction;
                    stateStep.SolvedPosition.y = stateSnapshot.Position.y;
                    stateStep.WallCorrection += correction;
                    stateEvidence.WallCorrection += correction;

                    float correctionLength = math.length(correction);
                    totalPositionCorrection += correctionLength;
                    maxPositionCorrection = math.max(
                        maxPositionCorrection,
                        correctionLength);
                    if (trackCorrectedBodies)
                        MarkCorrectedBody(bodyIndex);
                }
            }

            Bodies[bodyIndex] = stateSnapshot;
            NavigationStates[bodyIndex] = stateNavigation;
            MotionIntents[bodyIndex] = stateIntent;
            MotionEvidence[bodyIndex] = stateEvidence;
            StepStates[bodyIndex] = stateStep;
        }
    }
}
}
