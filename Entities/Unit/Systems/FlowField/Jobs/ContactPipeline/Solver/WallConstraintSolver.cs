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

        for (int bodyIndex = 0; bodyIndex < States.Length; bodyIndex++)
        {
            FlowMovementFrameState state = States[bodyIndex];
            if (!state.IsInsideGrid || state.InverseMass <= 0f)
                continue;

            int2 currentCell = EnvironmentGeometry.WorldToCell(
                state.PredictedPosition);

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    int2 checkCell = currentCell + new int2(x, y);
                    if (!IsObstacleCell(checkCell))
                        continue;

                    float3 wallPosition = ObstacleCellCenter(
                        checkCell,
                        state.PredictedPosition.y);
                    float3 delta = state.PredictedPosition - wallPosition;
                    delta.y = 0f;
                    float distance = math.length(delta);
                    float hardDistance =
                        CellRadius + math.max(0f, state.Radius);
                    if (distance >= hardDistance)
                        continue;

                    float3 normal = distance > 0.00001f
                        ? delta / distance
                        : DeterministicFallbackNormal(bodyIndex, checkCell.GetHashCode());
                    float3 correction =
                        normal * ((hardDistance - distance) * 0.5f);
                    state.PredictedPosition += correction;
                    state.PredictedPosition.y = state.CurrentPosition.y;
                    state.WallPositionCorrection += correction;
                    state.TimestepWallCorrection += correction;

                    float correctionLength = math.length(correction);
                    totalPositionCorrection += correctionLength;
                    maxPositionCorrection = math.max(
                        maxPositionCorrection,
                        correctionLength);
                    if (trackCorrectedBodies)
                        MarkCorrectedBody(bodyIndex);
                }
            }

            States[bodyIndex] = state;
        }
    }
}
}
