using Unity.Entities;
using Unity.Mathematics;

namespace RTS.Unit.FlowField.Jobs
{
public partial struct InteractionCertificationJob
{
    private void PrepareCurrentBodyLookup()
    {
        CurrentBodyIndexByEntity.Clear();
        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
            CurrentBodyIndexByEntity.TryAdd(Bodies[bodyIndex].Entity, bodyIndex);
    }


    private bool TryFindCurrentBodyIndex(Entity entity, out int bodyIndex)
    {
        return CurrentBodyIndexByEntity.TryGetValue(entity, out bodyIndex) &&
               bodyIndex >= 0 && bodyIndex < Bodies.Length;
    }


    private void CalculateNeighborPathBounds(
        CrowdMotionEvidence evidence,
        CrowdBodyStepState step,
        out float2 pathMin,
        out float2 pathMax)
    {
        pathMin = math.min(
            evidence.TrajectoryStart.xz,
            math.min(
                evidence.BaselineEnd.xz,
                math.min(step.UnconstrainedPosition.xz, step.SolvedPosition.xz)));
        pathMax = math.max(
            evidence.TrajectoryStart.xz,
            math.max(
                evidence.BaselineEnd.xz,
                math.max(step.UnconstrainedPosition.xz, step.SolvedPosition.xz)));
        if (SoftAvoidanceVelocitySolver !=
                SoftAvoidanceVelocitySolverMode.ReciprocalVelocityObstacle ||
            SoftAvoidanceShell <= 0f || SoftAvoidanceResponseRate <= 0f)
            return;

        float2 horizonEnd = step.SolvedPosition.xz +
                            step.BaseVelocity.xz * math.max(0f, RvoTimeHorizon);
        pathMin = math.min(pathMin, horizonEnd);
        pathMax = math.max(pathMax, horizonEnd);
    }
}
}
