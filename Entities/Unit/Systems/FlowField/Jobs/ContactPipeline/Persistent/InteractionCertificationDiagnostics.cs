using Unity.Mathematics;
using RTS.Unit.FlowField.Diagnostics;
namespace RTS.Unit.FlowField.Jobs
{
public partial struct InteractionCertificationJob
{

    private void AddSelectedPairDiagnostic(
        ContactConstraint pair,
        Stage3ContactDiagnosticPairKind kind,
        float closestTime,
        float minimumDistance,
        float radiusSum,
        byte wasActivated)
    {
        int selectedBodyIndex = -1;
        if (DiagnosticSelectedEntity != Unity.Entities.Entity.Null)
        {
            for (int i = 0; i < Bodies.Length; i++)
            {
                if (Bodies[i].Entity == DiagnosticSelectedEntity)
                {
                    selectedBodyIndex = i;
                    break;
                }
            }
        }
        if (selectedBodyIndex < 0 ||
            (pair.BodyA != selectedBodyIndex && pair.BodyB != selectedBodyIndex))
            return;

        int otherBodyIndex = pair.BodyA == selectedBodyIndex ? pair.BodyB : pair.BodyA;
        CrowdMotionEvidence selectedEvidence = MotionEvidence[selectedBodyIndex];
        CrowdBodySnapshot otherSnapshot = Bodies[otherBodyIndex];
        CrowdMotionEvidence otherEvidence = MotionEvidence[otherBodyIndex];
        float3 selectedClosest = math.lerp(
            selectedEvidence.TrajectoryStart,
            selectedEvidence.BaselineEnd,
            closestTime);
        float3 otherClosest = math.lerp(
            otherEvidence.TrajectoryStart,
            otherEvidence.BaselineEnd,
            closestTime);

        PairDiagnostics.Add(new Stage3ContactPairDiagnostic
        {
            OtherEntity = otherSnapshot.Entity,
            Kind = kind,
            WasActivated = wasActivated,
            WasAddedByFallback = pair.WasAddedByFallback,
            FirstActivatedSubstep = pair.FirstActivatedSubstep,
            ActivatedSubstepCount = pair.ActivatedSubstepCount,
            ClosestTime = closestTime,
            MinimumDistance = minimumDistance,
            RadiusSum = radiusSum,
            OtherRadius = otherSnapshot.Radius,
            OtherStartPosition = otherEvidence.TrajectoryStart,
            OtherPredictedPosition = otherEvidence.BaselineEnd,
            SelectedClosestPosition = selectedClosest,
            OtherClosestPosition = otherClosest
        });
    }

    private void ResetCorrectedBodyTracking()
    {
        for (int i = 0; i < CorrectedBodyIndices.Length; i++)
            CorrectedBodyFlags[CorrectedBodyIndices[i]] = 0;
        CorrectedBodyIndices.Clear();
    }



    private void MarkCorrectedBody(int bodyIndex)
    {
        if (CorrectedBodyFlags[bodyIndex] != 0)
            return;
        CorrectedBodyFlags[bodyIndex] = 1;
        CorrectedBodyIndices.Add(bodyIndex);
    }
}
}
