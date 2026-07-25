using Unity.Mathematics;
namespace RTS.Unit.FlowField.Jobs
{
public partial struct ConstraintSolverJob
{

    private void BuildContactHeatSamples()
    {
        if (!EnableDiagnostics)
            return;

        for (int bodyIndex = 0; bodyIndex < Bodies.Length; bodyIndex++)
        {
            CrowdBodySnapshot stateSnapshot = Bodies[bodyIndex];
            CrowdNavigationState stateNavigation = NavigationStates[bodyIndex];
            CrowdMotionIntent stateIntent = MotionIntents[bodyIndex];
            CrowdMotionEvidence stateEvidence = MotionEvidence[bodyIndex];
            CrowdBodyStepState stateStep = StepStates[bodyIndex];
            HeatSamples[bodyIndex] = new Stage3ContactHeatSample
            {
                Entity = stateSnapshot.Entity,
                Position = stateStep.SolvedPosition,
                ContactCorrection = math.length(stateEvidence.ContactCorrection),
                Escaped = stateEvidence.EnvelopeEscaped
            };
        }

        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            ContactConstraint pair = TimestepContactPairs[pairIndex];
            AccumulateHeatPair(pair.BodyA, pair);
            AccumulateHeatPair(pair.BodyB, pair);
        }
    }

    private void AccumulateHeatPair(int bodyIndex, ContactConstraint pair)
    {
        Stage3ContactHeatSample sample = HeatSamples[bodyIndex];
        sample.ContactPairDegree++;
        if (pair.WasActivatedThisTimestep != 0)
            sample.ActivePairDegree++;
        if (pair.ContactMode == ContactConstraintMode.Predictive)
            sample.PredictivePairDegree++;
        if (pair.WasAddedByFallback != 0)
            sample.HasFallbackPair = 1;
        HeatSamples[bodyIndex] = sample;
    }
}
}
