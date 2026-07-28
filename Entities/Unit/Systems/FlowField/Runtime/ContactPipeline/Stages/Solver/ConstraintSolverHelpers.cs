namespace RTS.Unit.FlowField.Jobs
{
public partial struct ConstraintSolverJob
{

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

    private void ResetTimestepContactSetForSubstep()
    {
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            ContactConstraint pair = TimestepContactPairs[pairIndex];
            pair.Lambda = 0f;
            pair.WasActivated = 0;
            TimestepContactPairs[pairIndex] = pair;
        }
    }

#if RTS_CONTACT_DIAGNOSTICS
    private void CountFinalContactSetUtilization(
        out int activatedPairCount,
        out int correctedPairCount)
    {
        activatedPairCount = 0;
        correctedPairCount = 0;
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            ContactConstraint pair = TimestepContactPairs[pairIndex];
            activatedPairCount += pair.WasActivatedThisTimestep != 0 ? 1 : 0;
            correctedPairCount += pair.WasCorrectedThisTimestep != 0 ? 1 : 0;
        }
    }
#endif
}
}
