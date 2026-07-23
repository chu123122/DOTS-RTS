namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Builds a deterministic CSR adjacency for the frame-local active contact view.
/// This is intentionally separate from the future persistent Entity-to-pair index.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private void InvalidateActiveConstraintIncidentIndex()
    {
        ActiveConstraintIncidentIndexDirty.Value = 1;
    }

    private void EnsureActiveConstraintIncidentIndex()
    {
        if (ActiveConstraintIncidentIndexDirty.Value == 0)
            return;

        int bodyCount = States.Length;
        for (int bodyIndex = 0; bodyIndex <= bodyCount; bodyIndex++)
            ActiveConstraintIncidentOffsets[bodyIndex] = 0;

        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            ActiveConstraintIncidentOffsets[pair.BodyA + 1]++;
            ActiveConstraintIncidentOffsets[pair.BodyB + 1]++;
        }

        for (int bodyIndex = 0; bodyIndex < bodyCount; bodyIndex++)
        {
            ActiveConstraintIncidentOffsets[bodyIndex + 1] +=
                ActiveConstraintIncidentOffsets[bodyIndex];
            ActiveConstraintIncidentWriteCursors[bodyIndex] =
                ActiveConstraintIncidentOffsets[bodyIndex];
        }

        int incidentCount = ActiveConstraintIncidentOffsets[bodyCount];
        ActiveConstraintIncidentPairIndices.ResizeUninitialized(incidentCount);
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            int writeA = ActiveConstraintIncidentWriteCursors[pair.BodyA]++;
            int writeB = ActiveConstraintIncidentWriteCursors[pair.BodyB]++;
            ActiveConstraintIncidentPairIndices[writeA] = pairIndex;
            ActiveConstraintIncidentPairIndices[writeB] = pairIndex;
        }

        ActiveConstraintIncidentIndexDirty.Value = 0;
    }
}
}
