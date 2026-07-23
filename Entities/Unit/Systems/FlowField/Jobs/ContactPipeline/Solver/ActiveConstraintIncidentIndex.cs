namespace RTS.Unit.FlowField.Jobs
{
/// <summary>
/// Frame-local CSR index from a body slot to active timestep contact-pair indices.
/// It is rebuilt only when the active contact view changes and is shared by the
/// serial reference and parallel Jacobi gather paths.
/// </summary>
public partial struct SolveXpbdUnitContactsJob
{
    private void RebuildActiveConstraintIncidentIndexIfNeeded()
    {
        if (ContactPositionSolver != ContactPositionSolverMode.Jacobi)
            return;

        int bodyCount = States.Length;
        for (int bodyIndex = 0; bodyIndex < bodyCount; bodyIndex++)
            ActiveIncidentWriteCursors[bodyIndex] = 0;

        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            ActiveIncidentWriteCursors[pair.BodyA]++;
            ActiveIncidentWriteCursors[pair.BodyB]++;
        }

        int entryCount = 0;
        ActiveIncidentOffsets[0] = 0;
        for (int bodyIndex = 0; bodyIndex < bodyCount; bodyIndex++)
        {
            entryCount += ActiveIncidentWriteCursors[bodyIndex];
            ActiveIncidentOffsets[bodyIndex + 1] = entryCount;
            ActiveIncidentWriteCursors[bodyIndex] =
                ActiveIncidentOffsets[bodyIndex];
        }

        ActiveIncidentPairIndices.ResizeUninitialized(entryCount);
        for (int pairIndex = 0; pairIndex < TimestepContactPairs.Length; pairIndex++)
        {
            UnitCollisionPair pair = TimestepContactPairs[pairIndex];
            int slotA = ActiveIncidentWriteCursors[pair.BodyA]++;
            int slotB = ActiveIncidentWriteCursors[pair.BodyB]++;
            ActiveIncidentPairIndices[slotA] = pairIndex;
            ActiveIncidentPairIndices[slotB] = pairIndex;
        }
    }
}
}
